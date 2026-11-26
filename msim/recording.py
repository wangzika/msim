"""Streaming JSONL metadata with optional per-scan, pickle-free NPZ payloads."""
import json
from pathlib import Path
from collections import Counter
import numpy as np
from .schema import Event


def safe_payload(root, name):
    if not isinstance(name,str) or '\\' in name or ':' in name:raise ValueError('Payload names must be portable relative POSIX paths')
    path=Path(name)
    if path.is_absolute() or '..' in path.parts:raise ValueError('Payload must stay inside the recording')
    root=Path(root).resolve();target=(root/path).resolve()
    if not target.is_relative_to(root) or target.suffix!='.npz':raise ValueError('Invalid cloud payload')
    return target


class RecordingWriter:
    def __init__(self, destination, metadata):
        self.root=Path(destination);self.root.mkdir(parents=True,exist_ok=False)
        self.metadata=dict(metadata,schema_version=1,complete=False)
        self.file=(self.root/'events.jsonl').open('w',encoding='utf-8',newline='\n')
        self.previous=None;self.counts=Counter();self.clouds=0
        self._manifest()

    def _manifest(self):
        temporary=self.root/'manifest.json.tmp'
        temporary.write_text(json.dumps(dict(self.metadata,counts=dict(self.counts)),ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        temporary.replace(self.root/'manifest.json')

    def write(self, event):
        event.validate()
        if self.previous is not None and event.arrival_ns<self.previous:raise ValueError('Unsorted availability times')
        self.file.write(json.dumps(event.row(),ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n')
        self.previous=event.arrival_ns;self.counts[event.stream]+=1

    def cloud(self, stamp_ns, arrival_ns, stream, frame, fields):
        if not fields or not all(k in fields for k in ('x','y','z')):raise ValueError('Cloud requires XYZ')
        arrays={k:np.asarray(v) for k,v in fields.items()}
        length=len(arrays['x'])
        if any(v.ndim!=1 or len(v)!=length or v.dtype.kind not in 'iuf' for v in arrays.values()):raise ValueError('Invalid cloud fields')
        self.clouds+=1;name=f'clouds/{self.clouds:08d}.npz';path=safe_payload(self.root,name);path.parent.mkdir(exist_ok=True)
        np.savez_compressed(path,**arrays)
        self.write(Event(stamp_ns,arrival_ns,'cloud',stream,frame,dict(file=name,point_count=length,fields=list(arrays))))

    def close(self, complete=True):
        self.file.close();self.metadata['complete']=complete;self._manifest()

    def __enter__(self):return self
    def __exit__(self, kind, *_):self.close(complete=kind is None)


class Recording:
    def __init__(self, root):
        self.root=Path(root)
        self.metadata=json.loads((self.root/'manifest.json').read_text(encoding='utf-8'))
        if type(self.metadata.get('schema_version')) is not int or self.metadata['schema_version']!=1 or self.metadata.get('complete') is not True:raise ValueError('Unsupported or incomplete recording')

    def events(self):
        previous=None
        with (self.root/'events.jsonl').open(encoding='utf-8') as stream:
            for line_number,line in enumerate(stream,1):
                try:event=Event.from_row(json.loads(line))
                except (ValueError,TypeError) as error:raise ValueError(f'events.jsonl:{line_number}: {error}') from error
                if previous is not None and event.arrival_ns<previous:raise ValueError('Unsorted availability times')
                if event.kind=='cloud' and not safe_payload(self.root,event.data['file']).is_file():raise ValueError('Missing cloud payload')
                previous=event.arrival_ns;yield event

    def cloud(self, event):
        with np.load(safe_payload(self.root,event.data['file']),allow_pickle=False) as payload:
            result={key:payload[key] for key in payload.files}
        n=event.data['point_count']
        if set(result)!=set(event.data['fields']) or any(v.ndim!=1 or len(v)!=n or v.dtype.kind not in 'iuf' for v in result.values()):raise ValueError('Cloud payload does not match event')
        return result
