"""Serve scene visuals from the same SDF and mesh resources used by Gazebo."""
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from urllib.parse import unquote


ROOT=Path(__file__).resolve().parents[1]
MODEL_ROOTS=(ROOT/'build/deps/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models',ROOT/'build/px4-assets',ROOT/'scenes')
ALLOWED_SUFFIXES={'.dae','.stl','.png','.jpg','.jpeg','.webp','.tga','.bmp'}


def pose_matrix(text='0 0 0 0 0 0'):
    values=list(map(float,text.split()))
    if len(values)!=6 or not all(map(math.isfinite,values)):raise ValueError('Invalid SDF pose')
    x,y,z,r,p,a=values;cr,sr,cp,sp,ca,sa=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(a),math.sin(a)
    return [[ca*cp,ca*sp*sr-sa*cr,ca*sp*cr+sa*sr,x],
            [sa*cp,sa*sp*sr+ca*cr,sa*sp*cr-ca*sr,y],[-sp,cp*sr,cp*cr,z],[0,0,0,1]]


def multiply(a,b):return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
def column_major(matrix):return [matrix[i][j] for j in range(4) for i in range(4)]


class SceneResources:
    def __init__(self,world):
        self.assets={};self.objects=[];self.warnings=[];self.materialized={}
        self.allowed_roots=[ROOT.resolve()]
        configuration=ROOT/'config/run-storage.json'
        if configuration.is_file():self.allowed_roots.append(Path(json.loads(configuration.read_text())['run_root']).resolve())
        path=Path(world)
        if not path.is_file():return
        root=ET.parse(path).getroot();node=root.find('world')
        if node is None:return
        self.visit_children(node,pose_matrix(),path.parent,0)

    def allowed(self,path):
        path=Path(path).resolve()
        return path.is_file() and path.suffix.lower() in ALLOWED_SUFFIXES and any(path.is_relative_to(root) for root in self.allowed_roots)

    def resource(self,uri,base):
        if not uri:return None
        uri=unquote(uri.strip())
        if uri.startswith('model://'):
            relative=uri.removeprefix('model://')
            candidates=[root/relative for root in MODEL_ROOTS]
        else:
            value=uri.removeprefix('file://')
            candidates=[Path(value)] if value.startswith('/') else [base/value,base.parent/value,base/'textures'/Path(value).name]
        return next((p.resolve() for p in candidates if self.allowed(p)),None)

    def asset(self,path):
        path=Path(path).resolve();stat=path.stat()
        token=hashlib.sha256(f'{path}:{stat.st_mtime_ns}:{stat.st_size}'.encode()).hexdigest()[:24]+path.suffix.lower()
        self.assets[token]=path
        return token

    def model_include(self,include,base):
        uri=include.findtext('uri','')
        if uri=='model://sun':return None  # Replaced by the browser's scene lighting.
        if not uri.startswith('model://'):return None
        relative=uri.removeprefix('model://')
        for model_root in MODEL_ROOTS:
            folder=(model_root/relative).resolve()
            if not folder.is_relative_to(ROOT):continue
            candidates=[folder/'model.sdf',folder/(folder.name+'.sdf')]
            config=folder/'model.config'
            if config.is_file():
                candidates=[folder/entry.text for entry in ET.parse(config).getroot().findall('sdf') if entry.text]+candidates
            for candidate in candidates:
                if candidate.is_file() and candidate.resolve().is_relative_to(ROOT):return candidate
        self.warnings.append('未找到模型 '+uri);return None

    def visit_children(self,parent,transform,base,depth):
        if depth>12 or len(self.objects)>1000:return
        for model in parent.findall('model'):self.visit_model(model,transform,base,depth+1)
        for include in parent.findall('include'):
            path=self.model_include(include,base)
            if path is None:continue
            model=ET.parse(path).getroot().find('model')
            if model is None:continue
            # An include pose overrides the included model pose in SDF.
            pose=model.find('pose')
            if pose is None:pose=ET.SubElement(model,'pose')
            pose.text=include.findtext('pose','0 0 0 0 0 0')
            self.visit_model(model,transform,path.parent,depth+1)

    def visit_model(self,model,parent,base,depth):
        transform=multiply(parent,pose_matrix(model.findtext('pose','0 0 0 0 0 0')))
        for link in model.findall('link'):
            link_transform=multiply(transform,pose_matrix(link.findtext('pose','0 0 0 0 0 0')))
            visuals=link.findall('visual') or link.findall('collision')
            for visual in visuals:
                geometry=visual.find('geometry')
                if geometry is None:continue
                matrix=multiply(link_transform,pose_matrix(visual.findtext('pose','0 0 0 0 0 0')))
                item=dict(name=model.get('name','')+'/'+visual.get('name',''),matrix=column_major(matrix),color=[.55,.65,.7,1])
                diffuse=visual.findtext('material/diffuse')
                if diffuse:
                    values=list(map(float,diffuse.split()))
                    if len(values)==4:item['color']=values
                if geometry.find('mesh') is not None:
                    mesh=geometry.find('mesh');path=self.resource(mesh.findtext('uri'),base)
                    if path is None or path.suffix.lower() not in ('.dae','.stl'):
                        self.warnings.append('未加载网格 '+mesh.findtext('uri',''));continue
                    item.update(kind=path.suffix.lower()[1:],url='/api/scene-assets/'+self.asset(path),scale=list(map(float,mesh.findtext('scale','1 1 1').split())))
                    if item['kind']=='dae':
                        root=ET.parse(path).getroot();ns={'c':root.tag.split('}')[0][1:]}
                        item['up_axis']=root.findtext('c:asset/c:up_axis','Y_UP',ns)
                        self.prepare_collada(path,root,ns)
                elif geometry.find('box') is not None:item.update(kind='box',size=list(map(float,geometry.findtext('box/size').split())))
                elif geometry.find('cylinder') is not None:item.update(kind='cylinder',radius=float(geometry.findtext('cylinder/radius')),length=float(geometry.findtext('cylinder/length')))
                elif geometry.find('sphere') is not None:item.update(kind='sphere',radius=float(geometry.findtext('sphere/radius')))
                elif geometry.find('plane') is not None:item.update(kind='plane',size=list(map(float,geometry.findtext('plane/size','2000 2000').split())))
                else:continue
                self.objects.append(item)
        self.visit_children(model,transform,base,depth)

    def prepare_collada(self,path,root,ns):
        token=self.asset(path)
        if token in self.materialized:return
        for image in root.findall('c:library_images/c:image/c:init_from',ns):
            texture=self.resource(image.text,path.parent)
            if texture:
                image.text=self.asset(texture)
            else:
                self.warnings.append('未找到纹理 '+str(image.text));image.text='missing-texture.png'
        ET.register_namespace('',ns['c'])
        self.materialized[token]=ET.tostring(root,encoding='utf8',xml_declaration=True)

    def manifest(self):
        return dict(objects=self.objects,warnings=self.warnings,asset_count=len(self.assets),
                    mesh_count=sum(item['kind'] in ('dae','stl') for item in self.objects),coordinate_frame='Gazebo ENU')


@lru_cache(maxsize=4)
def _scene_resources(filename,modified,size):return SceneResources(filename)


def scene_resources(world):
    path=Path(world)
    if not path.is_file():return SceneResources(path)
    stat=path.stat();return _scene_resources(str(path),stat.st_mtime_ns,stat.st_size)
