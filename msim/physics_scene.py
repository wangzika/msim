"""Strict static SDF collision import. Unsupported geometry is never dropped."""
from pathlib import Path
from urllib.parse import unquote, urlparse
from urllib.request import url2pathname
import xml.etree.ElementTree as ET
import numpy as np


def pose(text='0 0 0 0 0 0'):
    v = np.fromstring(text, sep=' ')
    if len(v) != 6 or not np.isfinite(v).all():
        raise ValueError('SDF pose must contain six finite values')
    x,y,z,r,p,a=v; cr,sr,cp,sp,ca,sa=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(a),np.sin(a)
    result=np.eye(4)
    result[:3,:3]=[[ca*cp,ca*sp*sr-sa*cr,ca*sp*cr+sa*sr],
                   [sa*cp,sa*sp*sr+ca*cr,sa*sp*cr-ca*sr],[-sp,cp*sr,cp*cr]]
    result[:3,3]=[x,y,z]
    return result


def node_pose(node):
    element=node.find('pose')
    if element is not None and element.get('relative_to'):
        raise ValueError('SDF relative_to frames require explicit conversion')
    return pose(node.findtext('pose','0 0 0 0 0 0'))


def load_collada(path):
    """Same pinned Z_UP triangle/matrix subset used by tools/ksql_mesh.py.

    Reject other encodings rather than loading a subtly different airport.
    """
    root=ET.parse(path).getroot(); ns={'c':root.tag.split('}')[0][1:]}
    if root.findtext('c:asset/c:up_axis',namespaces=ns) != 'Z_UP':
        raise ValueError('COLLADA adapter currently requires Z_UP')
    unit_node=root.find('c:asset/c:unit',ns)
    unit=float(unit_node.get('meter','1')) if unit_node is not None else 1.
    geometries={}
    for geo in root.findall('.//c:library_geometries/c:geometry',ns):
        mesh=geo.find('c:mesh',ns)
        if mesh is None:raise ValueError('Unsupported COLLADA geometry')
        inputs=mesh.findall('c:vertices/c:input',ns)
        source=next(x.get('source')[1:] for x in inputs if x.get('semantic')=='POSITION')
        vertices=np.fromstring(mesh.find(f"c:source[@id='{source}']/c:float_array",ns).text,sep=' ').reshape(-1,3)
        faces=[]
        for tri in mesh.findall('c:triangles',ns):
            ins=tri.findall('c:input',ns); stride=max(int(x.get('offset','0')) for x in ins)+1
            offset=next(int(x.get('offset','0')) for x in ins if x.get('semantic')=='VERTEX')
            index=np.fromstring(tri.findtext('c:p',namespaces=ns),sep=' ',dtype=np.int64)
            faces.append(index.reshape(-1,stride)[:,offset].reshape(-1,3))
        if mesh.findall('c:polylist',ns) or mesh.findall('c:polygons',ns) or not faces:
            raise ValueError('COLLADA requires triangle primitives')
        geometries[geo.get('id')]=(vertices,np.concatenate(faces))
    output=[]
    def visit(node,parent):
        matrix=parent.copy()
        for child in node:
            tag=child.tag.split('}')[-1]
            if tag=='matrix':matrix=matrix @ np.fromstring(child.text,sep=' ').reshape(4,4)
            elif tag in ('translate','rotate','scale','instance_node','instance_controller'):
                raise ValueError('COLLADA transform/instance unsupported; preconvert scene first')
        for instance in node.findall('c:instance_geometry',ns):
            vertices,faces=geometries[instance.get('url')[1:]]
            output.append((unit*(vertices @ matrix[:3,:3].T+matrix[:3,3]))[faces])
        for child in node.findall('c:node',ns):visit(child,matrix)
    active=root.find('c:scene/c:instance_visual_scene',ns)
    if active is None:raise ValueError('COLLADA has no active visual scene')
    visual=root.find(f"c:library_visual_scenes/c:visual_scene[@id='{active.get('url')[1:]}']",ns)
    if visual is None:raise ValueError('COLLADA visual scene missing')
    for node in visual.findall('c:node',ns):visit(node,np.eye(4))
    if not output:raise ValueError('Empty COLLADA collision mesh')
    triangles=np.concatenate(output)
    if not np.isfinite(triangles).all():raise ValueError('Non-finite mesh coordinates')
    return triangles


def load_world(filename,model_roots=()):
    filename=Path(filename).resolve();world=ET.parse(filename).getroot().find('world')
    if world is None:raise ValueError('Expected SDF world')
    objects=[]
    def visit(parent,transform,base,depth=0,static=False):
        if depth>12:raise ValueError('SDF nesting exceeds 12 levels')
        if parent.findall('actor'):raise ValueError('Dynamic SDF actors are not supported by this static importer')
        for include in parent.findall('include'):
            uri=include.findtext('uri','')
            if uri=='model://sun':continue
            if uri=='model://ground_plane':
                objects.append(dict(name='ground_plane',kind='plane',matrix=(transform@node_pose(include)).tolist(),normal=[0,0,1]));continue
            if not uri.startswith('model://'):raise ValueError('Unsupported include '+uri)
            candidates=[Path(r)/uri[8:] for r in model_roots]
            target=next((p/n for p in candidates for n in ('model.sdf',p.name+'.sdf') if (p/n).is_file()),None)
            if target is None:raise ValueError('Missing model; specify --model-root: '+uri)
            model=ET.parse(target).getroot().find('model')
            if model is None:raise ValueError('Included model has no model element')
            old=model.find('pose')
            if old is not None:model.remove(old)
            holder=ET.Element('holder');holder.append(model)
            visit(holder,transform@node_pose(include),target.parent,depth+1,static)
        for model in parent.findall('model'):
            fixed=static or model.findtext('static','false').lower() in ('true','1')
            if not fixed:raise ValueError('Dynamic scene objects require explicit backend support: '+model.get('name',''))
            mt=transform@node_pose(model)
            for link in model.findall('link'):
                lt=mt@node_pose(link)
                for collision in link.findall('collision'):
                    geometry=collision.find('geometry');t=lt@node_pose(collision)
                    if geometry is None or len(geometry)!=1:raise ValueError('Missing/ambiguous collision geometry')
                    kind=geometry[0].tag
                    item=dict(name=model.get('name','')+'/'+collision.get('name',''),kind=kind,matrix=t.tolist())
                    if kind=='box':item['size']=list(map(float,geometry.findtext('box/size').split()))
                    elif kind=='sphere':item['radius']=float(geometry.findtext('sphere/radius'))
                    elif kind=='cylinder':
                        item['radius']=float(geometry.findtext('cylinder/radius'));item['length']=float(geometry.findtext('cylinder/length'))
                    elif kind=='plane':item['normal']=list(map(float,geometry.findtext('plane/normal','0 0 1').split()))
                    elif kind=='mesh':
                        mesh=geometry.find('mesh');uri=unquote(mesh.findtext('uri',''))
                        if mesh.find('submesh') is not None:raise ValueError('Mesh submesh selection unsupported')
                        if uri.startswith('file://'):path=Path(url2pathname(urlparse(uri).path))
                        elif uri.startswith('model://'):
                            path=next((Path(r)/uri[8:] for r in model_roots if (Path(r)/uri[8:]).is_file()),None)
                        else:path=base/uri
                        if path is None or not path.is_file():raise ValueError('Missing collision mesh '+uri)
                        item['path']=str(path.resolve());item['scale']=list(map(float,mesh.findtext('scale','1 1 1').split()))
                    else:raise ValueError('Unsupported collision geometry: '+kind)
                    for key in ('size','scale','normal'):
                        if key in item and (len(item[key])!=3 or not np.isfinite(item[key]).all()):raise ValueError('Invalid geometry '+key)
                    dimensions=[item[k] for k in ('radius','length') if k in item]+item.get('size',[])+item.get('scale',[])
                    if any(not np.isfinite(v) or v<=0 for v in dimensions):raise ValueError('Geometry dimensions must be positive')
                    objects.append(item)
            visit(model,mt,base,depth+1,fixed)
    visit(world,np.eye(4),filename.parent)
    if not objects:raise ValueError('World contains no supported collision geometry')
    gravity=np.fromstring(world.findtext('gravity',world.findtext('physics/gravity','0 0 -9.80665')),sep=' ')
    if len(gravity)!=3 or not np.isfinite(gravity).all():raise ValueError('Invalid gravity')
    return dict(objects=objects,gravity=gravity.tolist(),source=str(filename))
