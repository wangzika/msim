#!/usr/bin/env python3
"""Build two local, collision-enabled Gazebo Classic scene fixtures."""
import argparse
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


def text(parent,name,value):
    ET.SubElement(parent,name).text=str(value)


def solid(world,name,pose,kind,size,color):
    model=ET.SubElement(world,'model',name=name)
    text(model,'static','true');text(model,'pose',' '.join(map(str,pose)))
    link=ET.SubElement(model,'link',name='body')
    for tag in ('collision','visual'):
        item=ET.SubElement(link,tag,name=tag)
        shape=ET.SubElement(ET.SubElement(item,'geometry'),kind)
        if kind=='box':text(shape,'size',' '.join(map(str,size)))
        else:text(shape,'radius',size[0]);text(shape,'length',size[1])
        if tag=='visual':
            mat=ET.SubElement(item,'material')
            text(mat,'ambient',' '.join(map(str,(*color,1))))
            text(mat,'diffuse',' '.join(map(str,(*color,1))))
    return dict(name=name,pose=pose,kind=kind,size=size)


def camera(world):
    model=ET.SubElement(world,'model',name='msim_scene_camera')
    text(model,'static','true');text(model,'pose','40 -52 45 0 0.55 2.28')
    link=ET.SubElement(model,'link',name='camera_link')
    sensor=ET.SubElement(link,'sensor',name='overview',type='camera')
    text(sensor,'always_on','true');text(sensor,'update_rate',0.2)
    cam=ET.SubElement(sensor,'camera');text(cam,'horizontal_fov',1.15)
    img=ET.SubElement(cam,'image');text(img,'width',1280);text(img,'height',800);text(img,'format','R8G8B8')
    clip=ET.SubElement(cam,'clip');text(clip,'near',.1);text(clip,'far',250)
    plugin=ET.SubElement(sensor,'plugin',name='scene_camera_ros',filename='libgazebo_ros_camera.so')
    text(plugin,'robotNamespace','/msim');text(plugin,'cameraName','scene_overview')
    text(plugin,'imageTopicName','image_raw');text(plugin,'cameraInfoTopicName','camera_info')
    text(plugin,'frameName','scene_camera');text(plugin,'updateRate',.2)


def base_world():
    tree=ET.parse(ROOT.parent/'FAST_GLIO/simulation/uav_simulator/worlds/urban_canyon.world')
    world=tree.getroot().find('world')
    for model in list(world.findall('model')):world.remove(model)
    return tree,world


def simple_reference():
    tree=ET.parse(ROOT.parent/'FAST_GLIO/simulation/uav_simulator/worlds/urban_canyon.world')
    world=tree.getroot().find('world');objects=[]
    for model in world.findall('model'):
        size=model.findtext('link/collision/geometry/box/size')
        if size:
            objects.append(dict(name=model.attrib['name'],pose=list(map(float,model.findtext('pose').split())),
                                kind='box',size=list(map(float,size.split()))))
    return tree,world,objects,[]


def urban():
    tree,world=base_world();world.set('name','complex_urban')
    objects=[]
    # Clear strip: E in [-10,4], N in [-23,16], up to 9.5 m.
    buildings=[(-18,-27,8,10,12,0),(-17,-11,8,11,7.5,0),(-21,6,12,10,19,.12),
               (-18,23,9,11,10,-.18),(12,-27,10,9,18,0),(11,-12,9,10,6.5,0),
               (16,4,12,9,14,-.2),(12,22,10,10,8,.15),(-32,-7,10,13,24,.3),
               (30,9,13,12,22,-.3),(-3,35,14,10,17,.15)]
    for i,(x,y,sx,sy,h,yaw) in enumerate(buildings):
        objects.append(solid(world,f'urban_building_{i}',[x,y,h/2,0,0,yaw],'box',[sx,sy,h],(.45+.03*(i%4),.55,.62)))
        # Roof equipment is part of the ray/collision geometry.
        objects.append(solid(world,f'roof_unit_{i}',[x+1,y,h+.6,0,0,yaw],'box',[2,2,1.2],(.35,.4,.45)))
    for i,y in enumerate((-25,-16,-5,6,17)):
        for side,x in enumerate((-12,6)):
            h=6.5+(i%3)*2
            objects.append(solid(world,f'urban_column_{i}_{side}',[x,y,h/2,0,0,0],'cylinder',[.45,h],(.58,.58,.56)))
    # Elevated bridge: underside remains above the entire flight envelope.
    objects.append(solid(world,'urban_overpass',[-3,4,11.2,0,0,0],'box',[30,3,.5],(.55,.5,.43)))
    for x in (-17,11):
        objects.append(solid(world,f'bridge_support_{x}',[x,4,5.5,0,0,0],'box',[1.2,1.5,11],(.5,.5,.5)))
    for i,(x,y) in enumerate(((9,-2),(-15,13),(19,-20),(-26,-20))):
        objects.append(solid(world,f'angled_facade_{i}',[x,y,4,0,0,.45*(-1)**i],'box',[2.5,4,8],(.66,.49,.37)))
    return tree,world,objects,[]


def model_dependencies(model_path,models):
    for node in ET.parse(model_path).iter('uri'):
        value=node.text or ''
        if value.startswith('model://'):
            relative=value[len('model://'):]
            if '/' in relative and not(models/relative).exists():
                raise FileNotFoundError(f'model resource missing: {relative}')


def industrial(px4):
    tree,world=base_world();world.set('name','industrial_yard')
    models=px4/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'
    original=ET.parse(px4/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/warehouse.world')
    imports=[];seen={}
    for include in original.getroot().find('world').findall('include'):
        uri=include.findtext('uri','')
        name=uri.removeprefix('model://')
        if name.endswith('_no_collision'):name=name.removesuffix('_no_collision')
        if name not in ('shelves_high2','europallet','big_box3','big_box4','pallet_full'):continue
        seen[name]=seen.get(name,0)+1
        selected={'shelves_high2':(1,2,5,9),'europallet':(1,5,9,13),
                  'big_box3':(1,2,3,4,5,6),'big_box4':(1,2,3),'pallet_full':(1,2)}
        if seen[name] not in selected[name]:continue
        path=models/name/'model.sdf'
        if not path.is_file():continue
        model_dependencies(path,models)
        for side,shift in [('west',-18),('east',20)]:
            clone=copy.deepcopy(include)
            clone.find('uri').text='model://'+name
            old_name=clone.find('name')
            clone_name=f"imported_{side}_{len(imports)}_{name}"
            if old_name is None:text(clone,'name',clone_name)
            else:old_name.text=clone_name
            pose=[float(x) for x in clone.findtext('pose','0 0 0 0 0 0').split()]
            pose[0]+=shift;pose[1]-=6
            pose_node=clone.find('pose')
            if pose_node is None:text(clone,'pose',' '.join(map(str,pose)))
            else:pose_node.text=' '.join(map(str,pose))
            # Import as static fixture: no rigid body settling can alter the test map.
            model=ET.parse(path).getroot().find('model')
            embedded=copy.deepcopy(model);embedded.set('name',clone_name)
            old_pose=embedded.find('pose')
            if old_pose is not None:embedded.remove(old_pose)
            text(embedded,'pose',' '.join(map(str,pose)))
            static=embedded.find('static')
            if static is None:text(embedded,'static','true')
            else:static.text='true'
            if name=='europallet':
                # Fine wooden slats are below this flight's useful angular resolution.
                # Preserve the mesh visual but avoid dozens of tiny physics contacts per pallet.
                for link in embedded.findall('link'):
                    for collision in list(link.findall('collision')):link.remove(collision)
                link=embedded.find('link')
                collision=ET.SubElement(link,'collision',name='pallet_envelope')
                text(collision,'pose','0 0 0.075 0 0 0')
                box=ET.SubElement(ET.SubElement(collision,'geometry'),'box')
                text(box,'size','1.2 0.8 0.15')
            world.append(embedded)
            imports.append(dict(name=clone_name,source=str(path),pose=pose,
                collision_geometry='single bounding box' if name=='europallet' else 'source model collisions'))
    objects=[]
    for i,(x,y,h) in enumerate([(-16,-26,9),(-17,-10,12),(-16,9,7),(13,-24,8),(13,-5,10),(14,14,6.5)]):
        objects.append(solid(world,f'warehouse_block_{i}',[x,y,h/2,0,0,.12*(-1)**i],'box',[6,7,h],(.53,.59,.65)))
    for side,x in enumerate((-12,6.5)):
        for i,y in enumerate((-24,-12,0,12,23)):
            objects.append(solid(world,f'yard_silo_{side}_{i}',[x,y,5+i%2,0,0,0],'cylinder',[.65,10+2*(i%2)],(.62,.57,.45)))
    for i,y in enumerate((-8,12)):
        objects.append(solid(world,f'overhead_beam_{i}',[-3,y,12,0,0,0],'box',[30,1.0,.5],(.65,.36,.2)))
    for i,(x,y,yaw) in enumerate(((10,5,.3),(-15,-19,-.3),(12,-17,.1))):
        for step in range(5):
            h=1+step*.9
            objects.append(solid(world,f'yard_step_{i}_{step}',[x+step*.4,y+step*.8,h/2,0,0,yaw],
                                 'box',[2,1.0,h],(.63,.48,.3)))
    return tree,world,objects,imports


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--px4-root',type=Path,default=ROOT/'build/deps/PX4-Autopilot')
    parser.add_argument('--output-dir',type=Path,default=ROOT/'scenes')
    args=parser.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    manifest={}
    for label,builder in [('simple',simple_reference),('complex_urban',urban),('industrial_yard',lambda:industrial(args.px4_root))]:
        tree,world,objects,imports=builder();camera(world)
        ET.indent(tree)
        path=args.output_dir/(label+'.world');tree.write(path,encoding='unicode',xml_declaration=True)
        manifest[label]=dict(world=str(path.resolve()),primitive_collision_objects=objects,
            imported_static_models=imports,route_spawn_enu=[0,-20,0],flight_height_m=8,
            intended_clear_strip_enu=dict(east=[-8,4] if label=='simple' else [-10,4],north=[-23,16],up=[0,9.5]),
            physics_step_s=.002,notes='static geometry fixtures; ray LiDAR uses collision geometry; no GNSS multipath/rain/dynamic actors')
    (args.output_dir/'scene-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({k:dict(primitives=len(v['primitive_collision_objects']),imported=len(v['imported_static_models'])) for k,v in manifest.items()},indent=2))


if __name__=='__main__':main()
