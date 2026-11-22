"""Read the two pinned KSQL COLLADA meshes for independent scene checks."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def load_mesh(path):
    root = ET.parse(path).getroot()
    ns = {'c': root.tag.split('}')[0][1:]}
    if root.findtext('c:asset/c:up_axis', namespaces=ns) != 'Z_UP':
        raise ValueError('Expected the pinned Z-up KSQL mesh')
    unit = float(root.find('c:asset/c:unit', ns).get('meter'))
    geometries = {}
    for geo in root.findall('.//c:library_geometries/c:geometry', ns):
        mesh = geo.find('c:mesh', ns)
        inputs = mesh.findall('c:vertices/c:input', ns)
        source = next(x.get('source')[1:] for x in inputs if x.get('semantic') == 'POSITION')
        array = mesh.find(f"c:source[@id='{source}']/c:float_array", ns)
        vertices = np.fromstring(array.text, sep=' ').reshape(-1, 3)
        faces = []
        for tri in mesh.findall('c:triangles', ns):
            ins = tri.findall('c:input', ns)
            stride = max(int(x.get('offset', '0')) for x in ins)+1
            offset = next(int(x.get('offset', '0')) for x in ins if x.get('semantic') == 'VERTEX')
            index = np.fromstring(tri.findtext('c:p', namespaces=ns), sep=' ', dtype=np.int64)
            faces.append(index.reshape(-1, stride)[:, offset].reshape(-1, 3))
        if not faces:
            raise ValueError('KSQL mesh has no supported triangle geometry')
        geometries[geo.get('id')] = (vertices, np.concatenate(faces))
    output = []

    def visit(node, parent):
        matrix = parent.copy()
        for child in node:
            tag = child.tag.split('}')[-1]
            if tag == 'matrix':
                matrix = matrix @ np.fromstring(child.text, sep=' ').reshape(4, 4)
            elif tag in ('translate', 'rotate', 'scale'):
                raise ValueError('Unexpected transform in pinned KSQL asset')
        for instance in node.findall('c:instance_geometry', ns):
            vertices, faces = geometries[instance.get('url')[1:]]
            vertices = unit*(vertices @ matrix[:3, :3].T + matrix[:3, 3])
            output.append(vertices[faces])
        for child in node.findall('c:node', ns):
            visit(child, matrix)

    for node in root.findall('.//c:visual_scene/c:node', ns):
        visit(node, np.eye(4))
    return np.concatenate(output)


def vertical_hits(triangles, east, north):
    """All vertical intersections with non-vertical mesh triangles, in metres."""
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    ab, ac = b-a, c-a
    det = ab[:, 0]*ac[:, 1]-ab[:, 1]*ac[:, 0]
    valid = np.abs(det) > 1e-9
    denominator = np.where(valid, det, 1.)
    de, dn = east-a[:, 0], north-a[:, 1]
    u = (de*ac[:, 1]-dn*ac[:, 0])/denominator
    v = (ab[:, 0]*dn-ab[:, 1]*de)/denominator
    mask = valid & (u >= -1e-8) & (v >= -1e-8) & (u+v <= 1+1e-8)
    return (a[:, 2]+u*ab[:, 2]+v*ac[:, 2])[mask]


def load_ksql(directory):
    directory = Path(directory)/'upstream/ksql_airport/media'
    meshes = {k: load_mesh(directory/f'SanCarlosAirport_{k}.DAE')
              for k in ('Terrain', 'Buildings')}
    # The upstream world places both meshes 3.5 m below their local coordinates.
    for triangles in meshes.values():
        triangles[:, :, 2] -= 3.5
    return meshes
