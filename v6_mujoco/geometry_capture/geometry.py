"""Independent triangle/box and convex-polytope diagnostics, in SI units."""
import numpy as np
from scipy.spatial import ConvexHull, cKDTree


def load_stl(path):
    raw = path.read_bytes()
    count = int.from_bytes(raw[80:84], 'little')
    dtype = np.dtype([('normal','<f4',(3,)), ('triangle','<f4',(3,3)), ('attr','<u2')])
    assert len(raw) == 84 + count * 50
    triangles = np.frombuffer(raw, dtype=dtype, offset=84)['triangle'].astype(float)
    vertices, inverse = np.unique(triangles.reshape(-1,3), axis=0, return_inverse=True)
    return vertices, inverse.reshape(-1,3)


def topology(v, f):
    edges = np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]])
    _, counts = np.unique(np.sort(edges,axis=1),axis=0,return_counts=True)
    directed = set(map(tuple,edges.tolist()))
    return {'vertices':len(v),'triangles':len(f),'closed_two_manifold_edges':bool(np.all(counts==2)),
            'consistently_oriented_edges':all((b,a) in directed for a,b in directed),
            'signed_volume_m3':float(np.sum(np.einsum('ij,ij->i',v[f[:,0]],np.cross(v[f[:,1]],v[f[:,2]])))/6),
            'convex_hull_volume_m3':float(ConvexHull(v).volume)}


def clip_triangle_box(triangle, half):
    """Sutherland-Hodgman clipping against all six closed box halfspaces.

    Covers triangle faces and edges crossing a box even with no inside vertex.
    A positive-area clipped polygon witnesses actual represented surface entry.
    """
    p = np.asarray(triangle).copy()
    for axis in range(3):
        for sign in [-1,1]:
            out=[]
            for a,b in zip(p,np.roll(p,-1,axis=0)):
                da=sign*a[axis]-half[axis]; db=sign*b[axis]-half[axis]
                if da<=0:out.append(a)
                if (da<0 and db>0) or (da>0 and db<0):out.append(a+(b-a)*da/(da-db))
            p=np.asarray(out).reshape(-1,3)
            if not len(p):return p
    return p


def winding_inside(points, v, f):
    result=[]
    for p in points:
        a,b,c=(v[f[:,i]]-p for i in range(3))
        lengths=[np.linalg.norm(x,axis=1) for x in [a,b,c]]
        den=lengths[0]*lengths[1]*lengths[2]+np.einsum('ij,ij->i',a,b)*lengths[2]+np.einsum('ij,ij->i',b,c)*lengths[0]+np.einsum('ij,ij->i',c,a)*lengths[1]
        solid=2*np.arctan2(np.einsum('ij,ij->i',a,np.cross(b,c)),den)
        result.append(abs(float(solid.sum()))>2*np.pi)
    return result


def mesh_box(v, f, half):
    topology_result=topology(v,f)
    polygons=[(i,clip_triangle_box(t,half)) for i,t in enumerate(v[f])]
    polygons=[(i,p) for i,p in polygons if len(p)>=3]
    inside=np.all(np.abs(v)<half-1e-12,axis=1)
    corners=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]])*half
    reliable=topology_result['closed_two_manifold_edges'] and topology_result['consistently_oriented_edges']
    containment=winding_inside(np.vstack([corners,np.zeros(3)]),v,f) if reliable else None
    return {'mesh_topology':topology_result,'strict_inside_vertex_count':int(inside.sum()),
            'triangle_box_crossing_count':len(polygons),'clipped_triangle_witnesses_box_m':[{'triangle':i,'polygon':p.tolist()} for i,p in polygons],
            'box_corners_and_center_inside_mesh':containment,
            'solid_test_reliable_for_this_closed_mesh':reliable,
            'represented_surface_intersects_box':bool(polygons or inside.any()),
            'limitation':'Only the supplied collision STL is represented; no manufacturing CAD or hardware certification.'}


def convex_box_sat(v, half):
    """Complete separating axes for convex mesh vs box; independent of MuJoCo.

    Reports a signed separating-axis gap, NOT Euclidean distance when separated.
    All face normals and edge cross edge axes are considered.
    """
    hull=ConvexHull(v); axes=[*np.eye(3),*hull.equations[:,:3]]
    edges=np.concatenate([hull.simplices[:,[0,1]],hull.simplices[:,[1,2]],hull.simplices[:,[2,0]]])
    edges=np.unique(np.sort(edges,axis=1),axis=0)
    for e in v[edges[:,1]]-v[edges[:,0]]:
        axes.extend(np.cross(e,np.eye(3)))
    axes=np.asarray(axes); axes=axes[np.linalg.norm(axes,axis=1)>1e-12]
    axes/=np.linalg.norm(axes,axis=1)[:,None]
    q=v@axes.T; b=np.abs(axes)@half
    gaps=np.maximum(q.min(0)-b,-b-q.max(0)); i=int(np.argmax(gaps))
    return {'intersect':bool(gaps[i]<-1e-10),'maximum_separating_axis_gap_m':float(gaps[i]),'axis_box':axes[i].tolist(),'axes_tested':len(axes)}
