from image_proc.pcd_io import load_xyzrgb
import bulletsim.rope_initialization as ri
import numpy as np

xyz, rgb = load_xyzrgb("data000000000004.pcd")
xyz = xyz.reshape(-1,3)
S = ri.skeletonize_point_cloud(xyz)
print "done!"


from mayavi import mlab
xs, ys, zs, us, vs, ws = [],[],[],[],[],[]
for (node0, node1) in S.edges():
    (x0,y0,z0) = S.node[node0]["xyz"]
    (x1,y1,z1) = S.node[node1]["xyz"]
    xs.append(x0)
    ys.append(y0)
    zs.append(z0)
    us.append(x1-x0)
    vs.append(y1-y0)
    ws.append(z1-z0)
    mlab.plot3d([x0,x1],[y0,y1],[z0,z1], color=(1,0,0),tube_radius=.0025)
    
#mlab.quiver3d(xs,ys,zs,us,vs,ws,mode= '2ddash',line_width=.001)
#mlab.points3d(xs,ys,zs)
mlab.points3d(*xyz.T,**dict(color=(0,0,1),scale_factor=.01,opacity=.25))