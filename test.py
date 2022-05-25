import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import tflib
import yaml
import cicp_solver as solver

Param={
  'icp_threshold':1.5,
  'icp_precision':0.1,
  'normal_radius': 3
}

def main():
  with open('data/camera_master0.yaml') as file:
    obj = yaml.safe_load(file)
    bTc = tflib.toRTfromVec(tflib.dict2vec(obj))
    Param["transform"]=bTc

  pcd_target = o3d.io.read_point_cloud('data/test_RZ3_1.ply')
  pcd_source = o3d.io.read_point_cloud('data/surface_1.ply')
  o3d.visualization.draw_geometries([pcd_target,pcd_source])

  source_pnts=np.array(pcd_source.points)
  solver.learn([source_pnts],Param)
  target_pnts=np.array(pcd_target.points)
  result=solver.solve([target_pnts],Param)
  RT=result["transform"][0]
  print('test result',RT.T[3,:3],Rot.from_matrix(RT[:3,:3]).as_euler('xyz',degrees=True))
  pcd_source.transform(RT)
  o3d.visualization.draw_geometries([pcd_target,pcd_source])
  print("RMSE",result["rmse"][0])
  print("Fitness",result["fitness"][0])

if __name__ == '__main__':
    main()
