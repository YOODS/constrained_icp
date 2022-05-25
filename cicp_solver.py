import open3d as o3d
import numpy as np
from scipy.optimize import least_squares as solver
from scipy.spatial.transform import Rotation as Rot
import tflib
import yaml
import copy

def Tmat(x):
    M = np.eye(4)
    M[:3, :3] = Rot.from_euler('xyz',[0,0,x[1]],degrees=True).as_matrix()
    M[2,3]=x[0]
    return M

def calc_residual(x, source, threshold):
#    print("x0",x)
    transform = Tmat(x)
    src=copy.deepcopy(source)
    src.transform(transform)
    points=np.array(src.points)

    residue=np.zeros(len(points))
#    for n,sp in enumerate(points):
#        k, idx, _ = target_kdtree.search_hybrid_vector_3d(sp, max_correspond_dist, 1)
#        if k == 1:
#            i=idx[0]
#            residue[n]=target_normals[i].dot(sp-target_points[i])
    res=o3d.pipelines.registration.evaluate_registration(src, target_pcd, threshold)
    kidx=np.asarray(res.correspondence_set)
    sidx=kidx.T[0]  #source points which have neighbor target points
    tidx=kidx.T[1]  #corespondance neighbor target points
    spoints=points[sidx]
    tpoints=target_points[tidx]
    tnormals=target_normals[tidx]
    dists=tnormals*(spoints-tpoints)
    dists=np.linalg.norm(dists,axis=1)
    residue[sidx]=dists
    return residue

def learn(datArray,param):
  global pcd_source,kd_param
  print("CGA solver learn")
  kd_param=o3d.geometry.KDTreeSearchParamHybrid(radius=param["normal_radius"], max_nn=30)
  pcd_source=o3d.geometry.PointCloud()
  pcd_source.points=o3d.utility.Vector3dVector(datArray[0])
  pcd_source.estimate_normals(kd_param)
  pcd_source.orient_normals_towards_camera_location()
  print("c-cip learn",pcd_source)
  return [pcd_source]

def solve(datArray,param):
  global target_pcd,target_kdtree,target_points,target_normals
  pcd_target=o3d.geometry.PointCloud()
  pcd_target.points=o3d.utility.Vector3dVector(datArray[0])
  pcd_target.estimate_normals(kd_param)
  pcd_target.orient_normals_towards_camera_location()
  if 'transform' in param:
    bTc=param['transform']
  else:
    bTc=np.eye(4)
  source_pcd=copy.deepcopy(pcd_source)
  source_pcd.transform(bTc)
  target_pcd=copy.deepcopy(pcd_target)
  target_pcd.transform(bTc)
  target_points=np.array(target_pcd.points)
  target_normals=np.array(target_pcd.normals)
  target_kdtree = o3d.geometry.KDTreeFlann(pcd_target)    #No need if 'evaluate_registration' is used
  x0 = np.array([0, 0], dtype=float)
  precision=param["icp_precision"]
  threshold=param["icp_threshold"]
  print("Coarse ICP")
  res1 = solver(calc_residual,x0,jac='2-point',method='trf',ftol=precision,loss='soft_l1',args=(source_pcd,threshold*10))
  x0 = res1.x
  print("Fine ICP")
  res2 = solver(calc_residual,x0,jac='2-point',method='trf',ftol=precision,loss='soft_l1',args=(source_pcd,threshold))
  RT=Tmat(res2.x)
  print('cicp result',RT.T[3,:3],Rot.from_matrix(RT[:3,:3]).as_euler('xyz',degrees=True))
  RT=np.linalg.inv(bTc).dot(RT).dot(bTc)
  result=o3d.pipelines.registration.evaluate_registration(pcd_source,pcd_target,threshold,RT)
  score={"transform":[RT],"fitness":[result.fitness],"rmse":[result.inlier_rmse]}
  return score
