#!/usr/bin/python
"""
@author: Christian Forster
"""

import numpy as np
import ze_py.transformations as tf

def get_rigid_body_trafo(quat,trans):
    T = tf.quaternion_matrix(quat)
    T[0:3,3] = trans
    return T
    
def ominus(a,b):
    """
    Compute the relative 3D transformation between a and b.
    
    Input:
    a -- first pose (homogeneous 4x4 matrix)
    b -- second pose (homogeneous 4x4 matrix)
    
    Output:
    Relative 3D transformation from a to b.
    """
    return np.dot(np.linalg.inv(a),b)
    
def distances_along_trajectory(traj):
    """
    Compute the translational distances along a trajectory. 
    """

    motion = [ np.linalg.norm(traj[i,:]-traj[i+1,:]) for i in range(len(traj)-1)]
    distances = [0]
    sum = 0
    for t in motion:
        sum += t
        distances.append(sum)
    return distances
    
def compute_comparison_indices_length(distances, dist, max_error):
    max_idx = len(distances)
    comparisons = []
    for idx, d in enumerate(distances):
        j = -1
        error = max_error
        for i in range(idx, max_idx):
            if np.abs(distances[i]-(d+dist)) < error: 
                j = i
                error = np.abs(distances[i]-(d+dist))
        comparisons.append(j)
    return comparisons
    
def compute_comparison_indices_time(t_gt, dist, max_error):
    max_idx = len(t_gt)
    comparisons = []
    for idx, d in enumerate(t_gt):
        j = -1
        error = max_error
        for i in range(idx, max_idx):
            if np.abs(t_gt[i]-(d+dist)) < error: 
                j = i
                error = np.abs(t_gt[i]-(d+dist))
        comparisons.append(j)
    return comparisons
    

def compute_angle(transform):
    """
    Compute the rotation angle from a 4x4 homogeneous matrix.
    """
    # an invitation to 3-d vision, p 27
    return np.arccos( min(1,max(-1, (np.trace(transform[0:3,0:3]) - 1)/2) ))*180.0/np.pi
    
def compute_relative_error(p_es, q_es, p_gt, q_gt, T_cm, dist, max_error, unit='m', t_gt=[], scale=1.0):
    
    # find for every frame another frame that is 'dist' away to compare
    if unit == 'm':
        distances = distances_along_trajectory(p_gt)
        comparisons = compute_comparison_indices_length(distances, dist, max_error)
    elif unit == 's':
        comparisons = compute_comparison_indices_time(t_gt, dist, max_error)
              
    print('#comparisons = '+str(len(comparisons)))
    # compute relative error
    T_mc = np.linalg.inv(T_cm)
    errors = []
    for idx, c in enumerate(comparisons):
        if not c==-1:
            T_c1 = get_rigid_body_trafo(q_es[idx,:], p_es[idx,:])
            T_c2 = get_rigid_body_trafo(q_es[c,:], p_es[c,:])
            T_c1_c2 = np.dot(np.linalg.inv(T_c1), T_c2)
            T_c1_c2[:3,3] *= scale
            # rotate error in world frame to get meaningful roll-pitch-yaw errors
            #w_T_c1_c2 = np.dot(T_c1, np.dot(c1_T_c1_c2, np.linalg.inv(T_c1))) 
            T_m1 = get_rigid_body_trafo(q_gt[idx,:], p_gt[idx,:])
            T_m2 = get_rigid_body_trafo(q_gt[c,:], p_gt[c,:])
            T_m1_m2 = np.dot(np.linalg.inv(T_m1), T_m2)
            T_m1_m2_in_c1 = np.dot(T_cm, np.dot(T_m1_m2, T_mc))
            T_error_in_c2 = np.dot(np.linalg.inv(T_c1_c2), T_m1_m2_in_c1)
            T_c2_rot = np.eye(4)
            T_c2_rot[0:3,0:3] = T_c2[0:3,0:3]
            T_error_in_w =  np.dot(T_c2_rot, np.dot(T_error_in_c2, np.linalg.inv(T_c2_rot))) 
            errors.append(T_error_in_w)
    
    error_trans_norm = []
    error_yaw = []
    error_gravity = []
    error_angle = []
    for e in errors:
        error_trans_norm.append(np.linalg.norm(e[0:3,3]))
        rpy_angles = tf.euler_from_matrix(e, 'rzyx')
        error_angle.append(compute_angle(e))
        error_yaw.append(abs(rpy_angles[0])*180.0/np.pi)
        error_gravity.append(np.sqrt(rpy_angles[1]**2+rpy_angles[2]**2)*180.0/np.pi)
        #error_gravity.append( 0.5*(rpy_angles[1]+rpy_angles[2]) )
    return errors, error_trans_norm, np.array(error_yaw), np.array(error_gravity), error_angle
    
def get_distance_from_start(gt_translation):
    distances = np.diff(gt_translation[:,0:3],axis=0)
    distances = np.sqrt(np.sum(np.multiply(distances,distances),1))
    distances = np.cumsum(distances)
    distances = np.concatenate(([0], distances))
    return distances
    