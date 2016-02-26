#!/usr/bin/python
"""
Zurich Eye
"""

import os
import logging
import numpy as np
import ze_trajectory_analysis.utils as utils

# -----------------------------------------------------------------------------
# Dataset loading
  
def load_dataset_csv(data_dir, filename_gt = 'traj_gt.csv', filename_es = 'traj_es.csv',
                     filename_matches = 'traj_es_gt_matches.csv', rematch_timestamps = False,
                     match_timestamps_offset = 0.0, match_timestamps_max_difference_sec = 0.02):
    logger = logging.getLogger(__name__)
    
    # generate or load matches:
    if not os.path.exists(os.path.join(data_dir, filename_matches)) or rematch_timestamps:
        logger.info("Find timestamp matches.")
        match_command = "rosrun ze_trajectory_analysis match_stamps" \
                        + " --data_dir=" + data_dir \
                        + " --filename_es=" + filename_es \
                        + " --filename_gt=" + filename_gt \
                        + " --filename_matches=" + filename_matches \
                        + " --offset_sec=" + str(match_timestamps_offset) \
                        + " --max_difference_sec=" + str(match_timestamps_max_difference_sec)
        logger.info("Executing command: "+match_command)
        os.system(match_command)
                            
    # load data:
    filename_gt = utils.check_file_exists(os.path.join(data_dir, filename_gt))
    filename_es = utils.check_file_exists(os.path.join(data_dir, filename_es))
    filename_matches = os.path.join(data_dir, filename_matches)
    keys_gt = utils.read_nanosecond_timestamps_from_csv_file(filename_gt, 0, ',')        
    keys_es = utils.read_nanosecond_timestamps_from_csv_file(filename_es, 0, ',')
    data_es = np.genfromtxt(filename_es, delimiter=',', dtype=np.float64, skip_header=1)[:,1:]
    data_gt = np.genfromtxt(filename_gt, delimiter=',', dtype=np.float64, skip_header=1)[:,1:]
    matches = np.genfromtxt(filename_matches, dtype=np.int64, delimiter=',', skip_header=1)
    
    # Create look-up table { es -> gt }
    matches_lut = dict([(row[0], row[1]) for row in matches])
    
    groundtruth = dict([ (keys_gt[i], data_gt[i,:]) for i in range(len(keys_gt))])
    
    p_es = []
    p_gt = []
    q_es = []
    q_gt = []
    t_gt = []
    t_es = []
    for i in range(len(keys_es)):
        if keys_es[i] in matches_lut:
            t_es_val = keys_es[i]
            t_gt_val = matches_lut[t_es_val]
            gt_data = groundtruth[t_gt_val]
            p_gt.append(gt_data[:3])
            tmp = gt_data[3:7] # quaternion order x y z w
            q_gt.append([tmp[1], tmp[2], tmp[3], tmp[0]])            
            p_es.append(data_es[i,0:3])
            q_es.append(data_es[i,3:7])
            t_es.append(t_es_val)
            t_gt.append(t_gt_val)
    p_es = np.array(p_es)
    p_gt = np.array(p_gt)
    q_es = np.array(q_es)
    q_gt = np.array(q_gt)
    t_gt = np.array(t_gt)
    t_es = np.array(t_es)

    return t_es, p_es, q_es, t_gt, p_gt, q_gt    
    

def load_estimator_results(filename):
    stamp = utils.read_nanosecond_timestamps_from_csv_file(filename, 0, ',')
    data = np.genfromtxt(filename, delimiter=',', dtype=np.float64, skip_header=1)[:,1:]
    velocity = data[:,7:10]    
    bias_gyr = data[:,10:13]
    bias_acc = data[:,13:17]
    return stamp, velocity, bias_gyr, bias_acc
        

def load_relative_errors_from_file(data_dir, segment_length,
                                   filename_result_prefix = 'traj_relative_errors'):
                                       
    data = np.genfromtxt(os.path.join(data_dir, filename_result_prefix+'_'+str(segment_length)+'.csv'),
                         delimiter=',', dtype=np.float64, skip_header=1)
    assert data[0, 7] == segment_length
    rel_pos_errors = np.abs(data[:,1:4]) # / segment_length
    rel_rot_errors = np.abs(data[:,4:7]) # / segment_length

    rel_pos_errors_norm = np.sqrt(np.sum(rel_pos_errors**2, 1))
    rel_roll_pitch_errors = np.sqrt(np.sum(rel_rot_errors[:,0:2]**2, 1))
    rel_yaw_errors = rel_rot_errors[:,2]
    
    start_indices = data[:,0].astype(int)
    scale_errors = data[:,9]
    
    return rel_pos_errors_norm, rel_roll_pitch_errors, rel_yaw_errors, scale_errors, start_indices


    
# -----------------------------------------------------------------------------
# DEPRECATED

def load_hand_eye_calib(params):
    print('loading hand-eye-calib')
    if 'T_sensor_trackable' in params:
        T_cm_quat = np.array([params['T_sensor_trackable']['qx'],
                              params['T_sensor_trackable']['qy'],
                              params['T_sensor_trackable']['qz'],
                              params['T_sensor_trackable']['qw']])
        T_cm_tran = np.array([params['T_sensor_trackable']['tx'],
                              params['T_sensor_trackable']['ty'],
                              params['T_sensor_trackable']['tz']])
        T_cm = utils.get_rigid_body_trafo(T_cm_quat, T_cm_tran)
    else:
        T_cm = np.eye(4)
    return T_cm
    
def load_dataset(results_dir, cam_delay):
    print('loading dataset in '+results_dir)   
    print('cam_delay = '+str(cam_delay))

    data_es = np.loadtxt(os.path.join(results_dir, 'traj_estimate.txt'))
    data_gt = np.loadtxt(os.path.join(results_dir, 'groundtruth.txt'))
    data_gt = dict([(int(row[0]), row[1:]) for row in data_gt])
    matches = np.loadtxt(os.path.join(results_dir, 'groundtruth_matches.txt'))
    matches = dict([(int(row[0]), int(row[1])) for row in matches])
    p_es = []
    p_gt = []
    q_es = []
    q_gt = []
    t_gt = []
    for row_es in data_es:
        image_id = int(row_es[0])
        if image_id in matches:
            groundtruth_id = matches[image_id]
            row_gt = data_gt[groundtruth_id]
            p_es.append(row_es[1:4])
            p_gt.append(row_gt[1:4])
            q_es.append(row_es[4:8])
            q_gt.append(row_gt[4:8])
            t_gt.append(row_gt[0])    
    p_es = np.array(p_es)
    p_gt = np.array(p_gt)
    q_es = np.array(q_es)
    q_gt = np.array(q_gt)
    t_gt = np.array(t_gt)

    return t_gt, p_es, q_es, t_gt, p_gt, q_gt
    

    
def load_synthetic_dataset(results_dir):
    gt_trajectory = np.loadtxt(os.path.join(results_dir, 'groundtruth.txt'))
    es_trajectory = np.loadtxt(os.path.join(results_dir, 'traj_estimate.txt'))
    N = es_trajectory.shape[0]
    p_es = es_trajectory[:, 1:4]
    q_es = es_trajectory[:, 4:8]

    map_index_slot = dict()
    for i, index in enumerate(gt_trajectory[:, 0]):
        map_index_slot[int(index)] = i
    gt_indices = np.zeros(N)
    for i in range(N):
        gt_indices[i] = map_index_slot[int(es_trajectory[i,0])]
        
    p_gt = gt_trajectory[np.int32(gt_indices), 1:4]
    q_gt = gt_trajectory[np.int32(gt_indices), 4:8]
    t_gt = gt_indices
    return t_gt, p_es, q_es, t_gt, p_gt, q_gt