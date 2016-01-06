#!/usr/bin/python
"""
@author: Christian Forster
"""

import os
import yaml
import argparse
import numpy as np
import matplotlib.pyplot as plt
import ze_trajectory_analysis.align as align_trajectory
import ze_trajectory_analysis.utils as traj_utils
import ze_trajectory_analysis.load as traj_loading
import ze_trajectory_analysis.plot as traj_plot
import ze_py.transformations as tf
from matplotlib import rc
rc('font',**{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)

FORMAT = '.pdf'

class TrajectoryAnalysis:
    def __init__(self, results_dir, data_format='txt', align_type='sim3',
                 align_num_frames=-1, use_hand_eye_calib=False, 
                 evaluation_type='rms', boxplot_distances=''):
        assert os.path.exists(results_dir)
        assert data_format in ['txt', 'csv', 'blender']
        assert align_type in ['first_frame', 'sim3', 'se3']
        assert evaluation_type in ['rms', 'relative_errors', 'synthetic']
        self.data_dir = results_dir
        self.statistics_filename = os.path.join(self.data_dir, 'analysis_statistics.yaml')
        self.data_loaded = False
        self.data_aligned = False
        self.data_format = data_format
        self.align_type = align_type
        self.align_num_frames = int(align_num_frames)
        self.use_hand_eye_calib = use_hand_eye_calib
        self.evaluation_type = evaluation_type
        self.boxplot_distances = boxplot_distances
        
    def load_data(self):
        """
        Loads the trajectory data. The resuls {p_es, q_es, p_gt, q_gt} is
        synchronized and has the same length.
        """
        print('Loading trajectory data..')
        
        
        cam_delay = 0.0
        self.T_cm = np.eye(4)
        
        # Load dataset parameters
        dataset_params_file = os.path.join(self.data_dir, 'dataset.yaml')
        if os.path.exists(dataset_params_file):
            dataset_params = yaml.load(open(dataset_params_file,'r'))
            self.T_cm = traj_loading.load_hand_eye_calib(dataset_params) # (marker) in (camera) frame
            if cam_delay in dataset_params:
                cam_delay = dataset_params['cam_delay']
                
        self.T_mc = tf.inverse_matrix(self.T_cm)
        
        # Load trajectory groundtruth and estimate
        if self.data_format == 'txt':
            self.t_es, self.p_es, self.q_es, self.t_gt, self.p_gt, self.q_gt =\
                traj_loading.load_dataset(self.data_dir, cam_delay)
            
        elif self.data_format == 'csv':
            self.t_es, self.p_es, self.q_es, self.t_gt, self.p_gt, self.q_gt =\
                traj_loading.load_dataset_csv(self.data_dir)
        elif self.data_format == 'blender':
            self.t_es, self.p_es, self.q_es, self.t_gt, self.p_gt, self.q_gt =\
                traj_loading.load_synthetic_dataset(self.data_dir)
        else:
            raise ValueError('data_format' + self.data_format + ' not known.')
        self.distances = traj_utils.get_distance_from_start(self.p_gt)
               
        self.data_loaded = True
        print('...done.')
        
    def analyse_trajectory(self):
        if not self.data_loaded:
            self.load_data()
            
        if self.evaluation_type == 'synthetic':
            self.analyse_synthetic_trajectory(self.data_dir, self.boxplot_distances)
        elif self.evaluation_type == 'rms':
            self.compute_rms_errors()
        elif self.evaluation_type == 'relative_errors':
            self.compute_relative_errors()
        
    def align_trajectory(self):
        n = int(self.align_num_frames)
        if n < 0.0:
            print('Align all frames')
            n = len(self.p_es)
        else:
            print('Align trajectory using ' + str(n) + ' frames.')
         
        #
        # TODO: Apply hand-eye calibration to estimted trajectory
        #             
        
        if self.align_type == 'sim3':
            self.scale, self.rot, self.trans = align_trajectory.align_sim3(self.p_gt[:n,:], self.p_es[:n,:])
        elif self.align_type == 'se3':
            self.rot, self.trans = align_trajectory.align_se3(self.p_gt[:n,:], self.p_es[:n,:])
            self.scale = 1.0 
        elif self.align_type == 'first_frame':
            self.trans = np.zeros((3,))
            self.rot = np.eye(3)            
            self.scale = 1.0
            
        self.p_es_aligned = np.zeros(np.shape(self.p_es))
        self.q_es_aligned = np.zeros(np.shape(self.q_es))
        for i in range(np.shape(self.p_es)[0]):
            self.p_es_aligned[i,:] = self.scale*self.rot.dot(self.p_es[i,:]) + self.trans
            self.q_es_aligned[i,:] = ru.dcm2quat(self.rot.dot(ru.quat2dcm(self.q_es[i,:]))) # TODO: use tf
        self.data_aligned
            
    def compute_rms_errors(self):
       
        # align trajectory
        self.align_trajectory()
        
        # position error 
        e_trans = (self.p_gt-self.p_es_aligned)  
        e_trans_euclidean = np.sqrt(np.sum(e_trans**2, 1))
        
        # orientation error 
        e_rot = np.zeros((len(e_trans_euclidean,)))
        e_rpy = np.zeros(np.shape(self.p_es_aligned))
        for i in range(np.shape(self.p_es_aligned)[0]):
            R_we = tf.matrix_from_quaternion(self.q_es_aligned[i,:])
            R_wg = tf.matrix_from_quaternion(self.q_gt[i,:])
            R_ge = np.dot(np.linalg.inv(R_wg), R_we)
            e_rpy[i,:] = tf.euler_from_matrix(R_ge, 'rzyx')
            e_rot[i] = np.linalg.norm(tf.logmap_so3(R_ge[:3,:3]))
        
        # scale drift
        motion_gt = np.diff(self.p_gt, 0)
        motion_es = np.diff(self.p_es_aligned, 0)
        dist_gt = np.sqrt(np.sum(np.multiply(motion_gt,motion_gt),1))
        dist_es = np.sqrt(np.sum(np.multiply(motion_es,motion_es),1))
        e_scale_rel = np.divide(dist_es,dist_gt)-1.0

        # position drift
        
        # plot
        traj_plot.plot_translation_error(self.distances, e_trans, self.data_dir)
        traj_plot.plot_rotation_error(self.distances, e_rpy, self.data_dir)
        traj_plot.plot_scale_error(self.distances, e_scale_rel*100.0, self.data_dir)
        
        # compute error statistics:
        stats_logger.compute_and_save_statistics(e_trans_euclidean, 'trans', self.statistics_filename)
        stats_logger.compute_and_save_statistics(e_rot, 'rot', self.statistics_filename)
        stats_logger.compute_and_save_statistics(e_scale_rel, 'scale', self.statistics_filename)
        
        # plot trajectory
        traj_plot.plot_trajectory(self.data_dir, self.p_gt, self.p_es_aligned,
                                  self.align_num_frames)
                            
        if os.path.exists(os.path.join(self.data_dir, 'pointcloud.txt')):
            traj_plot.plot_pointcloud_3d(self.data_dir, 
                                         self.p_gt, self.p_es_aligned,
                                         self.scale, self.rot, self.trans)
    
    def get_trajectory_length(self):
        assert(self.data_loaded)
        return self.distances[-1]

def analyse_synthetic_trajectory(results_dir):
    
    data = np.loadtxt(os.path.join(results_dir, 'translation_error.txt'))
    t_gt, p_es, q_es, t_gt, p_gt, q_gt = load_synthetic_dataset(results_dir)

    distances = traj_utils.get_distance_from_start(p_gt)
    translation_error = data[:,1:4]
    plot_translation_error(distances, translation_error, results_dir)
      
    # plot orientation error
    data = np.loadtxt(os.path.join(results_dir, 'orientation_error.txt'))
    orientation_error = data[:,1:4]
    plot_rotation_error(distances, orientation_error, results_dir)

    # plot trajectory
    plot_trajectory(results_dir, p_gt, p_es, -1, q_gt, q_es)
    
    # plot mean/variance boxplots for a subset of travelled distances
    #if boxplot_distances != '':
    #  plot_boxplots_distances(results_dir, p_es, q_es, p_gt, q_gt, boxplot_distances)
    
def compute_relative_errors(results_dir, t_es, p_es, q_es, t_gt, p_gt, q_gt, n_align_frames, estimate_scale, T_cm): 

    

    # align Sim3/SE3
    if estimate_scale:
        print('align Sim3 using '+str(n_align_frames)+' first frames.')
        scale,rot,trans = align_trajectory.align_sim3(p_gt[0:n_align_frames,:], p_es[0:n_align_frames,:])
    else:
        scale = 1.0 
    print('scale = '+str(scale))
  
    # get trafo between (v)ision and (o)ptitrack frame
    print(q_gt[0,:])
    print(p_gt[0,:])
    T_om = get_rigid_body_trafo(q_gt[0,:], p_gt[0,:])
    T_vc = get_rigid_body_trafo(q_es[0,:], scale*p_es[0,:])
    T_cv = tf.inverse_matrix(T_vc)
    T_ov = np.dot(T_om, np.dot(T_mc, T_cv))
    print('T_ov = ' + str(T_ov))
  
    # apply transformation to estimated trajectory
    q_es_aligned = np.zeros(np.shape(q_es))
    p_es_aligned = np.zeros(np.shape(p_es))
    for i in range(np.shape(p_es)[0]):
        T_vc = get_rigid_body_trafo(q_es[i,:],p_es[i,:])
        T_vc[0:3,3] *= scale
        T_om = np.dot(T_ov, np.dot(T_vc, T_cm))
        p_es_aligned[i,:] = T_om[0:3,3]
        q_es_aligned[i,:] = tf.quaternion_from_matrix(T_om) 
      
    traj_plot.plot_trajectory(results_dir, p_gt, p_es_aligned, n_align_frames, q_gt, q_es_aligned)
    
    # compute relative rotation errors    (TODO: activate again for TUM datasets)
    dist = 1.0
    unit='s'
    errors, e_trans, e_yaw, e_gravity, e_angle = traj_utils.compute_relative_error(p_es, q_es, p_gt, q_gt, T_cm, dist, 0.2*dist, unit, t_gt, scale)
    e_trans_median = np.median(e_trans)
    e_angle_median = np.median(e_angle)
    print('Relative Error ' + str(dist)+'/'+unit+' over ' + str(len(e_trans)) + ' measurements:')
    print('trans median error = ' + str(e_trans_median) + 'm/s,   mean error = ' + str(np.mean(e_trans)))
    print('rot   median error = ' + str(e_angle_median) + 'deg/s, mean error = ' + str(np.mean(e_angle)))
    fig = plt.figure(figsize=(8, 4))
    ax = fig.add_subplot(211, xlabel='Measurement', ylabel='Relative Error [m/sec]', title='Median error = '+str(e_trans_median*100)+'[cm/s]')
    ax.plot(e_trans)
    ax = fig.add_subplot(212, xlabel='Measurement', ylabel='Relative Error [deg/sec]', title='Median error = '+str(e_angle_median)+'[deg/s]')
    ax.plot(e_angle)
    fig.tight_layout()
    fig.savefig(results_dir+'/relative_error'+FORMAT)
    
    # write to file
    file_out = open(os.path.join(results_dir, 'relative_error.csv'), 'w')
    file_out.write('# Relative errors over 1 meter distance \n')
    file_out.write('# translation error [m/s], angular error [rad/s], yaw error [rad/s], gravity error [rad/s] \n')
    for i in range(len(e_trans)):
        file_out.write(
            '%.8f, %.8f, %.8f, %.8f\n' %
            (e_trans[i], e_angle[i], e_yaw[i], e_gravity[i]))
    file_out.close()
    
      
def analyse_trajectory(results_dir, n_align_frames = 200, use_hand_eye_calib = True,
                       estimate_scale=True, boxplot_distances='', synthetic_realign_frames = False,
                       data_format = 'txt'):
    dataset_params = yaml.load(open(os.path.join(results_dir, 'dataset.yaml'),'r'))
    print('Analysing trajectory:')
    print('* use_hand_eye_calib = ' + str(use_hand_eye_calib))
    print('* estimate_scale = ' + str(estimate_scale))
    print('* dataset_is_blender = ' + str(dataset_params['dataset_is_blender']))
    
    # TODO: refactor!!!
    if data_format == 'txt':
        t_es, p_es, q_es, t_gt, p_gt, q_gt = traj_loading.load_dataset(results_dir, dataset_params['cam_delay'])
    elif data_format == 'csv':
        t_es, p_es, q_es, t_gt, p_gt, q_gt = traj_loading.load_dataset_csv(results_dir, dataset_params['cam_delay'])
    elif data_format == 'blender':
        t_es, p_es, q_es, t_gt, p_gt, q_gt = traj_loading.load_synthetic_dataset(results_dir)
    
    if dataset_params['dataset_is_blender']:
        if synthetic_realign_frames:
            T_cm = traj_loading.load_hand_eye_calib(dataset_params)
            compute_relative_errors(results_dir, t_es, p_es, q_es, t_gt, p_gt, q_gt, n_align_frames, estimate_scale, T_cm)
        else:
            analyse_synthetic_trajectory(results_dir, boxplot_distances)
     
    elif use_hand_eye_calib:
        T_cm = traj_loading.load_hand_eye_calib(dataset_params)
        compute_relative_errors(results_dir, t_es, p_es, q_es, t_gt, p_gt, q_gt, n_align_frames, estimate_scale, T_cm)
    else:
        compute_rms_errors(results_dir, p_es, q_es, p_gt, q_gt, n_align_frames, True)
        
        

    
def plot_boxplots_distances(results_dir, p_es, q_es, p_gt, q_gt, distances_str):
    distances = [int(item) for item in distances_str.split(',')]
    trans_errors_rel = []
    trans_errors = []
    ang_yaw_errors = []
    print ('-----------------------')
    for dist in distances:
        print('Distance = '+str(dist))
        max_error = 0.2*np.double(dist)
        T_cm = np.array([[1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])
        errors, e_trans, e_yaw, e_gravity, e_angle = traj_utils.compute_relative_error(p_es, q_es, p_gt, q_gt, T_cm, dist, max_error)
        e_trans_median = np.median(e_trans)
        print('dist = '+str(dist)+', median error = ' + str(e_trans_median) + ' ('+str(e_trans_median/np.double(dist)*100.0)+'%) ' + str(len(e_trans)) + 'measurements')
        trans_errors.append(e_trans)
        trans_errors_rel.append(e_trans/np.double(dist)*100.0)
        ang_yaw_errors.append(e_yaw)
        
    # absolute error
    fig = plt.figure(figsize=(6,2.5))
    ax = fig.add_subplot(111, xlabel='Distance traveled [m]', ylabel='Translation error [m]')
    ax.boxplot(trans_errors, 0, '')
    ax.set_xticks(np.arange(len(distances))+1)
    ax.set_xticklabels(distances)
    ax.legend()
    fig.tight_layout()
    fig.savefig(results_dir+'/boxplot_translation_error'+FORMAT, bbox_inches="tight")
    
    # relative error
    fig = plt.figure(figsize=(6,2.5))
    ax = fig.add_subplot(111, xlabel='Distance traveled [m]', ylabel='Translation error [\%]')
    ax.boxplot(trans_errors_rel, 0, '')
    ax.set_xticks(np.arange(len(distances))+1)
    ax.set_xticklabels(distances)
    ax.legend()
    fig.tight_layout()
    fig.savefig(results_dir+'/boxplot_translation_error_relative'+FORMAT, bbox_inches="tight")
      
    ## yaw orientation error
    fig = plt.figure(figsize=(6,2.5))
    ax = fig.add_subplot(111, xlabel='Distance traveled [m]', ylabel='Yaw error [deg]')
    ax.boxplot(ang_yaw_errors, 0, '')
    ax.set_xticks(np.arange(len(distances))+1)
    ax.set_xticklabels(distances)
    ax.legend()
    fig.tight_layout()
    ax.set_ylim(-0.1, 15)
    fig.savefig(results_dir+'/boxplot_yaw_error'+FORMAT, bbox_inches="tight") 
  

def compute_and_save_statistics(data_vec, label, yaml_filename):
    
    # TODO: Move to another file!    

    # Load statistics
    stats = dict()
    if os.path.exists(yaml_filename):
        stats = yaml.load(open(yaml_filename,'r'))
     
    # Compute new statistics
    stats[label] = dict()
    stats[label]['rmse']   = float(np.sqrt(np.dot(data_vec,data_vec) / len(data_vec)))
    stats[label]['mean']   = float(np.mean(data_vec))
    stats[label]['median'] = float(np.median(data_vec))
    stats[label]['std']    = float(np.std(data_vec))
    stats[label]['min']    = float(np.min(data_vec))
    stats[label]['max']    = float(np.max(data_vec))
    stats[label]['num_samples'] = int(len(data_vec))
    
    # Save updated statistics
    with open(yaml_filename,'w') as outfile:
        outfile.write(yaml.dump(stats, default_flow_style=False))

if __name__ == '__main__':
  
    # parse command line
    parser = argparse.ArgumentParser(description='''
    Analyse trajectory
    ''')
    parser.add_argument('--trace_dir', help='folder with the results', default='')
    parser.add_argument('--evaluation_type', help='', default='rms')
    parser.add_argument('--use_hand_eye_calib', help='', default='True')
    parser.add_argument('--align_num_frames', help='', default=-1)
    parser.add_argument('--align_type', help='', default='sim3') # option: 'sim3', 'se3'
    parser.add_argument('--boxplot_distances', help='', default='')
    parser.add_argument('--data_format', default='txt') # option: 'csv', 'txt', 'blender'
    args = parser.parse_args()
    
    ta = TrajectoryAnalysis( args.trace_dir,
                             data_format = args.data_format,
                             align_type = args.align_type,
                             align_num_frames = args.align_num_frames,
                             use_hand_eye_calib = args.use_hand_eye_calib, 
                             evaluation_type = args.evaluation_type,
                             boxplot_distances = args.boxplot_distances)
    ta.load_data()
    ta.analyse_trajectory()