#!/usr/bin/python
"""
Zurich Eye
"""

import os
import yaml
import logging
import argparse
import numpy as np
import numpy.testing as npt
import matplotlib.pyplot as plt
import ze_trajectory_analysis.align as align_trajectory
import ze_trajectory_analysis.utils as utils
import ze_trajectory_analysis.load as traj_loading
import ze_trajectory_analysis.plot as traj_plot
import ze_py.transformations as tf
           
class TrajectoryAnalysis:
    
    def __init__(self, result_dir):
        """Analyse trajectory. 
        
        result_dir: The results of the analysis are saved at this location.
        """
        self.logger = logging.getLogger(__name__)
        self.reset()
        self.result_dir = utils.check_folder_exists(result_dir)
        self.statistics_filename = os.path.join(self.result_dir, 'statistics.yaml')
       
    def reset(self):
        self.data_loaded = False
        self.data_aligned = False
          
    def load_data(self, data_dir, data_format='csv',
                  filename_gt = 'traj_gt.csv', filename_es = 'traj_es.csv', 
                  filename_matches = 'traj_es_gt_matches.csv', rematch_timestamps = False, 
                  match_timestamps_offset = 0.0, rematch_timestamps_max_difference_sec = 0.02):
        """Loads the trajectory data.
        
        The resuls {p_es, q_es, p_gt, q_gt} is synchronized and has the same length.
        
        filename_matches:   Optional, if it does not exist, it is created.
        rematch_timestamps: If True, finds matching timestamps between estimated
                            and groundtruth trajectory. Saves the results to
                            filename_matches.
        """
        utils.check_file_exists(os.path.join(data_dir, filename_gt))
        utils.check_file_exists(os.path.join(data_dir, filename_es))
 
        # Load trajectory data
        self.logger.info('Loading trajectory data..')
        if data_format == 'csv':
            self.t_es, self.p_es, self.q_es, self.t_gt, self.p_gt, self.q_gt =\
                traj_loading.load_dataset_csv(data_dir, filename_gt, filename_es,
                                              filename_matches, rematch_timestamps,
                                              match_timestamps_offset,
                                              rematch_timestamps_max_difference_sec)
        else:
            raise ValueError('data_format' + self.data_format + ' not known.')
        self.logger.info('...done.')
        
        # Compute distance along trajectory from start to each measurement.
        self.distances = utils.get_distance_from_start(self.p_gt) 
        traj_plot.plot_travelled_distance(self.distances, self.result_dir)
        
        self.data_dir = data_dir
        self.data_loaded = True
        
    def plot_estimator_results(self, data_dir, data_format='swe', filename = 'traj_es.csv'):
        self.logger.info('Loading estimator data')
        filename = utils.check_file_exists(os.path.join(data_dir, filename))
        if data_format == 'swe':
            self.estimator_ts, self.vel_es, self.bias_gyr_es, self.bias_acc_es = \
                traj_loading.load_estimator_results(filename)
        else:
            self.logger.error("Estimator results format \""+data_format+"\" not known.")
            return
            
        # Plot estimated biases
        self.logger.info('Plotting estimator data')
        traj_plot.plot_imu_biases(self.estimator_ts, self.bias_gyr_es,
                                  self.bias_acc_es, self.result_dir)
               
    def align_trajectory(self, align_type = 'se3', first_idx = 0, last_idx = -1):
        """Align trajectory segment with ground-truth trajectory.
        
        first_idx: First index of data to align.
        last_idx:  Last index of data to align.
        align_type: 'se3' - translation and orientation
                    'sim3' - translation, orientation, and scale
                    'first_frame' - align just the first frame, alignment = Identity.
        """
        if not self.data_loaded:
            raise ValueError("You need to first load the data")           

        if last_idx < 0:
            if first_idx == 0:
                self.logger.info('Align trajectory using all frames.')
            else:
                self.logger.info('Align trajectory from index '+str(first_idx)+' to the end.')
            last_idx = len(self.p_es)
        else:
            self.logger.info('Align trajectory from index ' + str(first_idx) + \
                             ' to ' + str(first_idx) + '.')
         
        #
        # TODO: Apply hand-eye calibration to estimted trajectory
        #             
        
        # Compute alignment parameters
        if align_type == 'sim3':
            self.logger.info('Align Sim3 - rotation, translation and scale.')
            self.scale, self.rot, self.trans = \
                align_trajectory.align_sim3(self.p_gt[first_idx:last_idx,:], self.p_es[first_idx:last_idx,:])
        elif align_type == 'se3':
            self.logger.info('Align SE3 - rotation, translation and scale.')
            self.rot, self.trans = \
                align_trajectory.align_se3(self.p_es[first_idx:last_idx,:], self.p_gt[first_idx:last_idx,:])
            self.rot = np.transpose(self.rot)
            self.scale = 1.0 
        elif align_type == 'first_frame':
            self.trans = np.zeros((3,))
            self.rot = np.eye(3)            
            self.scale = 1.0
            
        self.logger.info('Alignment translation: \n' + str(self.trans))
        self.logger.info('Alignment rotation: \n' + str(self.rot))
        self.logger.info('Alignment scale: \n' + str(self.scale))
        npt.assert_almost_equal(np.linalg.det(self.rot), 1.0)
            
        self.logger.info('Apply alignment to estimated trajectory to fit groundtruth')
        q = tf.quaternion_from_matrix(tf.convert_3x3_to_4x4(self.rot))
        self.p_es_aligned = np.zeros(np.shape(self.p_es))
        self.q_es_aligned = np.zeros(np.shape(self.q_es))
        for i in range(np.shape(self.p_es)[0]):
            self.p_es_aligned[i,:] = self.scale*self.rot.dot(self.p_es[i,:]) + self.trans
            self.q_es_aligned[i,:] = tf.quaternion_multiply(q, self.q_es[i,:])
        
        self.align_first_idx = first_idx
        self.align_last_idx = last_idx
        self.data_aligned = True         
            
            
    def plot_aligned_trajectory(self, plot_format = 'png'):
        """Saves a plot of the aligned trajectory to 'result_dir'"""
        
        if not self.data_aligned:
            raise ValueError("You need to first load and align the data")   
            
        self.logger.info('Save a plot of the aligned trajectory to: ' + self.result_dir)
        traj_plot.plot_trajectory(self.result_dir, self.p_gt, self.p_es_aligned,
                                  self.align_first_idx, self.align_last_idx)
                                  
    def compute_rms_errors(self):
        """Compute Root Mean Square Error (RMSE) of aligned trajectory w.r.t
        groundtruth trajectory.
        """
        
        if not self.data_aligned:
            raise ValueError("You need to first load and align the data")   
        
        # position error 
        e_trans = (self.p_gt-self.p_es_aligned)  
        e_trans_euclidean = np.sqrt(np.sum(e_trans**2, 1))
        
        self.logger.info('Compute orientation error')
        e_rot = np.zeros((len(e_trans_euclidean,)))
        e_rpy = np.zeros(np.shape(self.p_es_aligned))
        for i in range(np.shape(self.p_es_aligned)[0]):
            R_we = tf.matrix_from_quaternion(self.q_es_aligned[i,:])
            R_wg = tf.matrix_from_quaternion(self.q_gt[i,:])
            R_ge = np.dot(np.linalg.inv(R_wg), R_we)
            e_rpy[i,:] = tf.euler_from_matrix(R_ge, 'rzyx')
            e_rot[i] = np.linalg.norm(tf.logmap_so3(R_ge[:3,:3]))
        
        self.logger.info('Compute scale drift')
        motion_gt = np.diff(self.p_gt, 0)
        motion_es = np.diff(self.p_es_aligned, 0)
        dist_gt = np.sqrt(np.sum(np.multiply(motion_gt,motion_gt),1))
        dist_es = np.sqrt(np.sum(np.multiply(motion_es,motion_es),1))
        e_scale_rel = np.divide(dist_es,dist_gt)-1.0

        self.logger.info('Save error plots')
        traj_plot.plot_translation_error(self.distances, e_trans, self.result_dir)
        traj_plot.plot_rotation_error(self.distances, e_rpy, self.result_dir)
        traj_plot.plot_scale_error(self.distances, e_scale_rel*100.0, self.result_dir)
        
        # compute error statistics:
        self.compute_and_save_statistics(e_trans_euclidean, 'trans')
        self.compute_and_save_statistics(e_rot, 'rot')
        self.compute_and_save_statistics(e_scale_rel, 'scale')
    
    def get_trajectory_length(self):
        assert(self.data_loaded)
        return self.distances[-1]
        
    def compute_and_save_statistics(self, data_vec, label):
        # Load statistics
        stats = dict()
        if os.path.exists(self.statistics_filename):
            stats = yaml.load(open(self.statistics_filename,'r'))
         
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
        with open(self.statistics_filename,'w') as outfile:
            outfile.write(yaml.dump(stats, default_flow_style=False))

if __name__=='__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='Compute relative errors')
    parser.add_argument('--data_dir', help='folder with the results',
                        default='')
    parser.add_argument('--format_gt', help='format groundtruth {swe,pose,euroc}',
                        default='pose')
    parser.add_argument('--format_es', help='format estimate {swe,pose,euroc}',
                        default='pose')
    options = parser.parse_args()    
    
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger(__name__)
    logger.info('Compute relative errors')

    if options.data_dir:
        ta = TrajectoryAnalysis(result_dir = options.data_dir)
        
        ta.load_data(data_dir=options.data_dir,
                     data_format='csv')
                     
        ta.align_trajectory('sim3', 0, -1)
        ta.plot_aligned_trajectory()
        ta.compute_rms_errors()
        ta.plot_estimator_results(options.data_dir,
                                  data_format=options.format_es,
                                  filename = 'traj_es.csv')