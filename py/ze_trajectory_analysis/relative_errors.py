#!/usr/bin/env python3
"""
Zurich Eye
"""

import os
import logging
import numpy as np
import matplotlib.pyplot as plt
from pylab import setp

def plot_relative_errors(relative_errors):
    colors = ['blue', 'black', 'green', 'red', 'mangenta', 'cyan', 'orange']
     
    n_exp = 1
    n_dist = len(relative_errors.keys())
    spacing = 2
    pos = np.arange(0, n_dist*(n_exp+spacing), (n_exp+spacing))
    
    # relative error
    fig_trans = plt.figure(figsize=(8,7))
    ax_trans = fig_trans.add_subplot(211, xlabel='Distance traveled [m]', ylabel='Rel. Transl. error [%]')
    ax_trans.yaxis.grid(ls='--', color='0.7')
    
    #fig_yaw = plt.figure(figsize=(6,2.5))
    ax_yaw = fig_trans.add_subplot(212, xlabel='Distance traveled [m]', ylabel='Rel. Rot. error [deg]')
    ax_yaw.yaxis.grid(ls='--', color='0.7')
    
    #fig_g = plt.figure(figsize=(6,2.5))
    #ax_g = fig_trans.add_subplot(313, xlabel='Distance traveled [m]', ylabel='Avg. Pitch and Roll err. [deg]')
    #ax_g.yaxis.grid(ls='--', color='0.7')
    
    def set_boxplot_colors(boxplot_object, color):
        setp(boxplot_object['boxes'][0], color=color)
        setp(boxplot_object['caps'][0], color=color)
        setp(boxplot_object['caps'][1], color=color)
        setp(boxplot_object['whiskers'][0], color=color)
        setp(boxplot_object['whiskers'][1], color=color)
        setp(boxplot_object['fliers'][0], color=color)
        setp(boxplot_object['fliers'][1], color=color)
        setp(boxplot_object['medians'][0], color=color)
    
    dummy_plots_trans = []
    dummy_plots_yaw = []
    dummy_plots_g = []
    labels = []
    #for idx_exp, e in enumerate(exp):  
    idx_exp = 0
    #D = yaml.load(open(os.path.join('data', e, 'relative_errors.yaml'), 'r'))
    dummy_plot_trans = ax_trans.plot([1,1], '-', color=colors[idx_exp])
    dummy_plots_trans.append(dummy_plot_trans[0])
    dummy_plot_yaw = ax_yaw.plot([1,1], '-', color=colors[idx_exp])
    dummy_plots_yaw.append(dummy_plot_yaw[0])
    dummy_plot_g = ax_yaw.plot([1,1], '-', color=colors[idx_exp])
    dummy_plots_g.append(dummy_plot_g[0])
    
    labels.append('label')
    i = 0
    distances = list()
    for dist, data in sorted(relative_errors.items()):
        print('dist = ' + str(dist))
        pb = ax_trans.boxplot(data['rel_pos_err']*100.0, False, '', positions=[pos[i]+idx_exp], widths=0.8)
        #set_boxplot_colors(pb, colors[idx_exp])
    
        pb = ax_yaw.boxplot(data['rel_rot_err']*100.0, False, '', positions=[pos[i]+idx_exp], widths=0.8)
        #set_boxplot_colors(pb, colors[idx_exp])
        
        i += 1
        distances.append(dist)
        
    
    ax_trans.set_xticks(pos+0.5*n_exp-0.5)
    ax_trans.set_xticklabels(distances)
    ax_trans.set_xlim(xmin=pos[0]-2, xmax=pos[-1]+5)
    
    ax_yaw.set_xticks(pos+0.5*n_exp-0.5)
    ax_yaw.set_xticklabels(distances)
    ax_yaw.set_xlim(xmin=pos[0]-2, xmax=pos[-1]+5)

    
    # create legend
    ax_trans.legend(dummy_plots_yaw, labels, loc='upper left', ncol=2)
    for p in dummy_plots_trans:
        p.set_visible(False)
        
    
    for p in dummy_plots_yaw:
        p.set_visible(False)
    #    
    #ax_g.legend(dummy_plots_g, labels, loc='upper left')
    for p in dummy_plots_g:
        p.set_visible(False)
    
    fig_trans.tight_layout()
    #fig_trans.savefig(os.path.join(args.trace_dir, 'error_comparison'+FORMAT), bbox_inches="tight")
    #
    #fig_yaw.tight_layout()
    #fig_yaw.savefig('vicon_yaw_error_comparison'+FORMAT, bbox_inches="tight")
    #
    #fig_g.tight_layout()
    #fig_g.savefig('vicon_gravity_error_comparison'+FORMAT, bbox_inches="tight")
    
    

def compute_relative_errors(data_dir,
                            segment_lengths = [10, 50, 100, 200],
                            skip_frames = 10,
                            format_gt = 'pose',
                            format_es = 'pose',
                            filename_gt = 'traj_gt.csv',
                            filename_es = 'traj_es.csv',
                            filename_result_prefix = 'traj_relative_errors',
                            match_timestamps_offset = 0.0,
                            match_timestamps_max_difference_sec = 0.02):
    logger = logging.getLogger(__name__)
    
    relative_errors = dict()
    for segment_length in segment_lengths:
        cmd = "rosrun ze_trajectory_analysis kitti_evaluation" \
            + " --data_dir=" + data_dir \
            + " --filename_es=" + filename_es \
            + " --filename_gt=" + filename_gt \
            + " --format_es=" + format_es \
            + " --format_gt=" + format_gt \
            + " --filename_result_prefix=" + filename_result_prefix \
            + " --offset_sec=" + str(match_timestamps_offset) \
            + " --max_difference_sec=" + str(match_timestamps_max_difference_sec) \
            + " --segment_length=" + str(segment_length) \
            + " --skip_frames=" + str(skip_frames)
                                           
        logger.info("Executing command: "+cmd)
        os.system(cmd)
        
        # load data
        result = np.genfromtxt(os.path.join(data_dir, filename_result_prefix+'_'+str(segment_length)+'.csv'),
                               delimiter=',', dtype=np.float64, skip_header=1)
        relative_errors[segment_length] = dict()
        relative_errors[segment_length]['rel_rot_err'] = result[:,1]
        relative_errors[segment_length]['rel_pos_err'] = result[:,2]
        
    return relative_errors
        
if __name__ == '__main__':        
    
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger(__name__)
    
    data_dir = '/home/cfo/Desktop/result_example'
    relative_errors = compute_relative_errors(data_dir,
                                              segment_lengths = [5, 10, 20, 30, 50],
                                              skip_frames = 1,
                                              format_gt = 'euroc',
                                              format_es = 'pose')
                                              
    
                                              
    plot_relative_errors(relative_errors)