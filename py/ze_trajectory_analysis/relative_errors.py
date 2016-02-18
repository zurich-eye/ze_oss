#!/usr/bin/env python3
"""
Zurich Eye
"""

import os
import logging
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
from pylab import setp

def load_relative_errors_from_file(data_dir, 
                                   segment_length,
                                   filename_result_prefix = 'traj_relative_errors'):
                                       
    data = np.genfromtxt(os.path.join(data_dir, filename_result_prefix+'_'+str(segment_length)+'.csv'),
                         delimiter=',', dtype=np.float64, skip_header=1)
    assert data[0, 7] == segment_length
    rel_pos_errors = np.abs(data[:,1:4]) # / segment_length
    rel_rot_errors = np.abs(data[:,4:7]) # / segment_length

    rel_pos_errors_norm = np.sqrt(np.sum(rel_pos_errors**2, 1))
    rel_roll_pitch_errors = np.sqrt(np.sum(rel_rot_errors[:,0:2]**2, 1))
    rel_yaw_errors = rel_rot_errors[:,2]
    
    return rel_pos_errors_norm, rel_roll_pitch_errors, rel_yaw_errors
    
def _set_boxplot_colors(boxplot_object, color):
    setp(boxplot_object['boxes'][0], color=color)
    setp(boxplot_object['caps'][0], color=color)
    setp(boxplot_object['caps'][1], color=color)
    setp(boxplot_object['whiskers'][0], color=color)
    setp(boxplot_object['whiskers'][1], color=color)
    setp(boxplot_object['fliers'], color=color)
    setp(boxplot_object['medians'][0], color=color)
    
def plot_relative_errors(data_dirs, segment_lengths):
    """data_dirs can contain a list of result directories. The plots are stored
    in the first directory.        
    """    
    
    colors = ['blue', 'black', 'green', 'red', 'mangenta', 'cyan', 'orange']

    # Precompute position of boxplots in plot.     
    n_exp = len(data_dirs)
    n_dist = len(segment_lengths)
    spacing = 1
    pos = np.arange(0, n_dist*(n_exp+spacing), (n_exp+spacing))
    
    def _ax_formatting(ax):
       ax.yaxis.grid(ls='--', color='0.7')
       ax.yaxis.set_major_formatter(FuncFormatter(lambda y, pos: '%.2f'%y)) 
       
    # Init axes
    fig = plt.figure(figsize=(8,7))
    ax_pos = fig.add_subplot(311, xlabel='Distance traveled [m]', ylabel='Translation error [m]')
    ax_yaw = fig.add_subplot(312, xlabel='Distance traveled [m]', ylabel='Yaw error [deg]')
    ax_rap = fig.add_subplot(313, xlabel='Distance traveled [m]', ylabel='Roll and Pitch error [deg]')
    _ax_formatting(ax_yaw)
    _ax_formatting(ax_pos)    
    _ax_formatting(ax_rap)
     
    dummy_plots_pos = []
    dummy_plots_yaw = []
    dummy_plots_rap = []
    labels = []
    
    for idx_exp, data_dir in enumerate(data_dirs):
        
        # The dummy plots are used to create the legends.
        dummy_plot_pos = ax_pos.plot([1,1], '-', color=colors[idx_exp])
        dummy_plots_pos.append(dummy_plot_pos[0])
        dummy_plot_yaw = ax_yaw.plot([1,1], '-', color=colors[idx_exp])
        dummy_plots_yaw.append(dummy_plot_yaw[0])
        dummy_plot_rap = ax_yaw.plot([1,1], '-', color=colors[idx_exp])
        dummy_plots_rap.append(dummy_plot_rap[0])
        labels.append('exp-'+str(idx_exp)) # TODO: Read label from data_dir
        

        for idx_segment_length, segment_length in enumerate(segment_lengths):
            e_pos, e_rap, e_yaw = load_relative_errors_from_file(data_dir, segment_length)
            pb = ax_pos.boxplot(e_pos, False, '',
                                positions=[pos[idx_segment_length] + idx_exp], widths=0.8)                    
            _set_boxplot_colors(pb, colors[idx_exp])
            pb = ax_yaw.boxplot(e_yaw * 360.0 / np.pi, False, '',
                                positions=[idx_exp + pos[idx_segment_length]], widths=0.8)
            _set_boxplot_colors(pb, colors[idx_exp])
            pb = ax_rap.boxplot(e_rap * 360.0 / np.pi, False, '',
                                positions=[idx_exp + pos[idx_segment_length]], widths=0.8)
            _set_boxplot_colors(pb, colors[idx_exp])
                   
    # Set x-labels
    ax_pos.set_xticks(pos+0.5*n_exp-0.5)
    ax_pos.set_xticklabels(segment_lengths)
    ax_pos.set_xlim(xmin=pos[0]-2, xmax=pos[-1]+5)
    ax_yaw.set_xticks(pos+0.5*n_exp-0.5)
    ax_yaw.set_xticklabels(segment_lengths)
    ax_yaw.set_xlim(xmin=pos[0]-2, xmax=pos[-1]+5)    
    ax_rap.set_xticks(pos+0.5*n_exp-0.5)
    ax_rap.set_xticklabels(segment_lengths)
    ax_rap.set_xlim(xmin=pos[0]-2, xmax=pos[-1]+5)

    # create legend
    ax_pos.legend(dummy_plots_yaw, labels, loc='upper right', ncol=2)
    for p in dummy_plots_pos:
        p.set_visible(False)
    for p in dummy_plots_yaw:
        p.set_visible(False)
    for p in dummy_plots_rap:
        p.set_visible(False)
    
    fig.tight_layout()
    fig.savefig(os.path.join(data_dirs[0], 'traj_relative_errors.png'), bbox_inches="tight")



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
            + " --skip_frames=" + str(skip_frames) \
            + " --least_squares_align"
                                           
        logger.info("Executing command: "+cmd)
        os.system(cmd)
        
if __name__=='__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    Analyse trajectory
    ''')
    parser.add_argument('--data_dir', help='folder with the results', default='')
    options = parser.parse_args()    
    
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger(__name__)
    
    if options.data_dir:
        compute_relative_errors(options.data_dir,
                                segment_lengths = [5, 10, 20, 30, 50],
                                skip_frames = 1,
                                format_gt = 'euroc',
                                format_es = 'pose')

        plot_relative_errors([options.data_dir], segment_lengths = [5, 10, 20, 30, 50])