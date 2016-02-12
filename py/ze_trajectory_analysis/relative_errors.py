#!/usr/bin/env python3
"""
Zurich Eye
"""

import os
import logging
import numpy as np

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
            + " --skip_frames=" + str(skip_frames)
                                           
        logger.info("Executing command: "+cmd)
        os.system(cmd)
        
if __name__ == '__main__':        
    
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger(__name__)
    
    data_dir = '/home/cfo/Desktop/result_example'
    compute_relative_errors(data_dir,
                            segment_lengths = [5, 10, 20, 30],
                            skip_frames = 1,
                            format_gt = 'euroc',
                            format_es = 'pose')