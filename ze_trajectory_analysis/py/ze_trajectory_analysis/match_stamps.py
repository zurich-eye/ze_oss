#!/usr/bin/python
"""
Zurich Eye
"""

import logging
import argparse
import numpy as np
import ze_trajectory_analysis.utils as utils

def associate(stamps_es, stamps_gt, offset_sec = 0.0, max_difference_sec = 0.02):
    """
    Associate two lists of nanosecond timestamps. As the time stamps never match
    exactly, we aim to find the closest match for every input tuple.
    
    Input:
    stamps_es -- nanosecond timestamps of estimates
    stamps_gt -- nanosecond timestamps of estimates
    offset_sec -- time offset between both dictionaries (e.g., to model the delay between the sensors)
    max_difference_sec -- search radius for candidate generation

    Output:
    matches -- list of matched stamps [(stamp_es, stamp_gt), (stamp_es, stamp_gt)]
    
    """
    logger = logging.getLogger(__name__)
    es_keys = stamps_es.tolist()
    gt_keys = stamps_gt.tolist()
    offset = np.longlong(offset_sec*1e9)
    max_difference = np.longlong(max_difference_sec*1e9)
    logger.info("Computing potential matches...")
    potential_matches = [(abs(a - (b + offset)), a, b) 
                         for a in es_keys 
                         for b in gt_keys 
                         if abs(a - (b + offset)) < max_difference]
    logger.info('...done.')
    potential_matches.sort()
    matches = []
    logger.info('Find best matches...')
    for diff, a, b in potential_matches:
        if a in es_keys and b in gt_keys:
            es_keys.remove(a)
            gt_keys.remove(b)
            matches.append((a, b))
    logger.info('...done.')
    matches.sort()
    return matches
    
def write_matches_to_file(filename, matches):
    logger = logging.getLogger(__name__)
    associates_file = open(filename, 'w')     
    associates_file.write('# Estimate Groundtruth\n')                       
    for a,b in matches:
      associates_file.write('{:d},{:d}\n'.format(a, b))
    logger.info('Wrote ' + str(len(matches)) + ' matches to file: ' + filename)
    
if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script takes two data files with timestamps and associates them   
    ''')
    parser.add_argument('file_es', help='ESTIMATE estimated trajectory (format: timestamp data)')
    parser.add_argument('file_gt', help='GROUNDTRUTH (format: timestamp data)')
    parser.add_argument('--file_es_col', help='Timestamp column in file 1', default=0)
    parser.add_argument('--file_gt_col', help='Timestamp column in file 2', default=0)
    parser.add_argument('--offset_sec', help='time offset added to the timestamps of the second file (default: 0.0)', default=0.0)
    parser.add_argument('--max_difference_sec', help='maximally allowed time difference for matching entries (default: 0.02)', default=0.02)
    parser.add_argument('--delimiter', help='csv delimiter', default=',')
    args = parser.parse_args()

    # Load data.
    stamps_es = utils.read_nanosecond_timestamps_from_csv_file( \
                    args.file_es, args.file_es_col, args.delimiter)
    stamps_gt = utils.read_nanosecond_timestamps_from_csv_file( \
                    args.file_gt, args.file_gt_col, args.delimiter)
    
    # Match data.
    matches = associate(stamps_es, stamps_gt, args.offset_sec, args.max_difference_sec)    
    
    # Write to file.
    write_matches_to_file('matches.csv', matches)
        
