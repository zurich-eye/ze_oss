#!/usr/bin/python

import argparse
import numpy as np

def read_nanosecond_timestamps_from_csv_file(filename, col=0, delimiter=','):
    return np.genfromtxt(filename, usecols=(col), delimiter=delimiter, dtype=np.longlong, skip_header=1)
    
def associate(stamps_es, stamps_gt, offset_sec = 0.0, max_difference_sec = 0.02):
    """
    Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim 
    to find the closest match for every input tuple.
    
    Input:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
    max_difference -- search radius for candidate generation

    Output:
    matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))
    
    """
    first_keys = stamps_es.tolist()
    second_keys = stamps_gt.tolist()
    offset = np.longlong(offset_sec*1e9)
    max_difference = np.longlong(max_difference_sec*1e9)
    print('Computing potential matches...')
    potential_matches = [(abs(a - (b + offset)), a, b) 
                         for a in first_keys 
                         for b in second_keys 
                         if abs(a - (b + offset)) < max_difference]
    print('...done')
    potential_matches.sort()
    matches = []
    print('Find best matches...')
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))
    print('...done.')
    matches.sort()
    return matches
    
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
    stamps_es = read_nanosecond_timestamps_from_csv_file(args.file_es, args.file_es_col, args.delimiter)
    stamps_gt = read_nanosecond_timestamps_from_csv_file(args.file_gt, args.file_gt_col, args.delimiter)
    
    # Match data.
    matches = associate(stamps_es, stamps_gt, args.offset_sec, args.max_difference_sec)    
    
    # Write to file.
    print('Writing to file...')
    filename = 'matches.csv' 
    associates_file = open(filename, 'w')     
    associates_file.write('# Estimate: %s, Groundtruth: %s\n' % (args.file_es, args.file_gt))                        
    for a,b in matches:
      associates_file.write('{:d},{:d}\n'.format(a, b))
    print('...done. Wrote ' + str(len(matches)) + ' matches to file: ' + filename)
        
