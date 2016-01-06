#!/usr/bin/python

# This example shows how to:
# 1) Load two trajectories
# 2) Find closest timestamps
# 3) Align the trajectories
# 4) Compute error statistics

import logging
import ze_py.test.utils as test_utils
import ze_trajectory_analysis.analyse as traj_analysis

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

logger.info('test')
data_dir = test_utils.get_test_data_dir('ze_applanix_gt_data')

ta = traj_analysis.TrajectoryAnalysis()
ta.load_data(data_dir, rematch_timestamps = True)