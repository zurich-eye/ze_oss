#!/usr/bin/python3
"""
Zurich Eye
"""

import os
import yaml
import logging
import argparse
import numpy as np
import ze_trajectory_analysis.analyse as traj_analysis
import ze_py.transformations as tf
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import vikit_py.transformations as tf
from matplotlib import rc
from matplotlib.ticker import FuncFormatter
rc('font',**{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)

_EPS = np.finfo(float).eps * 4.0
FORMAT = '.pdf'

# Init logging.
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)
logger.info('Trajectory alignment example.')

# Load Data
data_dir = '/home/cfo/vin_ws/src/svo_gtsam/trace/data/20160407_1459_gtsam_vicon_asl_140313_vicon_aslam_2'

ta = traj_analysis.TrajectoryAnalysis(data_dir)
ta.load_data(data_dir, data_format='svo_gtsam')
ta.apply_hand_eye_calibration_to_groundtruth()
ta.align_trajectory('first_frame')
#ta.align_trajectory(align_type='se3', first_idx=0, last_idx=100)
ta.plot_aligned_trajectory()
#ta.compute_rms_errors()

t_es = ta.t_es
p_es = ta.p_es_aligned
q_es = ta.q_es_aligned
p_gt = ta.p_gt
q_gt = ta.q_gt
distances = ta.distances

# Load covariances:
cov_data = np.genfromtxt(os.path.join(data_dir, 'estimate_covariance.csv'), delimiter=',')
t_cov = cov_data[:,0]

# Load covariance
n = len(cov_data)
yaw_sigma_3 = np.zeros(n)
yaw_error = np.zeros(n)
roll_sigma_3 = np.zeros(n)
roll_error = np.zeros(n)
pitch_sigma_3 = np.zeros(n)
pitch_error = np.zeros(n)
error_pos_W = np.zeros((3,n))
error_pos_W_sigma_3 = np.zeros((3,n))
nees_rot = np.zeros(n)
nees_pos = np.zeros(n)
nees_se3 = np.zeros(n)
for i in range(1000):
    #assert t_cov[i] == t_es[i]
    Cov_T_B = np.reshape(cov_data[i,1:],(6,6))
    Cov_R_B = Cov_T_B[:3,:3]
    Cov_t_B = Cov_T_B[3:6,3:6]        
    p_W_Bes = p_es[i,:]
    p_W_Bgt = p_gt[i,:]
    R_W_Bes = tf.quaternion_matrix(q_es[i,:])[:3,:3]
    R_W_Bgt = tf.quaternion_matrix(q_gt[i,:])[:3,:3]
    Cov_R_W = np.dot(R_W_Bes, np.dot(Cov_R_B, np.transpose(R_W_Bes)))
    Cov_T_W = np.dot(R_W_Bes, np.dot(Cov_t_B, np.transpose(R_W_Bes)))
    yaw_sigma_3[i] = np.sqrt(Cov_R_W[2,2])*3.0*180/np.pi
    pitch_sigma_3[i] = np.sqrt(Cov_R_W[1,1])*3.0*180/np.pi
    roll_sigma_3[i] = np.sqrt(Cov_R_W[0,0])*3.0*180/np.pi
    R_Bgt_Bes = np.dot(R_W_Bgt, np.transpose(R_W_Bes))
    
    yaw_error[i], pitch_error[i], roll_error[i] = tf.euler_from_matrix(R_Bgt_Bes, 'rzyx')
    
    # compute normalized estimation error squared (in estimated body frame)
    error_rot_B = tf.logmap_so3(np.transpose(R_Bgt_Bes))
    error_pos_B = np.dot(np.transpose(R_W_Bes), (p_W_Bgt - p_W_Bes))
    error_se3_B = np.concatenate((error_rot_B, error_pos_B))
    nees_rot[i] = np.dot(error_rot_B, np.dot(np.linalg.inv(Cov_R_B), error_rot_B))        
    nees_pos[i] = np.dot(error_pos_B, np.dot(np.linalg.inv(Cov_t_B), error_pos_B))
    nees_se3[i] = np.dot(error_se3_B, np.dot(np.linalg.inv(Cov_T_B), error_se3_B))

    # translation error in world coordiantes
    error_pos_W[:,i] = p_W_Bgt - p_W_Bes
    error_pos_W_sigma_3[0,i] = np.sqrt(Cov_T_W[0,0])*3.0
    error_pos_W_sigma_3[1,i] = np.sqrt(Cov_T_W[1,1])*3.0
    error_pos_W_sigma_3[2,i] = np.sqrt(Cov_T_W[2,2])*3.0
    
    
yaw_error *= 180/np.pi
pitch_error *= 180/np.pi
roll_error *= 180/np.pi
n_max = 1000

# rotation error
D = distances[:n_max]
y_lim = 5 #args.rpy_ylim
fig = plt.figure(figsize=(6,8))
gs1 = gridspec.GridSpec(3, 1)
gs1.update(wspace=0.005) # set the spacing between axes.
ax = fig.add_subplot(611, ylabel='Err. Yaw [deg]')
ax.locator_params(axis = 'y', nbins = 4)
ax.yaxis.set_major_formatter(FuncFormatter(lambda y, pos: '%.2f'%y))
ax.plot(D, yaw_sigma_3[:n_max], 'r--')
ax.plot(D, -yaw_sigma_3[:n_max], 'r--')
ax.plot(D, yaw_error[:n_max], 'r-', lw=2)
ax.set_xticks([])
ax.set_ylim([-y_lim,y_lim])
y_lim = 4 #args.rpy_ylim
ax = fig.add_subplot(612, ylabel='Err. Pitch [deg]')
ax.locator_params(axis = 'y', nbins = 4)
ax.yaxis.set_major_formatter(FuncFormatter(lambda y, pos: '%.2f'%y))
ax.plot(D, pitch_sigma_3[:n_max], 'g--')
ax.plot(D, -pitch_sigma_3[:n_max], 'g--')
ax.plot(D, pitch_error[:n_max], 'g-', lw=2)
ax.set_xticks([])
ax.set_ylim([-y_lim,y_lim])
ax = fig.add_subplot(613, ylabel='Err. Roll [deg]')
ax.locator_params(axis = 'y', nbins = 4)
ax.yaxis.set_major_formatter(FuncFormatter(lambda y, pos: '%.2f'%y))
ax.plot(D, roll_sigma_3[:n_max], 'b--')
ax.plot(D, -roll_sigma_3[:n_max], 'b--')
ax.plot(D, roll_error[:n_max], 'b-', lw=2)
ax.set_ylim([-y_lim,y_lim])
ax.set_xticks([])

# translation error
y_lim = 0.9
ax = fig.add_subplot(614, ylabel='Err. x [m]')
ax.locator_params(axis = 'y', nbins = 4)
ax.yaxis.set_major_formatter(FuncFormatter(lambda y, pos: '%.2f'%y))
ax.plot(D, error_pos_W_sigma_3[0,:n_max], 'r--')
ax.plot(D, -error_pos_W_sigma_3[0,:n_max], 'r--')
ax.plot(D, error_pos_W[0,:n_max], 'r-', lw=2)
ax.set_xticks([])
ax.set_ylim([-y_lim,y_lim])
ax = fig.add_subplot(615, ylabel='Err. y [m]')
ax.locator_params(axis = 'y', nbins = 4)
ax.yaxis.set_major_formatter(FuncFormatter(lambda y, pos: '%.2f'%y))
ax.plot(D, error_pos_W_sigma_3[1,:n_max], 'g--')
ax.plot(D, -error_pos_W_sigma_3[1,:n_max], 'g--')
ax.plot(D, error_pos_W[1,:n_max], 'g-', lw=2)    
ax.set_ylim([-y_lim,y_lim])
ax.set_xticks([])
ax = fig.add_subplot(616, xlabel='Distance Travelled [m]', ylabel='Err. z [m]')
ax.locator_params(axis = 'y', nbins = 4)
ax.yaxis.set_major_formatter(FuncFormatter(lambda y, pos: '%.2f'%y))
ax.plot(D, error_pos_W_sigma_3[2,:n_max], 'b--')
ax.plot(D, -error_pos_W_sigma_3[2,:n_max], 'b--')
ax.plot(D, error_pos_W[2,:n_max], 'b-', lw=2)
ax.set_ylim([-y_lim,y_lim])
ax.tick_params('x',top='off')
fig.tight_layout()
fig.savefig(os.path.join(data_dir,'consistency_single_run'+FORMAT), bbox_inches="tight")

# write to file
file_out = open(os.path.join(data_dir, 'consistency_errors.csv'), 'w')
file_out.write('# trans_error -x, -y, -z, rot_error -yaw, -pitch, -roll\n')
for i in range(len(yaw_error)):
    file_out.write(
        '%.8f, %.8f, %.8f, %.8f, %.8f, %.8f\n' %
        (error_pos_W[0,i], error_pos_W[1,i], error_pos_W[2,i], yaw_error[i], pitch_error[i], roll_error[i])) 
file_out.close()

# NEES Rot and Pos
fig = plt.figure(figsize=(6,3))
ax = fig.add_subplot(211, ylabel='Rot. NEES')
ax.plot(nees_rot)
ax = fig.add_subplot(212, ylabel='Pos. NEES', xlabel='Keyframes')
ax.plot(nees_pos)
fig.savefig(os.path.join(data_dir,'consistency_nees_posrot'+FORMAT), bbox_inches="tight")

# NEES Pose
fig = plt.figure(figsize=(6,1.5))
ax = fig.add_subplot(111, ylabel='Pose NEES', xlabel='Keyframes')
ax.plot(nees_se3)
fig.savefig(os.path.join(data_dir,'consistency_pose'+FORMAT), bbox_inches="tight")

# write to file
file_out = open(os.path.join(data_dir, 'consistency_nees.csv'), 'w')
file_out.write('# NEES orientation, NEES position \n')
for i in range(len(nees_rot)):
    file_out.write('%.8f, %.8f, %.8f\n' % (nees_rot[i], nees_pos[i], nees_se3[i])) 
file_out.close()