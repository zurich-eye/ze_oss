#!/usr/bin/python
"""
Zurich Eye
"""

import os
import logging
import argparse
import numpy as np
import matplotlib
# Force matplotlib to not use any Xwindows backend.
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.ticker import FuncFormatter
import ze_py.plot_utils as plot_utils
import ze_trajectory_analysis.utils as utils
#from mpl_toolkits.mplot3d import Axes3D
#from matplotlib import rc
#rc('font',**{'family':'serif','serif':['Cardo']})
#rc('text', usetex=True)

FORMAT = '.pdf'

def plot_pointcloud_3d(results_dir, p_gt, p_es, scale, rot, trans):
    
    m = np.loadtxt(os.path.join(results_dir, 'pointcloud.txt'))
    m_aligned = np.zeros(np.shape(m))
    for i in range(len(m)):
        m_aligned[i,:] = scale * np.dot(rot, m[i,:]) + trans        
        
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(m_aligned[:,0], m_aligned[:,1], m_aligned[:,2], '.', ms=1, color='green')
    ax.plot(p_es[:,0], p_es[:,1], p_es[:,2], linewidth=3, color='blue', label='SVO Bundle Adjust')
    ax.plot(p_gt[:,0], p_gt[:,1], p_gt[:,2], linewidth=3, color='k', label='Groundtruth')
    #ax.set_xlim([-60, 60])
    #ax.set_ylim([-6, 55])
    #ax.set_zlim([-6,15])
    ax.legend()
    #plt.show()
    #fig.tight_layout()
    fig.savefig(results_dir+'/trajectory_3d'+FORMAT)
    
def plot_trajectory(results_dir, p_gt, p_es, align_first_idx = 0, align_last_idx = -1):
    
    # plot trajectory top
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, aspect='equal', xlabel='x [m]', ylabel='y [m]')
    ax.grid(ls='--', color='0.7')
    p_es_0 = p_es - p_gt[0,:]
    p_gt_0 = p_gt - p_gt[0,:]
    ax.plot(p_es_0[:,0], p_es_0[:,1], 'b-', label='Estimate')
    ax.plot(p_gt_0[:,0], p_gt_0[:,1], 'r-', label='Groundtruth')
    #if align_last_idx < len(p_gt):
    #    ax.plot(p_es_0[align_first_idx:align_last_idx,0], p_es_0[align_first_idx:align_last_idx,1], 'g-', linewidth=2, label='aligned')
    for (x1,y1,z1),(x2,y2,z2) in zip(p_es_0[align_first_idx:align_last_idx:10,:],p_gt_0[align_first_idx:align_last_idx:10,:]):
        ax.plot([x1,x2],[y1,y2],'-',color="gray")
    
    ax.legend(bbox_to_anchor=(0., 1.02, 1., .102),
              loc=3, ncol=3, mode='expand', borderaxespad=0.)
    #ax.set_ylim([-0.5, 5])
    #fig.tight_layout()
    fig.savefig(results_dir+'/trajectory_top'+FORMAT)
    
    # plot trajectory side
    fig = plt.figure(figsize=(8, 5))
    ax = fig.add_subplot(111, aspect='equal', xlabel='x [m]', ylabel='z [m]')
    ax.grid(ls='--', color='0.7')
    ax.plot(p_es[:,0]-p_gt[0,0], p_es[:,2]-p_gt[0,1], 'b-', label='Estimate')
    ax.plot(p_gt[:,0]-p_gt[0,0], p_gt[:,2]-p_gt[0,1], 'r-', label='Groundtruth')
    ax.legend(bbox_to_anchor=(0., 1.02, 1., .102),
              loc=3, ncol=3, mode='expand', borderaxespad=0.)
    #fig.tight_layout()
    fig.savefig(os.path.join(results_dir, 'trajectory_side'+FORMAT))
    
    # write aligned trajectory to file
    file_out = open(os.path.join(results_dir, 'trajectory.csv'), 'w')
    file_out.write('# estimate-x [m], estimate-y [m], estimate-z [m], groundtruth-x [m], groundtruth-y [m], groundtruth-z [m]\n')
    for i in range(len(p_es)):
        file_out.write(
            '%.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n' %
            (p_es[i,0]-p_gt[0,0], p_es[i,1]-p_gt[0,1], p_es[i,2]-p_gt[0,2],
             p_gt[i,0]-p_gt[0,0], p_gt[i,1]-p_gt[0,1], p_gt[i,2]-p_gt[0,2]))
    file_out.close()
    
def plot_translation_error(translation, translation_error, results_dir):
    
    # plot
    fig = plt.figure(figsize=(8, 2.5))
    ax = fig.add_subplot(111, xlabel='Distance [m]', ylabel='Position Drift [mm]', xlim=[0,translation[-1]])
    ax.plot(translation, translation_error[:,0]*1000, 'r-', label='x')
    ax.plot(translation, translation_error[:,1]*1000, 'g-', label='y')
    ax.plot(translation, translation_error[:,2]*1000, 'b-', label='z')
    ax.legend()
    #fig.tight_layout()
    fig.savefig(results_dir+'/translation_error'+FORMAT)
    
    # write to file
    file_out = open(os.path.join(results_dir, 'translation_error.csv'), 'w')
    file_out.write('# distance from start [m], error x [m], error y [m], error z [m]\n')
    for i in range(len(translation_error)):
        file_out.write(
            '%.8f, %.8f, %.8f, %.8f\n' %
            (translation[i], translation_error[i,0], translation_error[i,1], translation_error[i,2])) 
    file_out.close()

def plot_rotation_error(translation, rotation_error, results_dir):
    
    # plot
    fig = plt.figure(figsize=(8, 2.5))
    ax = fig.add_subplot(111, xlabel='Distance [m]', ylabel='Orient. err. [deg]', xlim=[0,translation[-1]])
    ax.plot(translation, rotation_error[:,0]*180.0/np.pi, 'r-', label='yaw')
    ax.plot(translation, rotation_error[:,1]*180.0/np.pi, 'g-', label='pitch')
    ax.plot(translation, rotation_error[:,2]*180.0/np.pi, 'b-', label='roll')
    ax.legend()
    #fig.tight_layout()
    fig.savefig(results_dir+'/orientation_error'+FORMAT)
    
    # write to file
    file_out = open(os.path.join(results_dir, 'orientation_error.csv'), 'w')
    file_out.write('# distance from start [m], error yaw [rad], error pitch [rad], error roll [rad]\n')
    for i in range(len(rotation_error)):
        file_out.write(
            '%.8f, %.8f, %.8f, %.8f\n' %
            (translation[i], rotation_error[i,0], rotation_error[i,1], rotation_error[i,2])) 
    file_out.close()

def plot_scale_error(translation, scale_error_perc, results_dir):
    
    # plot
    fig = plt.figure(figsize=(8,2.5))
    ax = fig.add_subplot(111, xlabel='Distance [m]', ylabel='Scale Drift [\%]', xlim=[0,translation[-1]])
    ax.plot(translation, scale_error_perc, 'b-')
    ax.set_ylim([-100, 100])
    #fig.tight_layout()
    fig.savefig(results_dir+'/scale_drift'+FORMAT)
    
    # write to file
    file_out = open(os.path.join(results_dir, 'scale_error.csv'), 'w')
    file_out.write('# distance from start [m], scale drift [%]\n')
    for i in range(len(scale_error_perc)):
        file_out.write('%.8f, %.8f\n' % (translation[i], scale_error_perc[i])) 
    file_out.close()
    
def plot_travelled_distance(distances, results_dir):
    fig = plt.figure(figsize=(8,5))
    ax = fig.add_subplot(111, xlabel='Measurement', ylabel='Distance [m]')
    ax.plot(range(len(distances)), distances)
    #fig.tight_layout()
    fig.savefig(results_dir+'/distance'+FORMAT)
    
def plot_imu_biases(stamps, bias_gyr, bias_acc, results_dir):
    
    stamps = np.array(stamps) - stamps[0]
    
    # Find min-max range of accelerometer bias
    acc_min = 1.1*np.min(bias_acc)
    acc_max = 1.1*np.max(bias_acc)

    # Plot Accelerometer Bias.
    fig = plt.figure(figsize=(8,12))
    gs1 = gridspec.GridSpec(3, 1)
    gs1.update(wspace=0.015) # set the spacing between axes. 
    ax0 = fig.add_subplot(611, ylabel='Acc. Bias x')
    ax0.set_xticks([])
    ax0.yaxis.set_major_formatter(FuncFormatter(lambda y, pos: '%.3f'%y))
    #ax0.locator_params(axis = 'y', nbins = 5)
    ax0.plot(stamps/1e9, bias_acc[:,0], color='blue')
    ax1 = fig.add_subplot(612, ylabel='Acc. Bias y')
    ax1.set_xticks([])
    ax1.yaxis.set_major_formatter(FuncFormatter(lambda y, pos: '%.3f'%y))
    #ax1.locator_params(axis = 'y', nbins = 5)
    ax1.plot(stamps/1e9, bias_acc[:,1], color='blue')
    ax2 = fig.add_subplot(613, ylabel='Acc. Bias z')
    #ax2.locator_params(axis = 'y', nbins = 5)
    ax2.set_xticks([])
    ax2.yaxis.set_major_formatter(FuncFormatter(lambda y, pos: '%.3f'%y))
    ax2.plot(stamps/1e9, bias_acc[:,2], color='blue')

    ax0.set_ylim([acc_min, acc_max])
    ax1.set_ylim([acc_min, acc_max])
    ax2.set_ylim([acc_min, acc_max])
    
    #ax0.legend(ncol=2,  loc='lower left', borderaxespad=0.2)
    
    # Find min-max range of gyroscope bias
    gyro_min = 1.1*np.min(bias_gyr)
    gyro_max = 1.1*np.max(bias_gyr)
    
    # Plot gyroscope bias
    ax3 = fig.add_subplot(614, ylabel='Gyro. Bias x')
    ax3.set_xticks([])
    ax3.yaxis.set_major_formatter(FuncFormatter(lambda y, pos: '%.3f'%y))
    #ax3.locator_params(axis = 'y', nbins = 5)
    ax3.plot(stamps/1e9, bias_gyr[:,0], color='blue')
    ax4 = fig.add_subplot(615, ylabel='Gyro. Bias y')
    ax4.set_xticks([])
    ax4.yaxis.set_major_formatter(FuncFormatter(lambda y, pos: '%.3f'%y))
    #ax4.locator_params(axis = 'y', nbins = 5)
    ax4.plot(stamps/1e9, bias_gyr[:,1], color='blue')
    ax5 = fig.add_subplot(616, ylabel='Gyro. Bias z', xlabel='Time [s]')
    #ax5.locator_params(axis = 'y', nbins = 5)
    ax5.yaxis.set_major_formatter(FuncFormatter(lambda y, pos: '%.3f'%y))
    ax5.plot(stamps/1e9, bias_gyr[:,2], color='blue')
    
    ax3.set_ylim([gyro_min, gyro_max])
    ax4.set_ylim([gyro_min, gyro_max])
    ax5.set_ylim([gyro_min, gyro_max])
    
    #ax3.legend(ncol=2, loc='lower left') 
    
    ax5.tick_params('x',top='off')
    #fig.tight_layout()
    fig.savefig(os.path.join(results_dir,'imu_states'+FORMAT))
    
    return 0
    
def plot_imu_state_along_trajectory(
        data_dir, bias_gyr, bias_acc, velocity, pos, circle_size=0.2):
    
    # plot bias
    fig = plt.figure(figsize=(8, 20))
    ax = fig.add_subplot(311, aspect='equal', xlabel='x [m]', ylabel='y [m]', 
                         title='Gyroscope Bias')
    ax.grid(ls='--', color='0.7')
    ax.plot(pos[:,0], pos[:,1], 'b-', label='Estimate')
    bias_gyr_norm = np.sqrt(np.sum(bias_gyr**2,1))
    out = plot_utils.circles(pos[:,0], pos[:,1], circle_size,
                             c=bias_gyr_norm, ax=ax, edgecolor='none', lw=0)
    cbar = fig.colorbar(out)
    cbar.set_label(r"Gyroscope Bias")

    # plot pos drift
    ax = fig.add_subplot(312, aspect='equal', xlabel='x [m]', ylabel='y [m]', 
                         title='Accelerometer Bias')
    ax.grid(ls='--', color='0.7')
    ax.plot(pos[:,0], pos[:,1], 'b-', label='Estimate')
    bias_acc_norm = np.sqrt(np.sum(bias_acc**2,1))
    out = plot_utils.circles(pos[:,0], pos[:,1], circle_size,
                             c=bias_acc_norm, ax=ax, edgecolor='none', lw=0)
    cbar = fig.colorbar(out)
    cbar.set_label(r"Accelerometer Bias")
 
    # plot yaw drift
    ax = fig.add_subplot(313, aspect='equal', xlabel='x [m]', ylabel='y [m]', 
                         title='Velocity')
    ax.grid(ls='--', color='0.7')
    ax.plot(pos[:,0], pos[:,1], 'b-', label='Estimate')
    velocity_norm = np.sqrt(np.sum(velocity**2,1))
    out = plot_utils.circles(pos[:,0], pos[:,1], circle_size,
                             c=velocity_norm, ax=ax, edgecolor='none', lw=0)
    cbar = fig.colorbar(out)
    cbar.set_label(r"Velocity")
    
    # Save fig
    fig.savefig(os.path.join(data_dir,'imu_states_along_trajectory'+FORMAT))
    
    
# -----------------------------------------------------------------------------
if __name__=='__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='Simple plotting of trajectory')
    
    parser.add_argument('--data_dir', help='folder with the results',
                        default='')
                        
    options = parser.parse_args()    
    
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger(__name__)
    logger.info('Plot trajectory')
    
    pos = np.genfromtxt(
        os.path.join(options.data_dir, "traj_es.csv"), delimiter=',',
        dtype=np.float64, skip_header=1)[:,1:4]
    length = utils.get_distance_from_start(pos)[-1]
    
    fig = plt.figure(figsize=(5, 8))
    ax = fig.add_subplot(211, aspect='equal', title="Trajectory length = {:.1f}m".format(length),
                         xlabel="x [m]", ylabel="y [m]")    
    ax.plot(pos[:,0], pos[:,1], 'b-', lw=2)
    ax = fig.add_subplot(212, aspect='equal', xlabel="x [m]", ylabel="z [m]")    
    ax.plot(pos[:,0], pos[:,2], 'b-', lw=2)
    fig.patch.set_visible(False)
    fig.tight_layout()
    fig.savefig(os.path.join(options.data_dir, "traj_es"+FORMAT))
