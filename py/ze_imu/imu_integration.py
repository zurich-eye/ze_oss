#!/usr/bin/python
"""
Zurich Eye
"""

import numpy as np
import matplotlib.pyplot as plt
import ze_py.plot_utils as plot_utils
import ze_py.transformations as tf
import ze_imu.imu_trajectory_simulation as imu_sim
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc

# tell matplotlib to use latex font
rc('font',**{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)

gravity = 9.81
g = np.array([0, 0, -gravity])
dt = 1.0/300.0
n = 300
t_max = n * dt
    
def renormalize_quaternion_in_state(state):
    state[:4] /= np.linalg.norm(state[:4])
    return state
       
def integrate_expmap(t_W_B0, v_W_B0, R_W_B0, B_a_W_B, B_w_W_B, dt):
    t_W_B = np.zeros((n,3))
    t_W_B[0,:] = t_W_B0
    v_W_B = np.zeros((n,3))
    v_W_B[0,:] = v_W_B0
    R_W_B = np.zeros((n, 9))
    R_W_B[0,:] = R_W_B0
    
    for i in range(1,n):
        R = np.reshape(R_W_B[i-1,:], (3,3))
        v = v_W_B[i-1,:]
        p = t_W_B[i-1,:]
        a = B_a_W_B[i-1,:]
        w = B_w_W_B[i-1,:]
        t_W_B[i,:] = p + v * dt  + g * (dt**2) * 0.5  + np.dot(R, a * (dt**2) * 0.5)
        v_W_B[i,:] = v + g * dt + np.dot(R, a * dt)
        R_W_B[i,:] = np.reshape(np.dot(R, tf.expmap_so3(w * dt)), (9,))
      
    return R_W_B, v_W_B, t_W_B
    
def integrate_quaternion_zero_order_hold(t_W_B0, v_W_B0, R_W_B0, B_a_W_B, B_w_W_B, dt):
    t_W_B = np.zeros((n,3))
    t_W_B[0,:] = t_W_B0
    v_W_B = np.zeros((n,3))
    v_W_B[0,:] = v_W_B0
    R_W_B = np.zeros((n, 9))
    R_W_B[0,:] = R_W_B0
    
    for i in range(1,n):
        # Last state:
        R_W_Bk = np.reshape(R_W_B[i-1,:], (3,3))
        q_Bk_W = tf.quaternion_from_matrix(tf.convert_3x3_to_4x4(np.transpose(R_W_Bk)))
        a = B_a_W_B[i-1,:]
        w = B_w_W_B[i-1,:]
 
        Theta = np.eye(4) + 0.5 * dt * tf.quat_Omega(w) # Eq. 124 Quat. Tutorial
        q_Bkp1_W = np.dot(Theta, q_Bk_W)                # Eq. 111 Quat. Tutorial
        R_W_B[i,:] = np.reshape(np.transpose(tf.matrix_from_quaternion(q_Bkp1_W)[:3,:3]), (9,))
        v_W_B[i,:] = v_W_B[i-1,:] + g * dt + np.dot(R_W_Bk, a) * dt
        t_W_B[i,:] = t_W_B[i-1,:] + v_W_B[i-1,:] * dt
    
    return R_W_B, v_W_B, t_W_B
   
def integrate_quaternion_runge_kutta_4(t_W_B0, v_W_B0, R_W_B0, B_a_W_B, B_w_W_B, dt):
    t_W_B = np.zeros((n,3))
    t_W_B[0,:] = t_W_B0
    v_W_B = np.zeros((n,3))
    v_W_B[0,:] = v_W_B0
    R_W_B = np.zeros((n, 9))
    R_W_B[0,:] = R_W_B0
    
    def state_derivative(imu_acc, imu_gyr, state, dt):
        q_B_W = state[:4]
        v     = state[4:7]
        R_W_B = np.transpose(tf.matrix_from_quaternion(q_B_W)[:3,:3])
        Omega = tf.quat_Omega(imu_gyr)
        
        # Quaternion derivative according to MARS-Lab Quaternion Tutorial.
        q_dot = 0.5 * np.dot(Omega, q_B_W)
        v_dot = np.array([0.0, 0.0, -gravity]) + np.dot(R_W_B, imu_acc)
        p_dot = v # + dt * np.array([0.0, 0.0, -gravity]) + dt * np.dot(R_W_B, imu_acc)
        state_dot = np.concatenate((q_dot, v_dot, p_dot))
        return state_dot
    
    for i in range(1,n):
        # Last state:
        R = np.reshape(R_W_B[i-1,:], (3,3))
        q_B_W = tf.quaternion_from_matrix(tf.convert_3x3_to_4x4(np.transpose(R)))
        v = v_W_B[i-1,:]
        p = t_W_B[i-1,:]
      
        # Get imu measurements from last, current timestamp and interpolate in between.
        imu_acc_1 = B_a_W_B[i-1,:]
        imu_acc_3 = B_a_W_B[i,:]
        imu_acc_2 = 0.5 * (imu_acc_1 + imu_acc_3)
        imu_gyr_1 = B_w_W_B[i-1,:]
        imu_gyr_3 = B_w_W_B[i,:]
        imu_gyr_2 = 0.5 * (imu_gyr_1 + imu_gyr_3)
      
        if True:
            # Runge-Kutta Integration:
            state_1 = np.concatenate((q_B_W, v, p))
            state_1_dot = state_derivative(imu_acc_1, imu_gyr_1, state_1, 0.0)
            
            state_2 = state_1 + 0.5 * dt * state_1_dot
            state_2 = renormalize_quaternion_in_state(state_2)
            state_2_dot = state_derivative(imu_acc_2, imu_gyr_2, state_2, 0.5 * dt)
            
            state_3 = state_1 + 0.5 * dt * state_2_dot
            state_3 = renormalize_quaternion_in_state(state_3)
            state_3_dot = state_derivative(imu_acc_2, imu_gyr_2, state_3, 0.5 * dt)
            
            state_4 = state_1 + 1.0 * dt * state_3_dot
            state_4 = renormalize_quaternion_in_state(state_4)
            state_4_dot = state_derivative(imu_acc_3, imu_gyr_3, state_4, dt)
            
            integrated_state = \
                state_1 + 1.0 / 6.0 * dt * (state_1_dot + 2.0 * state_2_dot + 2.0 * state_3_dot + state_4_dot) 
            integrated_state = renormalize_quaternion_in_state(integrated_state)
        else:
            state_1 = np.concatenate((q_B_W, v, p))
            state_1_dot = state_derivative(imu_acc_1, imu_gyr_1, state_1, 0.0)
            integrated_state = state_1 + dt * state_1_dot
            integrated_state = renormalize_quaternion_in_state(integrated_state)
        
        # Save next state:
        R = tf.matrix_from_quaternion(integrated_state[:4])[:3,:3]
        R_W_B[i,:] = np.reshape(np.transpose(R), (9,))
        v_W_B[i,:] = integrated_state[4:7]
        t_W_B[i,:] = integrated_state[7:10]
    
    return R_W_B, v_W_B, t_W_B
    
# -----------------------------------------------------------------------------
# Generate measurements and ground-truth data.

t_W_B, v_W_B, R_W_B, B_a_W_B, B_w_W_B = imu_sim.get_simulated_imu_data(t_max, dt)

ax = imu_sim.plot_trajectory(t_W_B, R_W_B)

# -------------------------------------------------------------------------
# Verification: Repeat integration to see how big the integration error is

R_expmap, v_expmap, t_expmap = integrate_expmap(
    t_W_B[0,:], v_W_B[0,:], R_W_B[0,:], B_a_W_B, B_w_W_B, dt)
    
R_qrk4, v_qrk4, t_qrk4 = integrate_quaternion_runge_kutta_4(
    t_W_B[0,:], v_W_B[0,:], R_W_B[0,:], B_a_W_B, B_w_W_B, dt)
 
R_q0or, v_q0or, t_q0or = integrate_quaternion_zero_order_hold(
    t_W_B[0,:], v_W_B[0,:], R_W_B[0,:], B_a_W_B, B_w_W_B, dt)
    
ax.plot(t_expmap[:,0], t_expmap[:,1], t_expmap[:,2], '-r', label='re-integrated expmap')
ax.plot(t_qrk4[:,0], t_qrk4[:,1], t_qrk4[:,2], '-g', label='re-integrated runge-kutta')
ax.plot(t_q0or[:,0], t_q0or[:,1], t_q0or[:,2], '-m', label='quaternion zero order')
ax.legend()
plot_utils.axis_equal_3d(ax)