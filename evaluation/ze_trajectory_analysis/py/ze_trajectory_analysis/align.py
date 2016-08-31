#!/usr/bin/python
"""
Zurich Eye
"""

import numpy as np

def align_sim3(p_gt, p_es):
    """Implementation of the paper: S. Umeyama, Least-Squares Estimation 
    of Transformation Parameters Between Two Point Patterns,
    IEEE Trans. Pattern Anal. Mach. Intell., vol. 13, no. 4, 1991.

    Input:
    p_gt -- first trajectory (nx3), numpy array type
    p_es -- second trajectory (nx3), numpy array type
    
    Output:
    s -- scale factor (scalar)
    R_gt_es -- rotation matrix (3x3)
    gt_t_gt_es -- translation vector (3x1)

    p_es_aligned = s * np.transpose(np.dot(R_gt_es, np.transpose(p_es))) + gt_t_gt_es
    """

    # substract mean
    mu_M = p_gt.mean(0)
    mu_D = p_es.mean(0)
    gt_zerocentered = p_gt - mu_M
    es_zerocentered = p_es - mu_D
    n = np.shape(p_gt)[0]

    # correlation
    C = 1.0/n*np.dot(gt_zerocentered.transpose(), es_zerocentered)
    sigma2 = 1.0/n*np.multiply(es_zerocentered, es_zerocentered).sum()
    U_svd,D_svd,V_svd = np.linalg.linalg.svd(C)
    D_svd = np.diag(D_svd)
    V_svd = np.transpose(V_svd)
    S = np.eye(3)

    if(np.linalg.det(U_svd)*np.linalg.det(V_svd) < 0):
        S[2,2] = -1

    R_gt_es = np.dot(U_svd, np.dot(S, np.transpose(V_svd)))
    s = 1.0/sigma2 * np.trace(np.dot(D_svd, S))
    gt_t_gt_es = mu_M - s * np.dot(R_gt_es, mu_D)
    
   
    return s, R_gt_es, gt_t_gt_es

def align_se3(p_gt, p_es):
    """Align two trajectories using the method of Horn (closed-form). 
        
    Input:
    model -- first trajectory (nx3), numpy array type
    data -- second trajectory (nx3), numpy array type
    
    Output:
    R_gt_es -- rotation matrix (3x3)
    gt_t_gt_es -- translation vector (3x1)
    """
    np.set_printoptions(precision=3,suppress=True)
    mu_M = p_gt.mean(0)
    mu_D = p_es.mean(0)
    gt_zerocentered = p_gt - mu_M
    es_zerocentered = p_es - mu_D
    
    W = np.zeros( (3,3) )
    for row in range(p_gt.shape[0]):
        W += np.outer(gt_zerocentered[row,:], es_zerocentered[row,:])
    U,d,Vh = np.linalg.linalg.svd(W.transpose())
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vh) < 0:
        S[2,2] = -1
    R = np.dot(U, np.dot(S, Vh))
    t = mu_D - np.dot(R, mu_M)
    
    R_gt_es = np.transpose(R)
    gt_t_gt_es = - np.dot(R_gt_es, t)

    return R_gt_es, gt_t_gt_es

def _matrix_log(A):
    theta = np.arccos((np.trace(A)-1.0)/2.0)
    log_theta = 0.5*theta/np.sin(theta) * (A - A.transpose())
    x = np.array([log_theta[2,1], log_theta[0,2], log_theta[1,0]])
    return x

def hand_eye_calib(q_gt, q_es, I=[], p_gt=[], p_es=[], delta=1, verbose=True):
    """Implementation of the least squares solution described in the paper:
    Robot Sensor Calibration: Solving AX=XB on the Euclidean Group
    by Frank C. Park and Bryan J. Martin
    """
    estimate_translation_offset = (len(p_es) != 0)
    if len(I) == 0:
        I = range(0,len(q_gt))
    n = len(I)
    M = np.zeros([3,3])
    C = np.zeros([3*n, 3])
    b_A = np.zeros([3*n,1])
    b_B = np.zeros([3*n,1])
    for ix, i in enumerate(I[0:-delta]):
      A1 = ru.quat2dcm(q_es[i,:])
      A2 = ru.quat2dcm(q_es[i+delta,:])
      A  = np.dot(A1.transpose(), A2)
      B1 = ru.quat2dcm(q_gt[i,:])
      B2 = ru.quat2dcm(q_gt[i+delta,:])
      B  = np.dot(B1.transpose(), B2)
      alpha = _matrix_log(A)
      beta = _matrix_log(B)
      M = M + np.dot(np.matrix(beta).transpose(), np.matrix(alpha))
      if estimate_translation_offset:
          C[3*ix:3*ix+3,:] = np.eye(3) - A
          b_A[3*ix:3*ix+3,0] = np.dot(np.transpose(A1), p_es[i+delta,:]-p_es[i,:])
          b_B[3*ix:3*ix+3,0] = np.dot(np.transpose(B1), p_gt[i+delta,:]-p_gt[i,:])

    # compute rotation  
    D,V = np.linalg.linalg.eig(np.dot(M.transpose(), M))
    Lambda = np.diag([np.sqrt(1.0/D[0]), np.sqrt(1.0/D[1]), np.sqrt(1.0/D[2])])
    Vinv = np.linalg.linalg.inv(V)
    X = np.dot(V, np.dot(Lambda, np.dot(Vinv, M.transpose())))

    if estimate_translation_offset:
        # compute translation
        d = np.zeros([3*n,1])
        for i in range(n):
            d[3*i:3*i+3,:] = b_A[3*i:3*i+3,:] - np.dot(X, b_B[3*i:3*i+3,:])
    
        b = np.dot(np.linalg.inv(np.dot(np.transpose(C),C)),  np.dot(np.transpose(C),d))

        return np.array(X), b
    else:
        return np.array(X)