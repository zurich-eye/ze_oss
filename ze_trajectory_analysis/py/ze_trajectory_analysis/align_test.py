#!/usr/bin/env python3
"""
Zurich Eye
"""

import unittest
import numpy as np
import numpy.testing as npt
import ze_py.transformations as tf
import ze_trajectory_analysis.align as align

class TestAlign(unittest.TestCase):
    
    def test_align_se3(self):
        for i in range(100):
            # Random data
            n_points = 100
            T_gt_es = tf.random_transformation()
            T_es_gt = tf.inverse_matrix(T_gt_es)
            p_gt = np.random.random((n_points,3))
            p_es = np.transpose(np.dot(T_es_gt[:3,:3], np.transpose(p_gt))) + T_es_gt[:3,3]
            
            # Compute alignment
            R_gt_es, gt_t_gt_es = align.align_se3(p_gt, p_es)
            T = np.eye(4)
            T[:3,:3] = R_gt_es
            T[:3,3]  = gt_t_gt_es
            npt.assert_array_almost_equal(T_gt_es, T)
            
            # Check alignment
            p_es_aligned = np.transpose(np.dot(R_gt_es, np.transpose(p_es))) + gt_t_gt_es
            npt.assert_array_almost_equal(p_es_aligned, p_gt)
            
    def test_align_sim3(self):
        for i in range(100):
            # Random data
            n_points = 100
            T_gt_es = tf.random_transformation()
            s_inv = np.random.random()
            T_es_gt = tf.inverse_matrix(T_gt_es)
            p_gt = np.random.random((n_points,3))
            p_es = s_inv * np.transpose(np.dot(T_es_gt[:3,:3], np.transpose(p_gt))) + s_inv * T_es_gt[:3,3]
            
            # Compute alignment
            s, R_gt_es, gt_t_gt_es = align.align_sim3(p_gt, p_es)
            T = np.eye(4)
            T[:3,:3] = R_gt_es
            T[:3,3]  = gt_t_gt_es
            npt.assert_almost_equal(s, 1.0 / s_inv)
            npt.assert_array_almost_equal(T_gt_es, T)
            
            # Check alignment
            p_es_aligned = s * np.transpose(np.dot(R_gt_es, np.transpose(p_es))) + gt_t_gt_es
            npt.assert_array_almost_equal(p_es_aligned, p_gt)
   
if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestAlign)
    unittest.TextTestRunner(verbosity=2).run(suite)