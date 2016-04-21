#include <random>
#include <ze/common/test_entrypoint.h>
#include <ze/common/matrix.h>
#include <ze/common/timer.h>
#include <ze/common/types.h>
#include <ze/common/transformation.h>
#include <ze/cameras/camera_impl.h>
#include <ze/cameras/camera_utils.h>
#include <ze/geometry/triangulation.h>

TEST(TriangulationTests, testSolver)
{
  using namespace ze;

  Transformation T_W_C;
  T_W_C.setRandom(); // Random camera to world transformation.

  PinholeCamera cam = createPinholeCamera(640, 480, 329.11, 329.11, 320.0, 240.0);
  Keypoint px(349, 210);
  Position p_W_true = T_W_C * (cam.backProject(px) * 2.0);

  TransformationVector T_C_W_vec;
  Bearings f_C(3, 10);
  int n = 0;
  std::ranlux24 gen;
  std::normal_distribution<FloatType> noise_rot(0, 0.1);
  std::normal_distribution<FloatType> noise_pos(0, 0.5);
  for(int i = 0; i < 10; ++i)
  {
    // Perturb pose:
    Vector6 pert;
    pert.head<3>() = Vector3::Constant(noise_pos(gen));
    pert.tail<3>() = Vector3::Constant(noise_rot(gen));
    Transformation T_C_W_perturbed = (T_W_C * Transformation::exp(pert)).inverse();

    Position p_C = T_C_W_perturbed * p_W_true;
    Keypoint px = cam.project(p_C);
    if(isVisible(cam.size(), px))
    {
      T_C_W_vec.push_back(T_C_W_perturbed);
      f_C.col(n++) = p_C.normalized();
    }
  }
  f_C.conservativeResize(3, T_C_W_vec.size());
  ASSERT_GE(T_C_W_vec.size(), 2u);

  // Triangulate.
  Vector4 p_W_homogeneous;
  bool success;
  std::tie(p_W_homogeneous, success) = triangulateHomogeneousDLT(T_C_W_vec, f_C);
  Vector3 p_W_estimated = p_W_homogeneous.head<3>() / p_W_homogeneous(3);

  // Compare error.
  EXPECT_LT((p_W_estimated - p_W_true).norm(), 1e-10);
}

ZE_UNITTEST_ENTRYPOINT
