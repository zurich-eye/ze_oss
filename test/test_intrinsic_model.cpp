#include <ze/common/test_entrypoint.h>

#include <ze/imu/imu_intrinsic_model.h>

TEST(IntrinsicModelTests, testIntrinsicModelCalibrated)
{
  using namespace ze;
  ImuIntrinsicModelCalibrated::Ptr model =
      std::make_shared<ImuIntrinsicModelCalibrated>();

  ASSERT_TRUE(ImuIntrinsicType::Calibrated == model->type());
}

TEST(IntrinsicModelTests, testIntrinsicModelScaleMisalignment)
{
  using namespace ze;
  Vector3 b; b << 1.0, 2.0, 3.0;
  Matrix3 M; M << 1.0, 0.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
  ImuIntrinsicModelScaleMisalignment::Ptr model =
      std::make_shared<ImuIntrinsicModelScaleMisalignment>(0.1, 10, b, M);

  EXPECT_TRUE(ImuIntrinsicType::ScaleMisalignment == model->type());
  EXPECT_DOUBLE_EQ(0.1, model->delay());
  EXPECT_EQ(10, model->range());
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(b, model->b()));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(M, model->M()));
}

TEST(IntrinsicModelTests, testIntrinsicModelScaleMisalignmentGSensitivity)
{
  using namespace ze;
  Vector3 b; b << 1.0, 2.0, 3.0;
  Matrix3 M; M << 1.0, 0.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
  Matrix3 Ma; Ma << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;

  ImuIntrinsicModelScaleMisalignmentGSensitivity::Ptr model =
      std::make_shared<ImuIntrinsicModelScaleMisalignmentGSensitivity>(
        0.1, 10, b, M, Ma);

  ASSERT_TRUE(ImuIntrinsicType::ScaleMisalignmentGSensitivity == model->type());

  EXPECT_DOUBLE_EQ(0.1, model->delay());
  EXPECT_EQ(10, model->range());
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(b, model->b()));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(M, model->M()));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(Ma, model->Ma()));
}

TEST(IntrinsicModelTests, testIntrinsicModelScaleMisalignmentSizeEffect)
{
  using namespace ze;
  Vector3 b; b << 1.0, 2.0, 3.0;
  Matrix3 M; M << 1.0, 0.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
  Matrix3 R; R << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
  ImuIntrinsicModelScaleMisalignmentSizeEffect::Ptr model =
      std::make_shared<ImuIntrinsicModelScaleMisalignmentSizeEffect>(
        0.1, 10, b, M, R);

  ASSERT_TRUE(ImuIntrinsicType::ScaleMisalignmentSizeEffect == model->type());

  EXPECT_DOUBLE_EQ(0.1, model->delay());
  EXPECT_EQ(10, model->range());
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(b, model->b()));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(M, model->M()));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(R, model->R()));
}

ZE_UNITTEST_ENTRYPOINT
