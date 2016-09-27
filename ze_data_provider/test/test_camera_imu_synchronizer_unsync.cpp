// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <string>
#include <iostream>

#include <imp/core/image_base.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/image_cv.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/data_provider/data_provider_csv.hpp>
#include <ze/data_provider/data_provider_rosbag.hpp>
#include <ze/data_provider/camera_imu_synchronizer_unsync.hpp>
#include <ze/imu/imu_model.hpp>

namespace ze {
// a dummy data provider
class DataProviderDummy : public DataProviderBase
{
public:
  DataProviderDummy(): DataProviderBase(DataProviderType::Rosbag) {}
  void spin() {}
  virtual bool spinOnce() { return true; }
  virtual bool ok() const { return true; }
  virtual size_t imuCount() const
  {
    return imu_count_;
  };
  virtual size_t cameraCount() const
  {
    return camera_count_;
  }
  size_t imu_count_;
  size_t camera_count_;
};
}

TEST(CameraImuSynchronizerUnsyncTest, testFunctionality)
{
  using namespace ze;
  // fake data provider with two cameras
  DataProviderDummy data_provider;
  data_provider.camera_count_ = 1;
  data_provider.imu_count_ = 1;

  // get an imu model
  std::shared_ptr<ImuIntrinsicModelCalibrated> intrinsics =
      std::make_shared<ImuIntrinsicModelCalibrated>();
  std::shared_ptr<ImuNoiseNone> noise = std::make_shared<ImuNoiseNone>();

  AccelerometerModel::Ptr a_model =
      std::make_shared<AccelerometerModel>(intrinsics, noise);
  GyroscopeModel::Ptr g_model =
      std::make_shared<GyroscopeModel>(intrinsics, noise);

  ImuModel model(a_model, g_model);

  CameraImuSynchronizerUnsync sync(data_provider,
                                  { std::make_shared<ImuModel>(a_model, g_model) });

  size_t measurements = 0u;
  sync.registerCameraImuCallback(
        [&](const StampedImages& images,
            const ImuStampsVector& imu_timestamps,
            const ImuAccGyrVector& imu_measurements)
        {
          ++measurements;
          EXPECT_EQ(1, images.size());
          EXPECT_EQ(1, imu_timestamps.size());
          EXPECT_EQ(1, imu_measurements.size());
        }
  );

  int64_t stamp1 = 1403636579763555580;
  int64_t stamp2 = 1403636579763555590;
  int64_t stamp3 = 1403636579763555600;

  auto img1 = std::make_shared<ImageRaw8uC1>(1, 1);

  sync.addGyroData(stamp1, Vector3(), 0);
  sync.addAccelData(stamp1 + 1, Vector3(), 0);
  EXPECT_EQ(0, measurements);

  sync.addImgData(stamp2, img1, 0);
  EXPECT_EQ(0, measurements);

  sync.addGyroData(stamp3, Vector3(), 0);
  EXPECT_EQ(0, measurements);
  sync.addAccelData(stamp3 + 1, Vector3(), 0);
  EXPECT_EQ(1, measurements);
}

ZE_UNITTEST_ENTRYPOINT
