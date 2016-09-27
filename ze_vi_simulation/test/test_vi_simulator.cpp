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

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/timer_statistics.hpp>
#include <ze/vi_simulation/camera_simulator.hpp>
#include <ze/vi_simulation/vi_simulator.hpp>
#include <ze/visualization/viz_ros.hpp>

TEST(CameraSimulator, testSplineScenario)
{
  using namespace ze;

  Visualizer::Ptr viz = std::make_shared<VisualizerRos>();
  ViSimulator::Ptr sim = createViSimulationScenario1();
  sim->setVisualizer(viz);

  ViSensorData data;
  bool success;
  std::tie(data, success) = sim->getMeasurement();
  int64_t last_cam_stamp = data.timestamp;
  TimerStatistics timer;
  for (int i = 0; i < 2000; ++i)
  {
    auto t = timer.timeScope();
    std::tie(data, success) = sim->getMeasurement();
    EXPECT_TRUE(success);
    EXPECT_EQ(data.imu_stamps(0), last_cam_stamp);
    EXPECT_EQ(data.imu_stamps(data.imu_stamps.size()-1), data.timestamp);
    last_cam_stamp = data.timestamp;
    sim->visualize(1.0, 4.0, 0.3);
  }
  VLOG(1) << "Average time per frame = " << timer.mean() << " milliseconds.";
}

ZE_UNITTEST_ENTRYPOINT
