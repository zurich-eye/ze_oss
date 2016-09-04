// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

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
