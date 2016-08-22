#include <ze/common/test_entrypoint.h>
#include <ze/common/timer_statistics.h>
#include <ze/vi_simulation/camera_simulator.hpp>
#include <ze/vi_simulation/vi_simulator.hpp>
#include <ze/visualization/viz_ros.h>

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
  for (int i = 0; i < 1000; ++i)
  {
    auto t = timer.timeScope();
    std::tie(data, success) = sim->getMeasurement();
    EXPECT_TRUE(success);
    EXPECT_EQ(data.imu_stamps(0), last_cam_stamp);
    EXPECT_EQ(data.imu_stamps(data.imu_stamps.size()-1), data.timestamp);
    last_cam_stamp = data.timestamp;
    sim->visualize();
  }
  VLOG(1) << "Average time per frame = " << timer.mean() << " milliseconds.";
}

ZE_UNITTEST_ENTRYPOINT
