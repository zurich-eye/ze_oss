#include <cmath>

#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>

TEST(TestUtilsTest, testTestData)
{
  EXPECT_NO_FATAL_FAILURE(ze::getTestDataDir("synthetic_room_pinhole"));
}

TEST(TestUtilsTest, testLoadCsvPoses)
{
  using namespace ze;

  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  std::string filename = data_path + "/traj_gt.csv";
  std::map<size_t, Transformation> poses = loadIndexedPosesFromCsv(filename);
  EXPECT_EQ(poses.size(), 50);
  EXPECT_DOUBLE_EQ(poses[1].getPosition().x(), 1.499260);
}

ZE_UNITTEST_ENTRYPOINT
