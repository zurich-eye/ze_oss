#include <functional>

#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>
#include <imp/bridge/opencv/cv_bridge.hpp>

TEST(ImpBridgeOpenCvTests, testLoad)
{
  using namespace ze;

  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  ImageCv8uC1::Ptr img = cvBridgeLoad8uC1(joinPath(data_path, "img", "1.png"));
  CHECK(img);
}

ZE_UNITTEST_ENTRYPOINT
