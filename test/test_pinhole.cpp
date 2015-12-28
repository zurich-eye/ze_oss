#include <string>
#include <vector>
#include <iostream>

#include <ze/common/test/entrypoint.h>
#include <ze/cameras/camera_impl.h>


TEST(CameraPinholeTest, testConstructor)
{
  ze::PinholeCamera<double> cam(752, 480, 310, 320, 376.0, 240.0);
  cam.setLabel("test");
  cam.print(std::cout);
}

TEST(CameraPinholeTest, testProjectionJacobian)
{

}


ZE_UNITTEST_ENTRYPOINT
