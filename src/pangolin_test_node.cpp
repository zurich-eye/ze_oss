#include <ze/pangolin/pangolin.hpp>
#include <ze/pangolin/type_watches.hpp>
#include <ze/pangolin/pangolin_insight.hpp>

int main( int /*argc*/, char* argv[] )
{
//  ze::internal::PangolinInsight<double> test("test");
  PANGOLIN_WATCH(double, test);

  while(true)
  {
    test += 0.3;
    sleep(1u);
  }

  return 0;
}
