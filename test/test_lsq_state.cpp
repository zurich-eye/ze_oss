#include <random>
#include <ze/common/test_entrypoint.h>
#include <ze/common/matrix.h>
#include <ze/common/types.h>
#include <ze/common/transformation.h>
#include <ze/geometry/lsq_state.h>

int main(int argc, char** argv)
{
  using namespace ze;
  State<Transformation,Vector3> state;
  state.print();


  return 0;
}

//ZE_UNITTEST_ENTRYPOINT
