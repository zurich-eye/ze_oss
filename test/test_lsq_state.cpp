#include <random>
#include <ze/common/test_entrypoint.h>
#include <ze/common/matrix.h>
#include <ze/common/manifold.h>
#include <ze/common/types.h>
#include <ze/common/transformation.h>
#include <ze/geometry/lsq_state.h>

TEST(StateTests, testRetract)
{
  using namespace ze;

  using MyState = State<Transformation,Vector3,FloatType>;
  EXPECT_EQ(MyState::dimension, 10);

  MyState state;
  state.print();

  MyState::TangentVector v;
  state.retract(v);

  EXPECT_EQ(State<Transformation>::dimension, 6);

  return 0;
}

ZE_UNITTEST_ENTRYPOINT
