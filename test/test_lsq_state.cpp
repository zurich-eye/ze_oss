#include <random>
#include <ze/common/test_entrypoint.h>
#include <ze/common/matrix.h>
#include <ze/common/manifold.h>
#include <ze/common/types.h>
#include <ze/common/transformation.h>
#include <ze/geometry/lsq_state.h>

TEST(StateTests, testTupleFixedSize)
{
  using namespace ze;
  using Tuple1 = std::tuple<Transformation, FloatType, Vector3>;
  using Tuple2 = std::tuple<Transformation, VectorX>;
  EXPECT_TRUE(internal::TupleIsFixedSize<Tuple1>::is_fixed_size);
  EXPECT_FALSE(internal::TupleIsFixedSize<Tuple2>::is_fixed_size);
}

TEST(StateTests, testStateFixedSize)
{
  using namespace ze;

  using MyState = State<Transformation,Vector3,FloatType>;
  EXPECT_EQ(MyState::dimension, 10);

  MyState state;
  state.print();

  MyState::TangentVector v;
  state.retract(v);

  EXPECT_EQ(State<Transformation>::dimension, 6);
}

TEST(StateTests, testStateDynamicSize)
{
  using namespace ze;
  using MyState = State<Transformation,VectorX>;

  // Test constructor of dynamic-sized state.
  MyState state;
  VectorX& x = state.at<1>();
  x.resize(5);
  x.setConstant(0.5);
  state.print();

  EXPECT_EQ(state.getDimension(), 11);
  EXPECT_TRUE(state.isDynamicSize());
  EXPECT_FALSE(state.isElementDynamicSize<0>());
  EXPECT_TRUE(state.isElementDynamicSize<1>());

  // Test retract.
  traits<MyState>::TangentVector v;
  v.resize(state.getDimension());
  v.setConstant(1.0);
  state.retract(v);
  state.print();
}

ZE_UNITTEST_ENTRYPOINT
