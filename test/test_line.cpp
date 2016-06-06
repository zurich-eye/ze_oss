#include <ze/common/line.hpp>
#include <ze/common/test_entrypoint.h>

TEST(LineTest, testLineGeneration)
{
  // Create some random line endpoints.
  const size_t n = 100;
  ze::Positions start, end;
  start.setRandom(3, n);
  end.setRandom(3, n);

  // Create lines.
  std::vector<ze::Line> lines;
  ze::Line::generateLinesFromEndpoints(start, end, lines);

  for (size_t i = 0; i < n; ++i)
  {
    ze::Vector3 direction = lines[i].getDirection();
    ze::Position anchor_point = lines[i].getAnchorPoint();
    // Check whether anchor point is really the closest point available.
    EXPECT_NEAR(direction.dot(anchor_point), 0.0, 1e-10);
    // Direction should be a normalized vector.
    EXPECT_NEAR(direction.norm(), 1.0, 1e-15);
    // Anchor, start and end point should be on the line.
    EXPECT_NEAR(lines[i].calculateDistanceToLine(anchor_point), 0.0, 1e-14);
    EXPECT_NEAR(lines[i].calculateDistanceToLine(start.col(i)), 0.0, 1e-14);
    EXPECT_NEAR(lines[i].calculateDistanceToLine(end.col(i)), 0.0, 1e-14);
  }
}

ZE_UNITTEST_ENTRYPOINT
