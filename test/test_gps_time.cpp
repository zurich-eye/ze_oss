#include <iostream>
#include <gtest/gtest.h>

#include <ze/common/time.h>
#include <ze/common/gps_time.h>
#include <ze/common/test_entrypoint.h>

TEST(GPSTime, convertGPS2Unix)
{
  //! GT Values from
  //! https://www.andrews.edu/~tzs/timeconv/timeconvert.php
  int gps_week{0};
  double gps_secs{0};
  int64_t unix_nsecs = ze::gps2UtcNSecs(gps_week, gps_secs);
  EXPECT_EQ(315964800000000000, unix_nsecs);

  gps_week = 1884;
  gps_secs = 454731;
  unix_nsecs = ze::gps2UtcNSecs(gps_week, gps_secs);
  EXPECT_EQ(1455862714000000000, unix_nsecs);
}

ZE_UNITTEST_ENTRYPOINT
