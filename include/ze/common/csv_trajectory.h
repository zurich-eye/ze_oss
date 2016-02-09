#pragma once

#include <ze/common/types.h>
#include <ze/common/buffer.h>
#include <ze/common/file_utils.h>
#include <ze/common/path_utils.h>
#include <ze/common/transformation.h>
#include <ze/common/time.h>

namespace ze {

class CSVTrajectory
{
public:
  virtual void load() = 0;

  Vector3 readTranslation(const std::vector<std::string>& items)
  {
    return Vector3(std::stod(items[order_.find("tx")->second]),
        std::stod(items[order_.find("ty")->second]),
        std::stod(items[order_.find("tz")->second]));
  }

  Vector4 readOrientation(const std::vector<std::string>& items)
  {
    return Vector4(std::stod(items[order_.find("qx")->second]),
        std::stod(items[order_.find("qy")->second]),
        std::stod(items[order_.find("qz")->second]),
        std::stod(items[order_.find("qw")->second]));
  }

  Vector7 readPose(const std::vector<std::string>& items)
  {
    Vector7 pose;
    pose << readTranslation(items) << readOrientation(items);
    return pose;
  }


protected:
  CSVTrajectory() { }
  std::map<std::string, int> order_;
  std::string header_;
  const char delimiter_{','};
};

class LLASeries : public CSVTrajectory
{
public:
  LLASeries()
  {
    order_["ts"] = 0;
    order_["tx"] = 1;
    order_["ty"] = 2;
    order_["tz"] = 3;

    header_ = "timestamp, latitude, longitude, altitude";
  }

  virtual void load() override
  {

  }

protected:
  Buffer<FloatType, 3> lla_buf_;
};

class PoseSeries : public CSVTrajectory
{
public:
  PoseSeries()
  {
    order_["ts"] = 0;
    order_["tx"] = 1;
    order_["ty"] = 2;
    order_["tz"] = 3;
    order_["qx"] = 4;
    order_["qy"] = 5;
    order_["qz"] = 6;
    order_["qw"] = 7;

    header_ = "timestamp, x, y, z, qx, qy, qz, qw";
  }
  virtual void load() override
  {

  }

protected:
  Buffer<FloatType, 7> pose_buf_;
};

} // ze namespace
