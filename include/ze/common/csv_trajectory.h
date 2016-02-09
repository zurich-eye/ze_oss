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
  virtual void load(const std::string& in_file_path) = 0;

protected:
  CSVTrajectory() { }
  void readHeader(const std::string& in_file_path)
  {
    in_str_.open(in_file_path);
    CHECK(in_str_.is_open());
    std::string line;
    getline(in_str_, line);
    CHECK_EQ(line, header_);
  }

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
    pose << readTranslation(items), readOrientation(items);
    return pose;
  }
  std::ifstream in_str_;
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

  virtual void load(const std::string& in_file_path) override
  {
    readHeader(in_file_path);
    std::string line;
    while(getline(in_str_, line))
    {
      if('%' != line.at(0) && '#' != line.at(0))
      {
        std::stringstream line_str(line);
        std::vector<std::string> items = ze::splitString(line, delimiter_);
        CHECK_GE(items.size(), 4u);
        int64_t stamp = std::stoll(items[order_.find("ts")->second]);
        Vector3 lla = readTranslation(items);
        lla_buf_.insert(stamp, lla);
      }
    }
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
  virtual void load(const std::string& in_file_path) override
  {
    readHeader(in_file_path);
    std::string line;
    while(getline(in_str_, line))
    {
      if('%' != line.at(0) && '#' != line.at(0))
      {
        std::stringstream line_str(line);
        std::vector<std::string> items = ze::splitString(line, delimiter_);
        CHECK_GE(items.size(), 8u);
        int64_t stamp = std::stoll(items[order_.find("ts")->second]);
        Vector7 pose = readPose(items);
        pose_buf_.insert(stamp, pose);
      }
    }
  }

protected:
  Buffer<FloatType, 7> pose_buf_;
};

} // ze namespace
