#include <ze/common/csv_trajectory.h>

namespace ze {

CSVTrajectory::CSVTrajectory()
{ }

void CSVTrajectory::readHeader(const std::string& in_file_path)
{
  in_str_.open(in_file_path);
  CHECK(in_str_.is_open());
  std::string line;
  getline(in_str_, line);
  CHECK_EQ(line, header_);
}

Vector3 CSVTrajectory::readTranslation(const std::vector<std::string>& items)
{
  return Vector3(std::stod(items[order_.find("tx")->second]),
      std::stod(items[order_.find("ty")->second]),
      std::stod(items[order_.find("tz")->second]));
}

Vector4 CSVTrajectory::readOrientation(const std::vector<std::string>& items)
{
  return Vector4(std::stod(items[order_.find("qx")->second]),
      std::stod(items[order_.find("qy")->second]),
      std::stod(items[order_.find("qz")->second]),
      std::stod(items[order_.find("qw")->second]));
}

Vector7 CSVTrajectory::readPose(const std::vector<std::string>& items)
{
  Vector7 pose;
  pose << readTranslation(items), readOrientation(items);
  return pose;
}

LLASeries::LLASeries()
{
  order_["ts"] = 0;
  order_["tx"] = 1;
  order_["ty"] = 2;
  order_["tz"] = 3;

  header_ = "timestamp, latitude, longitude, altitude";
}

void LLASeries::load(const std::string& in_file_path)
{
  readHeader(in_file_path);
  std::string line;
  while(getline(in_str_, line))
  {
    if('%' != line.at(0) && '#' != line.at(0))
    {
      std::vector<std::string> items = ze::splitString(line, delimiter_);
      CHECK_GE(items.size(), 4u);
      int64_t stamp = std::stoll(items[order_.find("ts")->second]);
      Vector3 lla = readTranslation(items);
      lla_buf_.insert(stamp, lla);
    }
  }
}

const Buffer<FloatType, 3>& LLASeries::getBuffer() const
{
  return lla_buf_;
}

Buffer<FloatType, 3>& LLASeries::getBuffer()
{
  return lla_buf_;
}

PositionSeries::PositionSeries()
{
  order_["ts"] = 0;
  order_["tx"] = 1;
  order_["ty"] = 2;
  order_["tz"] = 3;

  header_ = "timestamp, tx, ty, tz";
}

void PositionSeries::load(const std::string& in_file_path)
{
  readHeader(in_file_path);
  std::string line;
  while(getline(in_str_, line))
  {
    if('%' != line.at(0) && '#' != line.at(0))
    {
      std::vector<std::string> items = ze::splitString(line, delimiter_);
      CHECK_GE(items.size(), 4u);
      int64_t stamp = std::stoll(items[order_.find("ts")->second]);
      Vector3 position = readTranslation(items);
      position_buf_.insert(stamp, position);
    }
  }
}

const Buffer<FloatType, 3>& PositionSeries::getBuffer() const
{
  return position_buf_;
}

Buffer<FloatType, 3>& PositionSeries::getBuffer()
{
  return position_buf_;
}

PoseSeries::PoseSeries()
{
  order_["ts"] = 0;
  order_["tx"] = 1;
  order_["ty"] = 2;
  order_["tz"] = 3;
  order_["qx"] = 4;
  order_["qy"] = 5;
  order_["qz"] = 6;
  order_["qw"] = 7;

  header_ = "# timestamp, x, y, z, qx, qy, qz, qw";
}

void PoseSeries::load(const std::string& in_file_path)
{
  readHeader(in_file_path);
  std::string line;
  while(getline(in_str_, line))
  {
    if('%' != line.at(0) && '#' != line.at(0))
    {
      std::vector<std::string> items = ze::splitString(line, delimiter_);
      CHECK_GE(items.size(), 8u);
      int64_t stamp = std::stoll(items[order_.find("ts")->second]);
      Vector7 pose = readPose(items);
      pose_buf_.insert(stamp, pose);
    }
  }
}

const Buffer<FloatType, 7>& PoseSeries::getBuffer() const
{
  return pose_buf_;
}

Buffer<FloatType, 7>& PoseSeries::getBuffer()
{
  return pose_buf_;
}

SWEResultSeries::SWEResultSeries()
  : PoseSeries()
{
  header_ = "# timestamp, x, y, z, qx, qy, qz, qw, vx, vy, vz, bgx, bgy, bgz, bax, bay, baz";
}

} // ze namespace
