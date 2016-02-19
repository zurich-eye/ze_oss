#include <ze/common/csv_trajectory.h>

namespace ze {

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
  return Vector3(
        std::stod(items[order_.find("tx")->second]),
        std::stod(items[order_.find("ty")->second]),
        std::stod(items[order_.find("tz")->second]));
}

Vector4 CSVTrajectory::readOrientation(const std::vector<std::string>& items)
{
  Vector4 q(
        std::stod(items[order_.find("qx")->second]),
        std::stod(items[order_.find("qy")->second]),
        std::stod(items[order_.find("qz")->second]),
        std::stod(items[order_.find("qw")->second]));
  if(std::abs(q.squaredNorm() - 1.0) > 1e-4)
  {
    LOG(WARNING) << "Quaternion norm is = " << q.norm();
    CHECK_NEAR(q.norm(), 1.0, 0.01);
    q.normalize(); // This is only good up to some point.
  }
  return q;
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

  header_ = "# timestamp, latitude, longitude, altitude";
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

  header_ = "# timestamp, tx, ty, tz";
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
    if('%' != line.at(0) && '#' != line.at(0) && 't' != line.at(0))
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

StampedTransformationVector PoseSeries::getStampedTransformationVector()
{
  StampedTransformationVector vec;
  pose_buf_.lock();
  auto& data = pose_buf_.data();
  vec.reserve(data.size());
  for(const auto& it : data)
  {
    vec.push_back(std::make_pair(it.first, getTransformationFromVec7(it.second)));
  }
  pose_buf_.unlock();
  return vec;
}

Transformation PoseSeries::getTransformationFromVec7(const Vector7& data)
{
  Vector3 p = data.head<3>();
  Eigen::Quaterniond q(data(6), data(3), data(4), data(5));
  CHECK_NEAR(q.squaredNorm(), 1.0, 1e-4);
  q.normalize();
  return Transformation(q, p);
}

SWEResultSeries::SWEResultSeries()
  : PoseSeries()
{
  header_ = "timestamp, x, y, z, qx, qy, qz, qw, vx, vy, vz, bgx, bgy, bgz, bax, bay, baz";
}

EurocResultSeries::EurocResultSeries()
  : PoseSeries()
{
  order_["ts"] = 0;
  order_["tx"] = 1;
  order_["ty"] = 2;
  order_["tz"] = 3;
  order_["qx"] = 5;
  order_["qy"] = 6;
  order_["qz"] = 7;
  order_["qw"] = 4;

  header_ = "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [], v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1], b_w_RS_S_x [rad s^-1], b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1], b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]";
}

} // ze namespace
