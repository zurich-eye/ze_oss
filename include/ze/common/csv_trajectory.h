#pragma once

#include <ze/common/types.h>
#include <ze/common/buffer.h>
#include <ze/common/file_utils.h>
#include <ze/common/path_utils.h>
#include <ze/common/transformation.h>
#include <ze/common/time.h>

namespace ze {

using StampedTransformationVector =
  std::vector<std::pair<int64_t, Transformation>,
              Eigen::aligned_allocator<std::pair<int64_t, Transformation>>>;

class CSVTrajectory
{
public:
  virtual void load(const std::string& in_file_path) = 0;

protected:
  CSVTrajectory() = default;
  void readHeader(const std::string& in_file_path);
  Vector3 readTranslation(const std::vector<std::string>& items);
  Vector4 readOrientation(const std::vector<std::string>& items);
  Vector7 readPose(const std::vector<std::string>& items);
  std::ifstream in_str_;
  std::map<std::string, int> order_;
  std::string header_;
  const char delimiter_{','};
};

class LLASeries : public CSVTrajectory
{
public:
  LLASeries();
  virtual void load(const std::string& in_file_path) override;
  const Buffer<FloatType, 3>& getBuffer() const;
  Buffer<FloatType, 3>& getBuffer();

protected:
  Buffer<FloatType, 3> lla_buf_;
};

class PositionSeries : public CSVTrajectory
{
public:
  PositionSeries();
  virtual void load(const std::string& in_file_path) override;
  const Buffer<FloatType, 3>& getBuffer() const;
  Buffer<FloatType, 3>& getBuffer();

protected:
  Buffer<FloatType, 3> position_buf_;
};

class PoseSeries : public CSVTrajectory
{
public:
  PoseSeries();

  virtual void load(const std::string& in_file_path) override;
  const Buffer<FloatType, 7>& getBuffer() const;
  Buffer<FloatType, 7>& getBuffer();
  StampedTransformationVector getStampedTransformationVector();
  static Transformation getTransformationFromVec7(const Vector7& data);

protected:
  Buffer<FloatType, 7> pose_buf_;
};

class SWEResultSeries : public PoseSeries
{
public:
  SWEResultSeries();
};

} // ze namespace
