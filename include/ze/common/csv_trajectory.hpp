// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/common/buffer.hpp>
#include <ze/common/file_utils.hpp>
#include <ze/common/macros.hpp>
#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>

namespace ze {

class CSVTrajectory
{
public:
  ZE_POINTER_TYPEDEFS(CSVTrajectory);

  virtual void load(const std::string& in_file_path) = 0;
  virtual int64_t getTimeStamp(const std::string& ts_str) const;

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
  size_t num_tokens_in_line_;
};

class PositionSeries : public CSVTrajectory
{
public:
  ZE_POINTER_TYPEDEFS(PositionSeries);

  PositionSeries();
  virtual void load(const std::string& in_file_path) override;
  const Buffer<real_t, 3>& getBuffer() const;
  Buffer<real_t, 3>& getBuffer();

protected:
  Buffer<real_t, 3> position_buf_;
};

class PoseSeries : public CSVTrajectory
{
public:
  ZE_POINTER_TYPEDEFS(PoseSeries);

  PoseSeries();

  virtual void load(const std::string& in_file_path) override;
  virtual const Buffer<real_t, 7>& getBuffer() const;
  virtual Buffer<real_t, 7>& getBuffer();
  virtual StampedTransformationVector getStampedTransformationVector();

  static Transformation getTransformationFromVec7(const Vector7& data);

protected:
  Buffer<real_t, 7> pose_buf_;
};

class SWEResultSeries : public PoseSeries
{
public:
  SWEResultSeries();
};

class SWEGlobalSeries : public PoseSeries
{
public:
  SWEGlobalSeries();
};

class EurocResultSeries : public PoseSeries
{
public:
  EurocResultSeries();
};

} // ze namespace
