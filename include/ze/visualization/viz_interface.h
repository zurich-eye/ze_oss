// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <string>
#include <tuple>

#include <ze/common/macros.h>
#include <ze/common/types.h>
#include <ze/common/transformation.h>
#include <ze/visualization/viz_common.h>

namespace ze {

using LineMarkers = std::vector<std::pair<Position, Position>, Eigen::aligned_allocator<std::pair<Position, Position>>>;

//! Interface class for all visualizations.
class Visualizer
{
public:
  ZE_POINTER_TYPEDEFS(Visualizer);

  Visualizer() = default;
  virtual ~Visualizer() = default;

  // ---------------------------------------------------------------------------
  // Draw single elements

  virtual void drawPoint(
      const std::string& topic,
      const size_t id,
      const Position& point,
      const Color& color,
      const real_t size = 0.02) = 0;

  virtual void drawLine(
      const std::string& topic,
      const size_t id,
      const Position& line_from,
      const Position& line_to,
      const Color& color,
      const real_t size = 0.02) = 0;

  virtual void drawCoordinateFrame(
      const std::string& topic,
      const size_t id,
      const Transformation& pose, // T_W_B
      const real_t size = 0.02) = 0;

  virtual void drawRobot(
      const std::string& name,
      const Transformation& T_W_B) = 0;

  // ---------------------------------------------------------------------------
  // Draw multiple elements

  virtual void drawPoints(
      const std::string& topic,
      const size_t id,
      const Positions& points,
      const Color& color,
      const real_t size = 0.02) = 0;

  virtual void drawLines(
      const std::string& topic,
      const size_t id,
      const LineMarkers& lines,
      const Color& color,
      const real_t size = 0.02) = 0;

  virtual void drawCoordinateFrames(
      const std::string& topic,
      const size_t id,
      const TransformationVector& poses,
      const real_t size = 0.02) = 0;

  virtual void drawTrajectory(
      const std::string& topic,
      const size_t id,
      const std::vector<Position>& points,
      const Color& color,
      const real_t size = 0.02) = 0;

};

} // namespace ze
