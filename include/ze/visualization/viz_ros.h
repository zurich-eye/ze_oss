#pragma once

#include <memory>
#include <ze/visualization/viz_interface.h>

// fwd
namespace ros {
class Publisher;
class NodeHandle;
}

namespace ze {

//! @todo(cfo): Deal with multi-threaded publishing.
class VisualizerRos : public Visualizer
{
public:

  VisualizerRos();
  VisualizerRos(const std::string& frame);

  virtual ~VisualizerRos() = default;

  // ---------------------------------------------------------------------------
  // Draw single elements

  virtual void drawPoint(
      const std::string& ns,
      const size_t id,
      const Position& point,
      const Color& color,
      const FloatType size = 0.02) override;

  virtual void drawLine(
      const std::string& ns,
      const size_t id,
      const Position& line_from,
      const Position& line_to,
      const Color& color,
      const FloatType size = 0.02) override;

  virtual void drawCoordinateFrame(
      const std::string& ns,
      const size_t id,
      const Transformation& pose, // T_W_B
      const FloatType size = 0.2) override;

  // ---------------------------------------------------------------------------
  // Draw multiple elements

  virtual void drawPoints(
      const std::string& ns,
      const size_t id,
      const Positions& points,
      const Color& color,
      const FloatType size = 0.02) override;

  virtual void drawLines(
      const std::string& ns,
      const size_t id,
      const Lines& lines,
      const Color& color,
      const FloatType size = 0.02) override;

  virtual void drawCoordinateFrames(
      const std::string& ns,
      const size_t id,
      const TransformationVector& poses,
      const FloatType size = 0.2) override;

  virtual void drawTrajectory(
      const std::string& topic,
      const size_t id,
      const std::vector<Position>& points,
      const Color& color,
      const FloatType size = 0.02) override;

private:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<ros::Publisher> pub_marker_;
  std::string world_frame = "map";    //!< World-frame
  double viz_scale_ = 1.0;            //!< Scale marker size
};


} // namespace ze
