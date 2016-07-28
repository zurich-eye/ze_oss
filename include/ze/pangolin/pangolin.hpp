#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <ze/common/types.h>
#include <ze/common/logging.hpp>
#include <pangolin/pangolin.h>

namespace ze {

//! A class to continuously plot a series of measurements into a window.
class PangolinPlotter
{
public:
  ~PangolinPlotter();

  //! A threaded visualization loop.
  void loop();

  template<typename Scalar>
  void log(const std::string& identifier, Scalar value)
  {
    getLoggerOrCreate(identifier)->Log(value);
  }

  // Specializations for eigen vector types.
  void log(const std::string& identifier, VectorX value)
  {
    std::vector<float> values;
    values.resize(value.size());
    for (int i = 0; i < value.size(); ++i)
    {
      values[i] = value[i];
    }
    getLoggerOrCreate(identifier)->Log(values);
  }

  //! Singleton accessor.
  static PangolinPlotter* instance(
      const std::string& window_title = "",
      int width = 640,
      int height = 480)
  {
    if (!instance_)
    {
      instance_ = new PangolinPlotter(
                    window_title,
                    width,
                    height);
    }
    return instance_;
  }
private:
  //! The window title, also used as window context.
  const std::string window_title_;
  const int width_;
  const int height_;

  //! Temporary information to notify the visualizing loop thread that
  //! a new logger/plotter should be created.
  std::string new_logger_identifier_;
  bool add_logger_ = false;

  //! Maps from identifiers to plotters and data-logs.
  std::map<std::string, std::shared_ptr<pangolin::Plotter>> plotters_;
  std::map<std::string, std::shared_ptr<pangolin::DataLog>> data_logs_;

  std::unique_ptr<std::thread> thread_;
  std::mutex stop_mutex_;
  bool stop_requested_ = false;

  void requestStop();
  bool isStopRequested();
  const uint thread_sleep_ms_ = 40;

  //! The singleton instance of a pangolin plotter.
  static PangolinPlotter* instance_;

  //! Creates a new pangolin window
  PangolinPlotter(const std::string& window_title = "",
                  int width = 640,
                  int height = 480);

  //! Geta logger for a given identifier or create.
  std::shared_ptr<pangolin::DataLog>& getLoggerOrCreate(
      const std::string& new_logger_identifier_);
};

} // namespace ze
