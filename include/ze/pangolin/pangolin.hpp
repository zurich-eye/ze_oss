#pragma once

#include <memory>
#include <atomic>
#include <mutex>
#include <thread>
#include <ze/common/types.h>
#include <ze/common/logging.hpp>
#include <pangolin/pangolin.h>
#include <ze/common/noncopyable.hpp>

namespace ze {

//! A class to continuously plot a series of measurements into a window.
//! This implementation is not intended for production use but debugging only.
//! The primary reason being that it might not be guaranteed a singleton in
//! a shared-library setup. But it is a thread-safe implementation on multi-core
//! systems.
//! See http://preshing.com/20130930/double-checked-locking-is-fixed-in-cpp11/
//! for details on thread-safe and lock-free singletons.
class PangolinPlotter : Noncopyable
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
    PangolinPlotter* tmp = instance_.load(std::memory_order_acquire);
    if (tmp == nullptr)
    {
      std::lock_guard<std::mutex> lock(instance_mutex_);
      tmp = instance_.load(std::memory_order_relaxed);
      if (tmp == nullptr)
      {
        tmp = new PangolinPlotter(
                      window_title,
                      width,
                      height);
        instance_.store(tmp, std::memory_order_release);
      }
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
  std::mutex add_logger_mutex_;

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
  static std::atomic<PangolinPlotter*> instance_;
  static std::mutex instance_mutex_;

  //! Creates a new pangolin window
  PangolinPlotter(const std::string& window_title = "",
                  int width = 640,
                  int height = 480);

  //! Geta logger for a given identifier or create.
  std::shared_ptr<pangolin::DataLog>& getLoggerOrCreate(
      const std::string& new_logger_identifier_);
};

} // namespace ze
