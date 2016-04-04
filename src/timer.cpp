#include <iterator>
#include <ze/common/timer.h>
#include <ze/common/file_utils.h>

namespace ze {

void TimerCollection::saveToFile(const std::string& directory, const std::string& filename)
{
  std::ofstream fs;
  CHECK(isDir(directory));
  openOutputFileStream(joinPath(directory, filename), &fs);
  fs << *this;
}

std::ostream& operator<<(std::ostream& out, const TimerCollection& timers)
{
  for (const std::pair<std::string, TimerStatistics>& it : timers)
  {
    out << it.first << ":\n"
        << it.second.statistics();
  }
  return out;
}

int64_t getCurrentNanosecondTime()
{
  return std::chrono::duration_cast<Timer::ns>(
          Timer::Clock::now()-Timer::TimePoint()).count();
}

//! Get a formated string of the current time, hour, minute and second
std::string getCurrentTimeStr()
{
  std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm* t = std::localtime(&now);
  CHECK_NOTNULL(t);
  std::ostringstream ss;
  ss << t->tm_year-100 << "-"
     << std::setw(2) << std::setfill('0') << t->tm_mon+1 << "-"
     << std::setw(2) << std::setfill('0') << t->tm_mday << "_"
     << std::setw(2) << std::setfill('0') << t->tm_hour << "-"
     << std::setw(2) << std::setfill('0') << t->tm_min << "-"
     << std::setw(2) << std::setfill('0') << t->tm_sec;
  return ss.str();
}

} // namespace ze
