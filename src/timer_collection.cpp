#include <iterator>
#include <ze/common/timer_collection.h>
#include <ze/common/file_utils.h>

namespace ze {

void TimerCollection::saveToFile(
    const std::string& directory,
    const std::string& filename)
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

} // namespace ze
