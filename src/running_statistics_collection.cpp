#include <ze/common/running_statistics_collection.h>
#include <ze/common/file_utils.h>

namespace ze {

void StatisticsCollection::saveToFile(
    const std::string& directory,
    const std::string& filename)
{
  std::ofstream fs;
  CHECK(isDir(directory));
  openOutputFileStream(joinPath(directory, filename), &fs);
  fs << *this;
}

std::ostream& operator<<(std::ostream& out, const StatisticsCollection& collection)
{
  for (const std::pair<std::string, RunningStatistics>& it : collection)
  {
    out << it.first << ":\n"
        << it.second;
  }
  return out;
}

} // namespace ze
