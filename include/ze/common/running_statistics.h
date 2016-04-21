#pragma once

#include <algorithm>
#include <ze/common/types.h>

namespace ze {

//! Collects samples and incrementally computes statistical properties.
//! http://www.johndcook.com/blog/standard_deviation/
class RunningStatistics
{
public:

  RunningStatistics() = default;
  ~RunningStatistics() = default;

  inline void addSample(FloatType x)
  {
    ++n_;
    min_ = std::min(min_, x);
    max_ = std::max(max_, x);
    sum_ += x;

    // Online variance computation [Knuth TAOCP vol 2, 3rd edition, page 232].
    if (n_ == 1u)
    {
      M_ = x;
      S_ = 0.0;
    }
    else
    {
      FloatType M_new = M_ + (x - M_) / n_;
      S_ += (x - M_) * (x - M_new);
      M_ = M_new;
    }
  }

  inline FloatType numSamples() const { return n_; }
  inline FloatType min()  const { return min_; }
  inline FloatType max()  const { return max_; }
  inline FloatType sum()  const { return sum_; }
  inline FloatType mean() const { return M_; }

  // The use of (n-1) is due to https://en.wikipedia.org/wiki/Bessel's_correction
  // that's why the result for small sample sizes in unit tests may not coincide
  // with what you may expect.
  inline FloatType var()  const { return (n_ > 0u) ? S_ / (n_ - 1u) : 0.0; }
  inline FloatType std()  const { return std::sqrt(var()); }

  inline void reset()
  {
    n_ = 0;
    min_ = std::numeric_limits<FloatType>::max();
    max_ = 0.0;
    sum_ = 0.0;
    M_ = 0.0;
    S_ = 0.0;
  }

private:
  uint32_t n_ = 0u;
  FloatType min_ = std::numeric_limits<FloatType>::max();
  FloatType max_ = 0.0;
  FloatType sum_ = 0.0;
  FloatType M_ = 0.0;
  FloatType S_ = 0.0;
};

//! Print statistics:
inline std::ostream& operator<<(std::ostream& out, const RunningStatistics& stat)
{
  out << "  num_samples: " << stat.numSamples() << "\n"
      << "  min: " << stat.min() << "\n"
      << "  max: " << stat.max() << "\n"
      << "  sum: " << stat.sum() << "\n"
      << "  mean: " << stat.mean() << "\n"
      << "  variance: " << stat.var() << "\n"
      << "  standard_deviation: " << stat.std() << "\n";
  return out;
}

} // end namespace ze
