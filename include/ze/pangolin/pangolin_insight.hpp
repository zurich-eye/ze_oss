#pragma once

#include <ze/pangolin/pangolin.hpp>
#include <ze/pangolin/type_watches.hpp>

//! Helper functions and macros to wrap (currently only simple) types, watch
//! them for changes and update the pangolin graphs.

namespace ze {
namespace internal {

template<typename Scalar>
class PangolinInsight : public PrimitiveTypeWrapperImpl<Scalar>
{
public:
  PangolinInsight(const std::string& member_name)
    : plotter_(ze::PangolinPlotter(member_name))
  {
    this->setChangeCallback(std::bind(&PangolinInsight::change_callback, this, std::placeholders::_1));
  }

  void change_callback(Scalar value)
  {
    plotter_.log(value);
  }

private:
  ze::PangolinPlotter plotter_;
};

}
}

#define PANGOLIN_WATCH(TYPE, MEMBER_NAME)                         \
  ze::internal::PangolinInsight<TYPE> MEMBER_NAME("MEMBER_NAME")
