// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/pangolin/pangolin.hpp>

namespace ze {

//! A change event callback triggered, whenever the value of the wrapped scalar
//! is changed.
template<typename Scalar>
using ChangeEventCallback = std::function<void(Scalar)>;

//! A wrapper for primitive typically numeric types that calls a callback and
//! notifies a pangolin plotter about changes.
//! A pangolin context watching the primitive is only created if the watch
//! is createad with a name or from a value.
template<class T>
class PrimitiveTypeWrapperImpl {
public:
  using value_type = T;

  PrimitiveTypeWrapperImpl()
    : member_name_("Default")
  {
  }

  PrimitiveTypeWrapperImpl(T v)
    : value_(v)
    , member_name_("Default")
  {
    initialize();
  }

  PrimitiveTypeWrapperImpl(T v, bool watch)
    : PrimitiveTypeWrapperImpl(v)
  {
    if (watch)
    {
      initialize();
    }
  }

  PrimitiveTypeWrapperImpl(const PrimitiveTypeWrapperImpl<T>& rhs)
    : PrimitiveTypeWrapperImpl(rhs.value_)
  {
  }

  //! A string constructor to name the context / window of pangolin.
  PrimitiveTypeWrapperImpl(const std::string& member_name)
    : member_name_(member_name)
  {
    initialize();
  }

  operator T() const {return value_;}

  void change_callback(T value)
  {
    ze::PangolinPlotter::instance().log(member_name_, value);
  }

  // Modifiers
  PrimitiveTypeWrapperImpl& operator=(T v) { value_=v; notify(); return *this; }
  PrimitiveTypeWrapperImpl& operator+=(T v) { value_+=v; notify(); return *this; }
  PrimitiveTypeWrapperImpl& operator-=(T v) { value_-=v; notify(); return *this; }
  PrimitiveTypeWrapperImpl& operator*=(T v) { value_*=v; notify(); return *this; }
  PrimitiveTypeWrapperImpl& operator/=(T v) { value_/=v; notify(); return *this; }
  PrimitiveTypeWrapperImpl& operator%=(T v) { value_%=v; notify(); return *this; }
  PrimitiveTypeWrapperImpl& operator++() { ++value_; notify(); return *this; }
  PrimitiveTypeWrapperImpl& operator--() { --value_; notify(); return *this; }
  PrimitiveTypeWrapperImpl operator++(int) { notify(); return PrimitiveTypeWrapperImpl(value_++, false); }
  PrimitiveTypeWrapperImpl operator--(int) { notify(); return PrimitiveTypeWrapperImpl(value_--, false); }
  PrimitiveTypeWrapperImpl& operator&=(T v) { value_&=v; notify(); return *this; }
  PrimitiveTypeWrapperImpl& operator|=(T v) { value_|=v; notify(); return *this; }
  PrimitiveTypeWrapperImpl& operator^=(T v) { value_^=v; notify(); return *this; }
  PrimitiveTypeWrapperImpl& operator<<=(T v) { value_<<=v; notify(); return *this; }
  PrimitiveTypeWrapperImpl& operator>>=(T v) { value_>>=v; notify(); return *this; }
  //accessors
  PrimitiveTypeWrapperImpl operator+() const { return PrimitiveTypeWrapperImpl(+value_, false); }
  PrimitiveTypeWrapperImpl operator-() const { return PrimitiveTypeWrapperImpl(-value_, false); }
  PrimitiveTypeWrapperImpl operator!() const { return PrimitiveTypeWrapperImpl(!value_, false); }
  PrimitiveTypeWrapperImpl operator~() const { return PrimitiveTypeWrapperImpl(~value_, false); }
  //friends
  friend PrimitiveTypeWrapperImpl operator+(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) { return iw+=v; }
  friend PrimitiveTypeWrapperImpl operator+(PrimitiveTypeWrapperImpl iw, T v) { return iw+=v; }
  friend PrimitiveTypeWrapperImpl operator+(T v, PrimitiveTypeWrapperImpl iw) { return PrimitiveTypeWrapperImpl(v, false)+=iw; }
  friend PrimitiveTypeWrapperImpl operator-(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) { return iw-=v; }
  friend PrimitiveTypeWrapperImpl operator-(PrimitiveTypeWrapperImpl iw, T v) { return iw-=v; }
  friend PrimitiveTypeWrapperImpl operator-(T v, PrimitiveTypeWrapperImpl iw) { return PrimitiveTypeWrapperImpl(v, false)-=iw; }
  friend PrimitiveTypeWrapperImpl operator*(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) { return iw*=v; }
  friend PrimitiveTypeWrapperImpl operator*(PrimitiveTypeWrapperImpl iw, T v) { return iw*=v; }
  friend PrimitiveTypeWrapperImpl operator*(T v, PrimitiveTypeWrapperImpl iw) { return PrimitiveTypeWrapperImpl(v, false)*=iw; }
  friend PrimitiveTypeWrapperImpl operator/(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) { return iw/=v; }
  friend PrimitiveTypeWrapperImpl operator/(PrimitiveTypeWrapperImpl iw, T v) { return iw/=v; }
  friend PrimitiveTypeWrapperImpl operator/(T v, PrimitiveTypeWrapperImpl iw) { return PrimitiveTypeWrapperImpl(v, false)/=iw; }
  friend PrimitiveTypeWrapperImpl operator%(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) { return iw%=v; }
  friend PrimitiveTypeWrapperImpl operator%(PrimitiveTypeWrapperImpl iw, T v) { return iw%=v; }
  friend PrimitiveTypeWrapperImpl operator%(T v, PrimitiveTypeWrapperImpl iw) { return PrimitiveTypeWrapperImpl(v, false)%=iw; }
  friend PrimitiveTypeWrapperImpl operator&(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) { return iw&=v; }
  friend PrimitiveTypeWrapperImpl operator&(PrimitiveTypeWrapperImpl iw, T v) { return iw&=v; }
  friend PrimitiveTypeWrapperImpl operator&(T v, PrimitiveTypeWrapperImpl iw) { return PrimitiveTypeWrapperImpl(v, false)&=iw; }
  friend PrimitiveTypeWrapperImpl operator|(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) { return iw|=v; }
  friend PrimitiveTypeWrapperImpl operator|(PrimitiveTypeWrapperImpl iw, T v) { return iw|=v; }
  friend PrimitiveTypeWrapperImpl operator|(T v, PrimitiveTypeWrapperImpl iw) { return PrimitiveTypeWrapperImpl(v, false)|=iw; }
  friend PrimitiveTypeWrapperImpl operator^(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) { return iw^=v; }
  friend PrimitiveTypeWrapperImpl operator^(PrimitiveTypeWrapperImpl iw, T v) { return iw^=v; }
  friend PrimitiveTypeWrapperImpl operator^(T v, PrimitiveTypeWrapperImpl iw) { return PrimitiveTypeWrapperImpl(v, false)^=iw; }
  friend PrimitiveTypeWrapperImpl operator<<(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) { return iw<<=v; }
  friend PrimitiveTypeWrapperImpl operator<<(PrimitiveTypeWrapperImpl iw, T v) { return iw<<=v; }
  friend PrimitiveTypeWrapperImpl operator<<(T v, PrimitiveTypeWrapperImpl iw) { return PrimitiveTypeWrapperImpl(v, false)<<=iw;}
  friend PrimitiveTypeWrapperImpl operator>>(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) { return iw>>=v; }
  friend PrimitiveTypeWrapperImpl operator>>(PrimitiveTypeWrapperImpl iw, T v) { return iw>>=v; }
  friend PrimitiveTypeWrapperImpl operator>>(T v, PrimitiveTypeWrapperImpl iw) { return PrimitiveTypeWrapperImpl(v, false)>>=iw; }

  //! Set the callback to trigger on change.
  void setChangeCallback(ChangeEventCallback<T> change_cb)
  {
    change_cb_ = change_cb;
  }

private:
  //! The value wrapped by the object.
  T value_;
  //! The identifier / variable name wrapped in the object and also used for
  //! identification of the plot wrt. to pangolin.
  const std::string member_name_;

  //! A callback triggered when the primitive changes.
  ChangeEventCallback<T> change_cb_;

  //! Notifiy helper function that calls the callback if defined.
  void notify()
  {
    if (change_cb_)
    {
      change_cb_(value_);
    }
  }

  //! Initialize the pangolin logging callbacks on change.
  void initialize()
  {
    this->setChangeCallback(std::bind(&PrimitiveTypeWrapperImpl<T>::change_callback,
                                      this,
                                      std::placeholders::_1));
  }
};

typedef PrimitiveTypeWrapperImpl<int> intWrapper;
typedef PrimitiveTypeWrapperImpl<unsigned> uintWrapper;
typedef PrimitiveTypeWrapperImpl<short> shortWrapper;
typedef PrimitiveTypeWrapperImpl<char> charWrapper;
typedef PrimitiveTypeWrapperImpl<unsigned long long> ullWrapper;
typedef PrimitiveTypeWrapperImpl<float> floatWrapper;
typedef PrimitiveTypeWrapperImpl<double> doubleWrapper;
typedef PrimitiveTypeWrapperImpl<long double> longDoubleWrapper;

} // namespace ze
