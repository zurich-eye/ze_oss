#pragma once

#include <tuple>
#include <type_traits>
#include <iostream>

namespace ze {

template<typename... Elements>
class State
{
public:
  typedef decltype(std::tuple<Elements...>()) state_t;
  static const unsigned int Size = std::tuple_size<state_t>::value;

  state_t state_;

  void print() const
  {
    std::cout << "(";
    printImpl<0>();
    std::cout << std::endl;
  }

private:
  template<unsigned int i, typename std::enable_if<(i<Size)>::type* = nullptr>
  inline void printImpl() const
  {
    std::cout << std::get<i>(state_) << ", ";
    printImpl<i+1>();
  }

  template<unsigned int i, typename std::enable_if<(i>=Size)>::type* = nullptr>
  inline void printImpl() const
  {}
};



} // namespace ze
