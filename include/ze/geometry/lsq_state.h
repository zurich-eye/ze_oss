#pragma once

#include <tuple>
#include <type_traits>
#include <typeinfo>
#include <iostream>
#include <glog/logging.h>
#include <ze/common/types.h>

namespace ze {

namespace internal {

template<typename T> struct TupleGetDimension;

template<typename Element>
struct TupleGetDimension<std::tuple<Element>>
{
 static const unsigned int dim = traits<Element>::dimension;
};

template<typename Element, typename... Elements>
struct TupleGetDimension<std::tuple<Element, Elements...>>
{
  static const unsigned int dim = traits<Element>::dimension
           + TupleGetDimension<std::tuple<Elements...>>::dim;
};

} // namespace internal

// -----------------------------------------------------------------------------
// A State can contain fixed-sized elements such as Scalars, Vector3, Transformation.
template<typename... Elements>
class State
{
public:
  typedef State<Elements...> StateT;
  typedef decltype(std::tuple<Elements...>()) StateTuple;
  static const unsigned int size = std::tuple_size<StateTuple>::value;
  static const unsigned int dimension = internal::TupleGetDimension<StateTuple>::dim;

  template <size_t i>
  using ElementType = typename std::tuple_element<i, StateTuple>::type;

  typedef Eigen::Matrix<FloatType, dimension, 1> TangentVector;
  typedef Eigen::Matrix<FloatType, dimension, dimension> Jacobian;

  StateTuple state_;

  void print() const
  {
    printImpl<0>();
    std::cout << "--\n";
  }

  void retract(const TangentVector& v)
  {
    retractImpl<0,0>(v);
  }

private:
  template<unsigned int i, typename std::enable_if<(i<size)>::type* = nullptr>
  inline void printImpl() const
  {
    using T = ElementType<i>;
    std::cout << "--\nState-Index: " << i << "\n"
              << "Type = " << typeid(T).name() << "\n"
              << "Dimension = " << traits<T>::dimension << "\n"
              << "Value = \n" << std::get<i>(state_) << "\n";
    printImpl<i+1>();
  }

  template<unsigned int i, typename std::enable_if<(i>=size)>::type* = nullptr>
  inline void printImpl() const
  {}

  template<unsigned int i=0, unsigned int j=0, typename std::enable_if<(i<size)>::type* = nullptr>
  inline void retractImpl(const TangentVector& v)
  {
    static_assert(j < dimension, "");
    using T = ElementType<i>;
    //typename traits<T>::TangentVector t = v.template segment<traits<T>::dimension>(j);

    std::get<i>(state_) = traits<T>::Retract(
           std::get<i>(state_),
           v.template segment<traits<T>::dimension>(j));
    retractImpl<i+1, j+traits<T>::dimension>(v);
  }

  template<unsigned int i=0, unsigned int j=0, typename std::enable_if<(i>=size)>::type* = nullptr>
  inline void retractImpl(const TangentVector& v)
  {}
};


// -----------------------------------------------------------------------------
// Manifold traits for fixed-size state.
template<typename T> struct traits;

template<typename... Elements>
struct traits<State<Elements...>>
{
  typedef State<Elements...> StateT;
  static const unsigned int dimension = StateT::dimension;

  typedef Eigen::Matrix<FloatType, dimension, 1> TangentVector;
  typedef Eigen::Matrix<FloatType, dimension, dimension> Jacobian;

  static StateT Retract(const StateT& origin, const TangentVector& v,
                      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if(H1 || H2)
    {
      LOG(FATAL) << "Not implemented.";
    }
    StateT origin_perturbed = origin;
    origin_perturbed.retract(v);
    return origin_perturbed;
  }

  /*
  static bool Equals(const Matrix& v1, const Matrix& v2, double tol = 1e-8)
  {
    if (v1.size() != v2.size())
      return false;
    return (v1 - v2).array().abs().maxCoeff() < tol;
    // TODO(cfo): Check for nan entries.
  }

  static TangentVector Local(Matrix origin, Matrix other,
                            Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1) (*H1) = -Jacobian::Identity();
    if (H2) (*H2) =  Jacobian::Identity();
    TangentVector result;
    Eigen::Map<Matrix>(result.data()) = other - origin;
    return result;
  }
  */
};







} // namespace ze
