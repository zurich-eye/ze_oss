#pragma once

#include <tuple>
#include <type_traits>
#include <typeinfo>
#include <iostream>
#include <glog/logging.h>
#include <ze/common/types.h>
#include <ze/common/manifold.h>

namespace ze {

namespace internal {

// -----------------------------------------------------------------------------
// Check if any element in the tuple is of dynamic size
template<typename T> struct TupleIsFixedSize;

template<typename Element>
struct TupleIsFixedSize<std::tuple<Element>>
{
 static constexpr bool is_fixed_size = (traits<Element>::dimension > 0);
};

template<typename Element, typename... Elements>
struct TupleIsFixedSize<std::tuple<Element, Elements...>>
{
  static constexpr bool is_fixed_size = (traits<Element>::dimension > 0)
           && TupleIsFixedSize<std::tuple<Elements...>>::is_fixed_size;
};

// -----------------------------------------------------------------------------
// Sum the dimensions of all elements in a tuple.
template<typename T> struct TupleGetDimension;

template<typename Element>
struct TupleGetDimension<std::tuple<Element>>
{
  static constexpr int dimension = traits<Element>::dimension;
};

template<typename Element, typename... Elements>
struct TupleGetDimension<std::tuple<Element, Elements...>>
{
  static constexpr int dimension = traits<Element>::dimension
           + TupleGetDimension<std::tuple<Elements...>>::dimension;
};

} // namespace internal

// -----------------------------------------------------------------------------
// A State can contain fixed-sized elements such as Scalars, Vector3, Transformation.
template<typename... Elements>
class State
{
public:

  // typedefs
  using StateT = State<Elements...>;
  using StateTuple = decltype(std::tuple<Elements...>());

  enum StateSize : int { size = std::tuple_size<StateTuple>::value };
  enum StateDimension : int {
    // Dimension is -1 if State is not of fixed size (e.g. contains VectorX).
    dimension = (internal::TupleIsFixedSize<StateTuple>::is_fixed_size)
      ? internal::TupleGetDimension<StateTuple>::dimension : Eigen::Dynamic
  };

  using TangentVector = Eigen::Matrix<FloatType, dimension, 1> ;
  using Jacobian = Eigen::Matrix<FloatType, dimension, dimension>;

  // utility
  template <size_t i>
  using ElementType = typename std::tuple_element<i, StateTuple>::type;

  StateTuple state_;

  void print() const
  {
    printImpl<0>();
    std::cout << "--\n";
  }

  void retract(const TangentVector& v)
  {
    CHECK_EQ(v.size(), getDimension());
    retractImpl<0>(v, 0);
  }

  //! Get actual dimension of the state, also if state is of dynamic size.
  int getDimension() const
  {
    return getDimensionImpl<0>();
  }

  //! Get reference to element.
  template<uint32_t i>
  inline auto at() -> decltype (std::get<i>(state_)) &
  {
    return std::get<i>(state_);
  }

  //! Get const reference to element.
  template<uint32_t i>
  inline auto at() const -> decltype (std::get<i>(state_)) &
  {
    return std::get<i>(state_);
  }

  //! Returns whether one element of the state is of dynamic size.
  static constexpr bool isDynamicSize()
  {
    return (dimension == -1);
  }

  //! Returns whether element i is of dynamic size.
  template<uint32_t i>
  static constexpr bool isElementDynamicSize()
  {
    return (traits<ElementType<i>>::dimension == -1);
  }

private:

  //!@ getDimension recursive implementation.
  //!{
  template<uint32_t i, typename std::enable_if<(i<size-1 && traits<ElementType<i>>::dimension != -1)>::type* = nullptr>
  inline int getDimensionImpl() const
  {
    return traits<ElementType<i>>::dimension + getDimensionImpl<i+1>();
  }

  template<uint32_t i, typename std::enable_if<(i<size-1 && traits<ElementType<i>>::dimension == -1)>::type* = nullptr>
  inline int getDimensionImpl() const
  {
    return traits<ElementType<i>>::getDimension(std::get<i>(state_)) + getDimensionImpl<i+1>();
  }

  template<uint32_t i,typename std::enable_if<(i==size-1 && traits<ElementType<i>>::dimension != -1)>::type* = nullptr>
  inline int getDimensionImpl() const
  {
    return traits<ElementType<i>>::dimension;
  }

  template<uint32_t i, typename std::enable_if<(i==size-1 && traits<ElementType<i>>::dimension == -1)>::type* = nullptr>
  inline int getDimensionImpl() const
  {
    return traits<ElementType<i>>::getDimension(std::get<i>(state_));
  }
  //!}

  //!@ print recursive implementation.
  //!{
  template<uint32_t i, typename std::enable_if<(i<size)>::type* = nullptr>
  inline void printImpl() const
  {
    using T = ElementType<i>;
    std::cout << "--\nState-Index: " << i << "\n"
              << "Type = " << typeid(T).name() << "\n"
              << "Dimension = " << traits<T>::dimension << "\n"
              << "Value = \n" << std::get<i>(state_) << "\n";
    printImpl<i+1>();
  }

  template<uint32_t i, typename std::enable_if<(i>=size)>::type* = nullptr>
  inline void printImpl() const
  {}
  //!}

  //!@ retract recursive implementation.
  //!{
  template<uint32_t i=0, typename std::enable_if<(i<size-1 && traits<ElementType<i>>::dimension != -1)>::type* = nullptr>
  inline void retractImpl(const TangentVector& v, uint32_t j)
  {
    using T = ElementType<i>;
    std::get<i>(state_) = traits<T>::Retract(
           std::get<i>(state_),
           v.template segment<traits<T>::dimension>(j));
    retractImpl<i+1>(v, j + traits<T>::dimension);
  }

  template<uint32_t i=0, typename std::enable_if<(i<size-1 && traits<ElementType<i>>::dimension == -1)>::type* = nullptr>
  inline void retractImpl(const TangentVector& v, uint32_t j)
  {
    using T = ElementType<i>;
    const int element_dim = traits<T>::getDimension(std::get<i>(state_));
    std::get<i>(state_) = traits<T>::Retract(
           std::get<i>(state_), v.segment(j, element_dim));
    retractImpl<i+1>(v, j + element_dim);
  }

  template<uint32_t i=0, typename std::enable_if<(i==size-1 && traits<ElementType<i>>::dimension != -1)>::type* = nullptr>
  inline void retractImpl(const TangentVector& v, uint32_t j)
  {
    using T = ElementType<i>;
    std::get<i>(state_) = traits<T>::Retract(
           std::get<i>(state_),
           v.template segment<traits<T>::dimension>(j));
  }

  template<uint32_t i=0, typename std::enable_if<(i==size-1 && traits<ElementType<i>>::dimension == -1)>::type* = nullptr>
  inline void retractImpl(const TangentVector& v, uint32_t j)
  {
    using T = ElementType<i>;
    const int element_dim = traits<T>::getDimension(std::get<i>(state_));
    std::get<i>(state_) = traits<T>::Retract(
           std::get<i>(state_), v.segment(j, element_dim));
  }
  //!}

};


// -----------------------------------------------------------------------------
// Manifold traits for fixed-size state.
template<typename T> struct traits;

template<typename... Elements>
struct traits<State<Elements...>>
{
  typedef State<Elements...> StateT;
  enum { dimension = StateT::dimension };

  typedef Eigen::Matrix<FloatType, dimension, 1> TangentVector;
  typedef Eigen::Matrix<FloatType, dimension, dimension> Jacobian;

  static StateT Retract(
           const StateT& origin, const TangentVector& v,
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
