#pragma once

namespace ze {

//! A change event callback triggered, whenever the value of the wrapped scalar
//! is changed.
template<typename Scalar>
using ChangeEventCallback = std::function<void(Scalar)>;

//! A wrapper for primitive typically numeric types.
template<class T>
class PrimitiveTypeWrapperImpl {
public:
    typedef T value_type;
    PrimitiveTypeWrapperImpl() :value() {}
    PrimitiveTypeWrapperImpl(T v) :value(v) {}
    operator T() const {return value;}
    //modifiers
    PrimitiveTypeWrapperImpl& operator=(T v) {value=v; notify(); return *this;}
    PrimitiveTypeWrapperImpl& operator+=(T v) {value+=v; notify(); return *this;}
    PrimitiveTypeWrapperImpl& operator-=(T v) {value-=v; notify(); return *this;}
    PrimitiveTypeWrapperImpl& operator*=(T v) {value*=value; notify(); return *this;}
    PrimitiveTypeWrapperImpl& operator/=(T v) {value/=value; notify(); return *this;}
    PrimitiveTypeWrapperImpl& operator%=(T v) {value%=value; notify(); return *this;}
    PrimitiveTypeWrapperImpl& operator++() {++value; notify(); return *this;}
    PrimitiveTypeWrapperImpl& operator--() {--value; notify(); return *this;}
    PrimitiveTypeWrapperImpl operator++(int) {notify(); return PrimitiveTypeWrapperImpl(value++);}
    PrimitiveTypeWrapperImpl operator--(int) {notify(); return PrimitiveTypeWrapperImpl(value--);}
    PrimitiveTypeWrapperImpl& operator&=(T v) {value&=v; notify(); return *this;}
    PrimitiveTypeWrapperImpl& operator|=(T v) {value|=v; notify(); return *this;}
    PrimitiveTypeWrapperImpl& operator^=(T v) {value^=v; notify(); return *this;}
    PrimitiveTypeWrapperImpl& operator<<=(T v) {value<<=v; notify(); return *this;}
    PrimitiveTypeWrapperImpl& operator>>=(T v) {value>>=v; notify(); return *this;}
    //accessors
    PrimitiveTypeWrapperImpl operator+() const {return PrimitiveTypeWrapperImpl(+value);}
    PrimitiveTypeWrapperImpl operator-() const {return PrimitiveTypeWrapperImpl(-value);}
    PrimitiveTypeWrapperImpl operator!() const {return PrimitiveTypeWrapperImpl(!value);}
    PrimitiveTypeWrapperImpl operator~() const {return PrimitiveTypeWrapperImpl(~value);}
    //friends
    friend PrimitiveTypeWrapperImpl operator+(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) {return iw+=v;}
    friend PrimitiveTypeWrapperImpl operator+(PrimitiveTypeWrapperImpl iw, T v) {return iw+=v;}
    friend PrimitiveTypeWrapperImpl operator+(T v, PrimitiveTypeWrapperImpl iw) {return PrimitiveTypeWrapperImpl(v)+=iw;}
    friend PrimitiveTypeWrapperImpl operator-(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) {return iw-=v;}
    friend PrimitiveTypeWrapperImpl operator-(PrimitiveTypeWrapperImpl iw, T v) {return iw-=v;}
    friend PrimitiveTypeWrapperImpl operator-(T v, PrimitiveTypeWrapperImpl iw) {return PrimitiveTypeWrapperImpl(v)-=iw;}
    friend PrimitiveTypeWrapperImpl operator*(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) {return iw*=v;}
    friend PrimitiveTypeWrapperImpl operator*(PrimitiveTypeWrapperImpl iw, T v) {return iw*=v;}
    friend PrimitiveTypeWrapperImpl operator*(T v, PrimitiveTypeWrapperImpl iw) {return PrimitiveTypeWrapperImpl(v)*=iw;}
    friend PrimitiveTypeWrapperImpl operator/(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) {return iw/=v;}
    friend PrimitiveTypeWrapperImpl operator/(PrimitiveTypeWrapperImpl iw, T v) {return iw/=v;}
    friend PrimitiveTypeWrapperImpl operator/(T v, PrimitiveTypeWrapperImpl iw) {return PrimitiveTypeWrapperImpl(v)/=iw;}
    friend PrimitiveTypeWrapperImpl operator%(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) {return iw%=v;}
    friend PrimitiveTypeWrapperImpl operator%(PrimitiveTypeWrapperImpl iw, T v) {return iw%=v;}
    friend PrimitiveTypeWrapperImpl operator%(T v, PrimitiveTypeWrapperImpl iw) {return PrimitiveTypeWrapperImpl(v)%=iw;}
    friend PrimitiveTypeWrapperImpl operator&(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) {return iw&=v;}
    friend PrimitiveTypeWrapperImpl operator&(PrimitiveTypeWrapperImpl iw, T v) {return iw&=v;}
    friend PrimitiveTypeWrapperImpl operator&(T v, PrimitiveTypeWrapperImpl iw) {return PrimitiveTypeWrapperImpl(v)&=iw;}
    friend PrimitiveTypeWrapperImpl operator|(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) {return iw|=v;}
    friend PrimitiveTypeWrapperImpl operator|(PrimitiveTypeWrapperImpl iw, T v) {return iw|=v;}
    friend PrimitiveTypeWrapperImpl operator|(T v, PrimitiveTypeWrapperImpl iw) {return PrimitiveTypeWrapperImpl(v)|=iw;}
    friend PrimitiveTypeWrapperImpl operator^(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) {return iw^=v;}
    friend PrimitiveTypeWrapperImpl operator^(PrimitiveTypeWrapperImpl iw, T v) {return iw^=v;}
    friend PrimitiveTypeWrapperImpl operator^(T v, PrimitiveTypeWrapperImpl iw) {return PrimitiveTypeWrapperImpl(v)^=iw;}
    friend PrimitiveTypeWrapperImpl operator<<(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) {return iw<<=v;}
    friend PrimitiveTypeWrapperImpl operator<<(PrimitiveTypeWrapperImpl iw, T v) {return iw<<=v;}
    friend PrimitiveTypeWrapperImpl operator<<(T v, PrimitiveTypeWrapperImpl iw) {return PrimitiveTypeWrapperImpl(v)<<=iw;}
    friend PrimitiveTypeWrapperImpl operator>>(PrimitiveTypeWrapperImpl iw, PrimitiveTypeWrapperImpl v) {return iw>>=v;}
    friend PrimitiveTypeWrapperImpl operator>>(PrimitiveTypeWrapperImpl iw, T v) {return iw>>=v;}
    friend PrimitiveTypeWrapperImpl operator>>(T v, PrimitiveTypeWrapperImpl iw) {return PrimitiveTypeWrapperImpl(v)>>=iw;}

    void setChangeCallback(ChangeEventCallback<T> change_cb)
    {
      change_cb_ = change_cb;
    }
private:
    T value;

    ChangeEventCallback<T> change_cb_;

  void notify() { /*change_cb_ &&*/ change_cb_(value); }
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
