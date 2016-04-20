#pragma once

namespace ze {

/**
 * Denotes a class that cannot be copied.
 * Depending on the class semantics it may be possible to move the object.
 **/
class Noncopyable
{
public:
  Noncopyable() = default;
  ~Noncopyable() = default;
  Noncopyable(const Noncopyable& other) = delete;
  Noncopyable& operator=(const Noncopyable& other) = delete;
};

/**
 * Denotes a class that can be copied but may not be assigned to
 * (e.g. because of reference member variables)
 **/
class Nonassignable
{
public:
  Nonassignable() = default;
  Nonassignable(const Nonassignable& other) = default;
  Nonassignable(Nonassignable&& other) = default;
  ~Nonassignable() = default;
  Nonassignable& operator=(const Nonassignable& other) = delete;
  Nonassignable& operator=(Nonassignable&& other) = delete;
};

} // namespace ze
