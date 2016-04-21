#pragma once

namespace ze {

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
