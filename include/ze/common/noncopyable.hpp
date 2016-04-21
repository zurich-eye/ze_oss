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

} // namespace ze
