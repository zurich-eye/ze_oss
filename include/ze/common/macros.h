#pragma once

#include <memory>

#define ZE_POINTER_TYPEDEFS(TypeName)               \
  typedef std::unique_ptr<TypeName> UniquePtr;      \
  typedef std::shared_ptr<TypeName> Ptr;            \
  typedef std::shared_ptr<const TypeName> ConstPtr

#define ZE_DELETE_COPY_ASSIGN(TypeName)             \
  TypeName(const TypeName&) = delete;               \
  void operator=(const TypeName&) = delete

// Give the compiler a hint that an if statement is false.
#define ZE_UNLIKELY(x)                              \
  __builtin_expect((bool)(x), 0)

// Give the compiler a hint that an if statement is true.
#define ZE_LIKELY(x)                                \
  __builtin_expect((bool)(x), 1)