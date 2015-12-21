#pragma once

#include <memory>

#define ZE_POINTER_TYPEDEFS(TypeName)               \
  typedef std::unique_ptr<TypeName> UniquePtr;      \
  typedef std::shared_ptr<TypeName> Ptr;            \
  typedef std::shared_ptr<const TypeName> ConstPtr