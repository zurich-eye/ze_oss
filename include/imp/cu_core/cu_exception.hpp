#pragma once

#include <sstream>
#include <cuda_runtime_api.h>
#include <imp/core/exception.hpp>

namespace ze {
namespace cu {

/** Assertion with additional error information
 */
class Exception : public ze::Exception
{
public:
  Exception() = default;
  virtual ~Exception() throw() = default;

  using ze::Exception::Exception;
//  Exception(const std::string& msg,
//            const char* file=nullptr, const char* function=nullptr, int line=0) throw()
//    : ze::Exception(msg, file, function, line)
//  {
//  }

  Exception(const std::string& msg, cudaError err,
            const char* file=nullptr, const char* function=nullptr, int line=0) throw()
    : msg_(msg)
    , err_(err)
    , file_(file)
    , function_(function)
    , line_(line)
  {
    std::ostringstream out_msg;

    out_msg << "IMP Exception (CUDA): ";
    out_msg << (msg_.empty() ? "unknown error" : msg_) << "\n";
    out_msg << "      cudaError code: " << cudaGetErrorString(err_);
    out_msg << " (" << err_ << ")" << "\n";
    out_msg << "      where: ";
    out_msg << (file_.empty() ? "no filename available" : file_) << " | ";
    out_msg << (function_.empty() ? "unknown function" : function_) << ":" << line_;
    msg_ = out_msg.str();
  }

  virtual const char* what() const throw()
  {
    return msg_.c_str();
  }

  std::string msg_;
  cudaError err_;
  std::string file_;
  std::string function_;
  int line_;
};

} // namespace cu
} // namespace ze

#define IMP_CU_THROW_EXCEPTION(msg, err) \
  throw ze::cu::Exception(msg, err, __FILE__, __FUNCTION__, __LINE__)

