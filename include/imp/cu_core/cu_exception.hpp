#ifndef CUCORE_CU_EXCEPTION_HPP
#define CUCORE_CU_EXCEPTION_HPP

#include <sstream>
#include <cuda_runtime_api.h>
#include <imp/core/exception.hpp>

namespace imp { namespace cu {

/** Assertion with additional error information
 */
class Exception : public imp::Exception
{
public:
  Exception() = default;
  virtual ~Exception() throw() = default;

  using imp::Exception::Exception;
//  Exception(const std::string& msg,
//            const char* file=nullptr, const char* function=nullptr, int line=0) throw()
//    : imp::Exception(msg, file, function, line)
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
} // namespace imp

#define IMP_CU_THROW_EXCEPTION(msg, err) \
  throw imp::cu::Exception(msg, err, __FILE__, __FUNCTION__, __LINE__)

#endif // CUCORE_CU_EXCEPTION_HPP

