#ifndef CORE_EXCEPTION_HPP
#define CORE_EXCEPTION_HPP

#include <sstream>

namespace imp {

/** Exception with additional error information
 */
class Exception : public std::exception
{
public:
  Exception() = default;
  virtual ~Exception() throw() = default;

  Exception(const std::string& msg,
            const char* file=nullptr, const char* function=nullptr, int line=0) throw():
    msg_(msg),
    file_(file),
    function_(function),
    line_(line)
  {
    std::ostringstream out_msg;

    out_msg << "IMP Exception: ";
    out_msg << (msg_.empty() ? "unknown error" : msg_) << "\n";
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
  std::string file_;
  std::string function_;
  int line_;
};

#define IMP_THROW_EXCEPTION(msg) throw imp::Exception(msg, __FILE__, __FUNCTION__, __LINE__)

}

#endif // CORE_EXCEPTION_HPP

