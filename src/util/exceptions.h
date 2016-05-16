#ifndef __UMF_EXCEPTIONS_H
#define __UMF_EXCEPTIONS_H

#include <exception>

namespace umf {

class UMFException: public std::exception
{
  virtual const char* what() const throw()
  {
    return "UMF Exception";
  }
};

}

#endif // EXCEPTIONS_H
