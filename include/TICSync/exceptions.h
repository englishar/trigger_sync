/* 
 * copyright {
 * TICSync - A Clock Synchronization Library
 * Copyright 2012 Isis Innovation Limited
 * University of Oxford
 * 
 * Authors: Alastair Harrison
 *          arh@robots.ox.ac.uk
 *
 * See included LICENSE file for license details
 * }
 */

#ifndef __exceptions_h
#define __exceptions_h

#include "TICSync/Core.h"
#include <exception>
#include <string>

namespace TICSync
{

  class NotEnoughPtsException : public std::exception
  {
  public:
    virtual const char* what() const throw ()
    {
      return "Too few data points to perform requested operation.";
    }
  };

  class UnknownEnvTypeException : public std::exception
  {
  public:
    virtual const char* what() const throw ()
    {
      return "Unknown Envelope type.";
    }
  };

  class BadPointIndexException : public std::exception
  {
  public:
    virtual const char* what() const throw ()
    {
      return "Bad index for requested point.";
    }
  };

  class InvalidPointException : public std::exception
  {
  public:
    virtual const char* what() const throw ()
    {
      return "New points may only be added at the end of the convex envelope.";
    }
  };

  class TimeJumpException : public std::exception
  {
  public:
    virtual const char* what() const throw ()
    {
      return "Time has jumped backwards.  Can't build a convex envelope with this point.";
    }
  };

  // Hopefully this will never need to be thrown, as it would
  // indicate a logic error of some sort.
  class UnexpectedAlgorithmFailure : public std::exception
  {
  public:
    virtual const char* what() const throw ()
    {
      return "Algorithm has encountered a case it cannot deal with.";
    }
  };

  class UnsignedTypeException : public std::exception
  {
  public:
    virtual const char* what() const throw ()
    {
      return "You must use a signed type for this templated class.";
    }
  };

}

#endif // __exceptions_h
