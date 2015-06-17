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

#ifndef __dataTypes_h_
#define __dataTypes_h_

#include "TICSync/Core.h"
#include "TICSync/exceptions.h"
#include <iostream>
#include <vector>
#include <limits>


namespace TICSync
{

// A structure to hold segment points for the TICSync filter
template<class T>
class Point
{
public:

  Point() :
    m_x(), m_y()
  {
    //if (!std::numeric_limits<T>::is_signed())
    //  throw UnsignedTypeException();
  }

  Point(const T& x, const T& y) :
    m_x(x), m_y(y)
  {
    //if (!std::numeric_limits<T>::is_signed())
    //  throw UnsignedTypeException();
  }

  const T& x() const
  {
    return m_x;
  }

  const T& y() const
  {
    return m_y;
  }

  void dump() const
  {
    std::cout << x() << " " << y() << "\n";
  }

public:
  Point<T> operator+(const Point<T>& pt) const
  {
    return Point<T>(x() + pt.x(), y() + pt.y());
  }

  Point<T> operator-(const Point<T>& pt) const
  {
    return Point(x() - pt.x(), y() - pt.y());
  }

  Point<T> operator*(const T& v) const
  {
    return Point<T> (x() * v, y() * v);
  }

  Point<T> operator+(const T& v) const
  {
    return Point<T> (x() + v, y() + v);
  }

  Point<T> operator-(const T& v) const
  {
    return Point<T> (x() - v, y() - v);
  }

  // Be careful using division, because it could lead to an unacceptable
  // loss of precision with some types.
  Point<T> operator/(const T& v) const
  {
    return Point<T> (x()/v, y()/v);
  }

private:
  T m_x;
  T m_y;
};

template<class T>
inline std::ostream& operator<<(std::ostream& os, const Point<T>& pt)
{
  os << pt.x() << " " << pt.y();
  return os;
}

// Structure for returning segments in a convex envelope
template<class T>
class Line
{
public:
  Line() : m_p1(), m_p2()
  {
  }

  Line(const Point<T>& p1, const Point<T>& p2) :
    m_p1(p1), m_p2(p2)
  {
  }

  const Point<T>& getP1() const
  {
    return m_p1;
  }

  const Point<T>& getP2() const
  {
    return m_p2;
  }

  void setP1(const Point<T>& o)
  {
    m_p1 = o;
  }

  void setP2(const Point<T>& o)
  {
    m_p2 = o;
  }

  T dx() const
  {
    return m_p2.x() - m_p1.x();
  }

  T dy() const
  {
    return m_p2.y() - m_p1.y();
  }

  bool slopeGreaterThan(Line<T>& o)
  {
    T t = dy() * o.dx() - o.dy() * dx();
    return t > 0;
  }

  bool absSlopeGreaterThan(Line<T>& o)
  {
    T t = absT(dy() * o.dx()) - absT(o.dy() * dx());
    return t > 0;
  }

  Line<T> operator+(const Point<T>& v) const
  {
    return Line<T> (m_p1+v, m_p2+v);
  }

  Line<T> operator-(const Point<T>& v) const
  {
    return Line<T> (m_p1-v, m_p2-v);
  }

  void dump()
  {
    std::cout << "(" << m_p1.x() << ", " << m_p1.y() << "), ";
    std::cout << "(" << m_p2.x() << ", " << m_p2.y() << ")\n";
  }

private:

  T absT(T v)
  {
    return ((v < 0) ? -v : v);
  }

private:
  Point<T> m_p1;
  Point<T> m_p2;
};

// We use a vector in preference to a deque because of the faster random
// access lookups.  We only ever add or remove elements from the back, so
// there's never any shifting of elements.
// In general, the convex envelopes from timing data never grow beyond
// about 20 segments, because the clock mapping is basically linear.
// Therefore frequent reallocation doesn't occur.
template<class T>
struct PointStack
{
  typedef std::vector<Point<T> > Type;
};

// This needs to be a signed value, because sometimes we use
// a negative number to indicate an invalid index.
typedef int Index;


} // namespace TICSync

#endif // __dataTypes_h_
