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
 
#ifndef __ConvexEnvelope_h
#define __ConvexEnvelope_h

#include "TICSync/Core.h"
#include "TICSync/dataTypes.h"
#include "TICSync/geometry.h"
#include "TICSync/exceptions.h"
#include "TICSync/utils.h"
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <functional>

namespace TICSync
{

class ConvexEnvelopeTypes
{
public:
  enum Direction
  {
    above, below
  };
};

// Everything is defined inline because it's the only portable way to define class templates

template<class T>
class ConvexEnvelope
{
public:

  ConvexEnvelope(ConvexEnvelopeTypes::Direction aboveOrBelow) :
    m_aboveOrBelow(aboveOrBelow), m_uiNumMeas(0)
  {
    reset();
  }

  ConvexEnvelope(const ConvexEnvelope<T>& v) :
    m_data(v.m_data), m_aboveOrBelow(v.m_aboveOrBelow), m_uiNumMeas(
        v.m_uiNumMeas)
  {
  }

  virtual ~ConvexEnvelope()
  {
  }

  ConvexEnvelope<T>& operator=(const ConvexEnvelope<T>& v)
  {
    ConvexEnvelope<T> tmp(v);
    swap(tmp);
    return *this;
  }

  void swap(ConvexEnvelope<T>& o)
  {
    std::swap(m_data, o.m_data);
    std::swap(m_aboveOrBelow, o.m_aboveOrBelow);
    std::swap(m_uiNumMeas, o.m_uiNumMeas);
  }

public:

  void reset()
  {
    m_data.clear();
    m_data.reserve(10); // Timing hulls rarely get larger than 20 segments

    m_uiNumMeas = 0;
  }

  void addPoint(const T& x, const T& y)
  {
    if (size() > 0 && x <= m_data.back().x())
    {
      // x values must be monotonically increasing, or we can't
      // build the convex envelope incrementally.
      throw TICSync::InvalidPointException();
    }

    // Pop segments from envelope until the new point
    // maintains convexity
    Point<T> p(x, y);
    while (size() > 1 && !newPointIsConvex(p))
    {
      m_data.pop_back();
    }

    // Now we can safely add the new point
    m_data.push_back(p);

    m_uiNumMeas++;
  }


  /// Number of points supporting the hull
  Index size() const
  {
    return m_data.size();
  }

  /// Number of measurements fed into the class
  Index getNumMeas() const
  {
    return m_uiNumMeas;
  }

  bool isEmpty() const
  {
    return m_data.empty();
  }

  /**
   * Return the span of the convex hull along the x direction
   * @return
   */
  T span() const
  {
    if (isEmpty())
      return static_cast<T> (0);

    return m_data.back().x() - m_data.front().x();
  }

  T earliestPoint() const
  {
    if (isEmpty())
      throw TICSync::NotEnoughPtsException();

    return m_data.front().x();
  }

  T latestPoint() const
  {
    if (isEmpty())
      throw TICSync::NotEnoughPtsException();

    return m_data.back().x();
  }

  /// Obtain a single point from the hull
  const Point< T >& getPoint(Index index) const
  {
    try
    {
      return m_data.at(index);
    }
    catch (std::out_of_range&)
    {
      throw TICSync::BadPointIndexException();
    }
  }

  const Line< T >& getSegment(Index index, Line< T >& seg) const
  {
    try
    {
      seg.setP1(m_data.at(index - 1));
      seg.setP2(m_data.at(index));
    }
    catch (std::out_of_range&)
    {
      throw TICSync::NotEnoughPtsException();
    }

    return seg;
  }

  // Debugging stuff
  void dump() const
  {
    if (m_data.empty())
      std::cout << 0;
    else
      std::cout << m_data.size() - 1;

    std::cout << " Segments\n";
    std::cout << *(this);
  }

public:

  typedef typename PointStack<T>::Type::const_iterator const_iterator;
  typedef typename PointStack<T>::Type::const_reverse_iterator const_reverse_iterator;


  const_iterator begin() const
  {
    return m_data.begin();
  }

  const_iterator end() const
  {
    return m_data.end();
  }

  const_reverse_iterator rbegin() const
  {
    return m_data.rbegin();
  }

  const_reverse_iterator rend() const
  {
    return m_data.rend();
  }

private:

  /**
   * Determines whether adding the proposed point to the hull will maintain
   * the convexity of the hull.
   * Result depends on whether this convex envelope is above
   * or below the data
   * @param p New point to be added
   * @return True if new point maintains convexity.  False otherwise.
   */
  bool newPointIsConvex(const Point<T>& p) const
  {
    // Get the last segment in the stack
    const Point<T>& a = *(rbegin() + 1);
    const Point<T>& b = *rbegin();

    switch (m_aboveOrBelow)
    {
    case ConvexEnvelopeTypes::above:
      return !geom<T>::leftOn(a, b, p);
    case ConvexEnvelopeTypes::below:
      return !geom<T>::rightOn(a, b, p);
    default:
      throw TICSync::UnknownEnvTypeException();
    };

    return false;
  }

private:
  /// Stack containing hull points
  typename PointStack<T>::Type m_data;

  /// Convex hull type.  Set in constructor.
  ConvexEnvelopeTypes::Direction m_aboveOrBelow;

  /// Number of measurements fed into the class
  Index m_uiNumMeas;
};


//
// inlines
//

template <class T>
inline std::ostream& operator<<(std::ostream& os, const ConvexEnvelope<T>& filt)
{
  TICSync::printer<Point<T> > printer(os);
  std::for_each(filt.begin(), filt.end(), printer);

  return os;
}


} // namespace TICSync


#endif // __ConvexEnvelope_h
