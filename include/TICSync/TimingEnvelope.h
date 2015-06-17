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

#ifndef __TimingEnvelope_h
#define __TimingEnvelope_h

#include "TICSync/Core.h"
#include "TICSync/dataTypes.h"
#include "TICSync/ConvexEnvelope.h"
#include "TICSync/exceptions.h"
#include <algorithm>
#include <functional>


namespace TICSync
{

// Makes use of a ConvexEnvelope to do timing specific things
template<class T>
class TimingEnvelope
{
public:

  TimingEnvelope(ConvexEnvelopeTypes::Direction aboveOrBelow) :
    m_hull(aboveOrBelow), m_longestSegID(0), m_longestSegLen(),
        m_midpointSegID(0), m_centroidSegID(0), m_sumX()
  {
    reset();
  }

  virtual ~TimingEnvelope()
  {}

  void swap(TimingEnvelope<T>& o)
  {
    m_hull.swap(o.m_hull);
    std::swap(m_longestSegID, o.m_longestSegID);
    std::swap(m_longestSegLen, o.m_longestSegLen);
    std::swap(m_midpointSegID, o.m_midpointSegID);
    std::swap(m_centroidSegID, o.m_centroidSegID);
    std::swap(m_sumX, o.m_sumX);
  }

  void reset()
  {
    m_hull.reset();

    m_longestSegID  = 0;
    m_longestSegLen = 0;
    m_midpointSegID = 0;
    m_centroidSegID = 0;
    m_sumX = 0;
  }

  void addPoint(const T& x, const T& y)
  {

    try
    {
      m_hull.addPoint(x, y);
    }
    catch (TICSync::InvalidPointException& e)
    {
      throw TICSync::TimeJumpException();
    }

    // Now update internal pointers and counters

    // TODO: m_sumX should be a sum of *offset* x values, to mitigate against
    //       overflow.  We should subtract the initial x value from each summand.
    m_sumX = m_sumX + x;

    if (getNumMeas() > 1)
    {
      postAdd_updateLongestSeg();
      postAdd_updateMidpointSeg();
      postAdd_updateCentroidSeg();
    }
  }


  Index size() const
  {
    return m_hull.size();
  }

  Index getNumMeas() const
  {
    return m_hull.getNumMeas();
  }

  bool isStable() const
  {
    return m_hull.size() > 1;
  }

  bool isEmpty() const
  {
    return m_hull.isEmpty();
  }

  T span() const
  {
    return m_hull.span();
  }

  T earliestPoint() const
  {
    return m_hull.earliestPoint();
  }

  T latestPoint() const
  {
    return m_hull.latestPoint();
  }

  const Line<T>& getLastSeg(Line<T>& seg) const
  {
    return m_hull.getSegment(size() - 1, seg);
  }

  const Line<T>& getLongestSeg(Line<T>& seg) const
  {
    return m_hull.getSegment(m_longestSegID, seg);
  }

  const Line<T>& getMidpointSeg(Line<T>& seg) const
  {
    return m_hull.getSegment(m_midpointSegID, seg);
  }

  const Line<T>& getCentroidSeg(Line<T>& seg) const
  {
    return m_hull.getSegment(m_centroidSegID, seg);
  }

  const Point<T>& getPoint(Index index) const
  {
    return m_hull.getPoint(index);
  }

  void getSegment(Index index, Line<T>& seg) const
  {
    m_hull.getSegment(index, seg);
  }


public:

  // Iterators for the underlying stack
  typedef typename ConvexEnvelope<T>::const_iterator const_iterator;
  typedef typename ConvexEnvelope<T>::const_reverse_iterator const_reverse_iterator;

  const_iterator begin() const
  {
    return m_hull.begin();
  }

  const_iterator end() const
  {
    return m_hull.end();
  }

  const_reverse_iterator rbegin() const
  {
    return m_hull.rbegin();
  }

  const_reverse_iterator rend() const
  {
    return m_hull.rend();
  }


public:

  template<class TT>
  friend std::ostream& operator<<(std::ostream& os, const TimingEnvelope<TT>& filt);


  void dump() const
  {
    m_hull.dump();

    std::cout << "Longest  Seg: " << m_longestSegID;
    std::cout << " (length=" << m_longestSegLen << ")\n";
    std::cout << "Midpoint Seg: " << m_midpointSegID << "\n";
    std::cout << "Centroid Seg: " << m_centroidSegID << std::endl;
  }

private:

  // This function should *only* be called from addPoint, as it exploits knowledge
  // about the behaviour of that algorithm.
  void postAdd_updateLongestSeg()
  {
    Index npts = size();

    Index lastSegID = npts - 1;
    Line<T> lastSeg;
    getSegment(lastSegID, lastSeg);

    // Update longest seg ID if it has changed
    if (m_longestSegID >= lastSegID || m_longestSegLen <= lastSeg.dx())
    {
      m_longestSegID = lastSegID;
      m_longestSegLen = lastSeg.dx();
    }

  }

  // This function should *only* be called from addPoint, as it exploits knowledge
  // about the behaviour of that algorithm.
  void postAdd_updateMidpointSeg()
  {
    Index npts = size();

    if (npts < 2)
      throw TICSync::NotEnoughPtsException();

    if (npts == 2 || m_midpointSegID >= npts)
    {
      m_midpointSegID = npts - 1;
      return;
    }

    // Get correct midpoint
    double dfMid = (begin()->x() + rbegin()->x()) / 2.0;

    // Search right from old midpoint-spanning segment until we find a point with x
    // greater than dfMid
    const_iterator it(begin());
    std::advance(it, m_midpointSegID);
    while (it != end() && it->x() <= dfMid)
      ++it;

    if (it == end())
      throw TICSync::BadPointIndexException();

    // Update class member
    m_midpointSegID = std::distance(begin(), it);
  }


  // This function should *only* be called from addPoint, as it exploits knowledge
  // about the behaviour of that algorithm.
  void postAdd_updateCentroidSeg()
  {
    Index npts = size();

    if (npts < 2)
      throw TICSync::NotEnoughPtsException();

    if (npts == 2 || m_centroidSegID >= npts)
    {
      m_centroidSegID = npts - 1;
      return;
    }

    // Get correct centroid.  Hopefully the division won't cause
    // too much of a loss of precision for integer types.
    double centroid = m_sumX / getNumMeas();

    const_iterator it(begin());
    std::advance(it, m_centroidSegID);
    while (it != end() && it->x() <= centroid)
      ++it;

    if (it == end())
      throw TICSync::BadPointIndexException();

    // Update class member
    m_centroidSegID = std::distance(begin(), it);
  }

private:

  ConvexEnvelope<T> m_hull;

  Index m_longestSegID;
  T m_longestSegLen;

  Index m_midpointSegID;
  Index m_centroidSegID;

  T m_sumX;
};


//
// inlines
//

template<class T>
inline std::ostream& operator<<(std::ostream& os, const TimingEnvelope<T>& filt)
{
  os << filt.m_hull;
  return os;
}


} // namespace TICSync

#endif // __TimingEnvelope_h
