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
 
#ifndef __MaxSepFilter_h
#define __MaxSepFilter_h

#include "TICSync/Core.h"
#include "TICSync/TimingEnvelope.h"
#include "TICSync/dataTypes.h"
#include "TICSync/exceptions.h"

#include <limits>
#include <cassert>

namespace TICSync
{

// Note this class has inline function definitions because that is apparently
// the only way to guarantee portability.

// TODO Some of these methods throw exceptions, which need to be documented.

/**
 * Maintains an upper and lower convex hull.  Computes on demand the lines of
 * maximum separation between the two hulls.
 */
template<class T>
class MaxSepFilter
{
public:
  MaxSepFilter() :
    m_top(ConvexEnvelopeTypes::below), m_bot(ConvexEnvelopeTypes::above), m_topClosestID(0), m_botClosestID(0), m_bClosestValid(false)
  {
  }

  ~MaxSepFilter()
  {
  }

  /// Empty all data structures and reset all counters.
  void reset()
  {
    m_top.reset();
    m_bot.reset();

    m_botClosestID = 0;
    m_topClosestID = 0;
    m_bClosestValid = false;
  }

  void swap(MaxSepFilter<T>& o)
  {
    m_top.swap(o.m_top);
    m_bot.swap(o.m_bot);
    std::swap(m_botClosestID, o.m_botClosestID);
    std::swap(m_topClosestID, o.m_topClosestID);
    std::swap(m_bClosestValid, o.m_bClosestValid);
  }

  bool isStable() const
  {
    return m_bClosestValid;
  }


  /// Add a point to the upper envelope
  void addUpperPoint(const T& x, const T& y)
  {
    m_top.addPoint(x, y);

    Index m_topLastID = m_top.size() - 1;
    if (m_topLastID - m_topClosestID <= 1 || m_topClosestID == 0)
    {
      // The new point infringed the previous lines of maximum
      // separation, so we'll need to recompute.
      updateHullProximity();
    }
  }

  /// Add a point to the lower envelope
  void addLowerPoint(const T& x, const T& y)
  {
    m_bot.addPoint(x, y);

    Index m_botLastID = m_bot.size() - 1;
    if (m_botLastID - m_botClosestID <= 1 || m_botClosestID == 0)
    {
      // The new point infringed the previous lines of maximum
      // separation, so we'll need to recompute.
      updateHullProximity();
    }
  }


  /// Add points to upper and lower envelopes at the same x location
  void addPointPair(const T& x, const T& ytop, const T& ybot)
  {
    // TODO: We need to deal with what to do if the second call
    // throws an exception or otherwise fails.  Do we need to roll-back
    // the operation to the first hull?  This would require an 'undo' method
    // for convex hulls.  Or we could make a backup copy of the top hull, but
    // that would be rather expensive.

    m_top.addPoint(x, ytop);
    m_bot.addPoint(x, ybot);

    Index m_botLastID = m_bot.size() - 1;
    Index m_topLastID = m_top.size() - 1;
    if (m_botLastID - m_botClosestID <= 1 || m_topLastID - m_topClosestID <= 1
        || m_botClosestID == 0 || m_topClosestID == 0)
    {
      updateHullProximity();
    }

  }

  T earliestPointTop() const
  {
    return m_top.earliestPoint();
  }

  T earliestPointBot() const
  {
    return m_bot.earliestPoint();
  }

  /// Returns x coord of most recent point on upper hull
  T latestPointTop() const
  {
    return m_top.latestPoint();
  }

  /// Returns x coord of most recent point on lower hull
  T latestPointBot() const
  {
    return m_bot.latestPoint();
  }

  T span() const
  {
    T minTime = std::min(earliestPointBot(), earliestPointTop());
    T maxTime = std::max(latestPointBot(), latestPointTop());
    return maxTime - minTime;
  }

  /**
   * Obtain params of lines of maximum separation through the hulls
   * \param[out] topLine A line segment lying along the upper maxsep line
   * \param[out] botLine A line segment lying along the lower maxsep line
   * Throws NotEnoughPointsException() if the hulls do not have a
   * horizontal overlap, or one or both is degenerate.
   */
  void getMaxSepParams(Line<T>& topLine, Line<T>& botLine) const
  {
    if (!m_bClosestValid)
    {
      throw TICSync::NotEnoughPtsException();
    }

    Index ntop = m_top.size();
    Index nbot = m_bot.size();

    if (ntop == 1 && nbot == 1)
    {
      topLine = Line<T> (m_top.getPoint(0), m_top.getPoint(0));
      botLine = Line<T> (m_bot.getPoint(0), m_bot.getPoint(0));
      return;
    }

    if (m_topClosestID == ntop - 1 && m_botClosestID == nbot - 1)
    {
      // This indicates a failure of the searchForClosestHullSegs
      // function.  It should never happen!
      throw TICSync::UnexpectedAlgorithmFailure();
    }

    // Find the leftmost of the closest support vectors.  This
    // will lie at the beginning of a segment which one of the MaxSep
    // lines should pass along.
    Point<T> top1 = m_top.getPoint(m_topClosestID);
    Point<T> bot1 = m_bot.getPoint(m_botClosestID);

    if (top1.x() < bot1.x())
    {
      Line<T> seg;
      m_top.getSegment(m_topClosestID + 1, seg);

      topLine = seg;
      botLine = seg - top1 + bot1;
    }
    else if (top1.x() > bot1.x())
    {
      Line<T> seg;
      m_bot.getSegment(m_botClosestID + 1, seg);

      botLine = seg;
      topLine = seg - bot1 + top1;
    }
    else // (top1.x == bot1.x)
    {
      // This is a slightly tricky case to deal with, as there are a range
      // of valid slopes.  The best we can do is to pick one in the middle.
      Line<T> meanSlopeSeg;
      getMeanSlopeSeg(meanSlopeSeg);

      topLine = meanSlopeSeg;
      botLine = meanSlopeSeg - top1 + bot1;
    }


  }

  /// Returns a line which lies midway between the maxsep lines
  /// If using integer T there may be some loss of precision
  /// because of the division by 2.
  Line< T >& getMidLine(Line< T > &midLine) const
  {
    Line< T > topLine;
    Line< T > botLine;
    getMaxSepParams(topLine, botLine);

    // Find a translation which will move the lower maxsep line
    // to the middle.
    Point< T >
        trans((topLine.getP1() - botLine.getP1()) / static_cast< T > (2));

    midLine = botLine + trans;

    return midLine;
  }


  void dump() const
  {
    std::cout << "TOP HULL:\n";
    m_top.dump();

    std::cout << "BOTTOM HULL:\n";
    m_bot.dump();

    std::cout.flush();
  }


private:

  /**
   * Searches from left to right along the upper and lower convex hull
   */
  bool searchForClosestHullSegs(Index topStartID, Index botStartID,
      Index& topID, Index& botID) const
  {

    // Starts from the given initial search points on each hull, and sweeps right
    // until the closest part between the two hulls is located.
    //
    // As new points are added to the hulls, the indices of the closest support
    // vectors between the top and bottom hulls will only ever move to the right,
    // because new points are only added on the right.

    // See how many support vectors are in each hull
    Index nTop = m_top.size();
    Index nBot = m_bot.size();

    // Check that hulls are overlapping in x direction.
    if (nTop == 0 || nBot == 0)
      return false;
    if (nTop == 1 && nBot == 1)
      return false;
    if (m_top.rbegin()->x() < m_bot.begin()->x())
      return false;
    if (m_top.begin()->x() > m_bot.rbegin()->x())
      return false;

    Index topLastID = nTop - 1;
    Index botLastID = nBot - 1;

    // The latest point added to a hull might have infringed the max sep lines,
    // in which case the initial search points will need to be updated.  In this
    // case, the new closest support vector(s) will be part of the latest segment.
    if (nTop == 1)
      topStartID = 0;
    else if (topStartID >= topLastID)
      topStartID = topLastID - 1;

    if (nBot == 1)
      botStartID = 0;
    else if (botStartID >= nBot - 1)
      botStartID = botLastID - 1;

    // Get iterators to the search start-points
    typename TimingEnvelope<T>::const_iterator top_it = m_top.begin();
    std::advance(top_it, topStartID);
    typename TimingEnvelope<T>::const_iterator bot_it = m_bot.begin();
    std::advance(bot_it, botStartID);

    // Now we're ready to start the search
    // Sweep from left to right to find closest support vectors.
    // This occurs at the point where the top hull becomes steeper
    // than the bottom hull.
    while (top_it + 1 != m_top.end() && bot_it + 1 != m_bot.end())
    {
      Line<T> topSeg(*top_it, *(top_it + 1));
      Line<T> botSeg(*bot_it, *(bot_it + 1));

      if (top_it->x() != bot_it->x())
      {
        // Compare slopes of segments immediately after the current points
        // If top section has greater (or equal) slope than bottom then we're done.
        if (!botSeg.slopeGreaterThan(topSeg))
          break;
      }

      // Top section still has lesser slope than bottom section, so
      // we're not there yet.  Move to the next support vector.
      if ((top_it + 1)->x() < (bot_it + 1)->x())
        ++top_it;
      else if ((top_it + 1)->x() > (bot_it + 1)->x())
        ++bot_it;
      else // ((top_it + 1)->x() == (bot_it + 1)->x())
      {
        // We'll choose to move along the hull which has the greatest
        // absolute slope
        if (topSeg.absSlopeGreaterThan(botSeg))
          ++top_it;
        else
          ++bot_it;
      }
    }

    // Recover indices into the vector
    topID = std::distance(m_top.begin(), top_it);
    botID = std::distance(m_bot.begin(), bot_it);

    // Only one of the closest support vectors can be allowed
    // to be at the end of its hull, unless the hulls have only a
    // single point each.
    assert((nTop == 1 && nBot == 1) || !(topID == nTop - 1 && botID == nBot - 1));

    return true;
  }


  /// A Helper function for getMaxSepParams.  Used in cases where
  /// there is a range of possible MaxSep slopes
  void getMeanSlopeSeg(Line<T>& meanSlopeSeg) const
  {
    // See how many support vectors are in each hull
    Index ntop = m_top.size();
    Index nbot = m_bot.size();

    if (ntop < 2 && nbot < 2)
      throw NotEnoughPtsException();

    Line<T> minSlope(Point<T> (0, 0), Point<T> (0, +1));
    Line<T> maxSlope(Point<T> (0, 0), Point<T> (0, -1));
    Line<T> tempSeg;

    if (ntop > m_topClosestID + 1)
    {
      m_top.getSegment(m_topClosestID + 1, tempSeg);
      if (tempSeg.slopeGreaterThan(maxSlope)) maxSlope = tempSeg;
      if (minSlope.slopeGreaterThan(tempSeg)) minSlope = tempSeg;
    }

    if (nbot > m_botClosestID + 1)
    {
      m_bot.getSegment(m_botClosestID + 1, tempSeg);
      if (tempSeg.slopeGreaterThan(maxSlope)) maxSlope = tempSeg;
      if (minSlope.slopeGreaterThan(tempSeg)) minSlope = tempSeg;
    }

    if (m_topClosestID > 0)
    {
      m_top.getSegment(m_topClosestID, tempSeg);
      if (tempSeg.slopeGreaterThan(maxSlope)) maxSlope = tempSeg;
      if (minSlope.slopeGreaterThan(tempSeg)) minSlope = tempSeg;
    }

    if (m_botClosestID > 0)
    {
      m_bot.getSegment(m_botClosestID, tempSeg);
      if (tempSeg.slopeGreaterThan(maxSlope)) maxSlope = tempSeg;
      if (minSlope.slopeGreaterThan(tempSeg)) minSlope = tempSeg;
    }

    // Make a fake segment that starts at the closest point on the
    // top hull, and has the correct mean slope.  Note we need to do
    // this without division, in case T is an integer type.
    Point<T> topClosest = m_top.getPoint(m_topClosestID);
    Point<T> diff(minSlope.dy()*maxSlope.dx() + minSlope.dx()*maxSlope.dy(),
        static_cast<T> (2)*minSlope.dx()*maxSlope.dx());
    meanSlopeSeg = Line<T>(topClosest, topClosest+diff);
  }


  /**
   * Gets called every time a newly added point lies between the existing
   * lines of maximum separation.
   */
  void updateHullProximity()
  {
    Index topID = 0, botID = 0;
    if (searchForClosestHullSegs(m_topClosestID, m_botClosestID, topID, botID))
    {
      m_topClosestID = topID;
      m_botClosestID = botID;
      m_bClosestValid = true;
    }
    else
    {
      m_topClosestID = 0;
      m_botClosestID = 0;
      m_bClosestValid = false;
    }
  }

private:
  // The underlying convex envelopes that do the hard work
  TimingEnvelope<T> m_top ; ///< The upper convex hull
  TimingEnvelope<T> m_bot ; ///< The lower convex hull

  // Indices to closest support vectors on each hull.
  Index m_topClosestID;
  Index m_botClosestID;

  // True if m_topClosestID and m_botClosestID are valid
  bool m_bClosestValid;
};

} // namespace TICSync

#endif /* __MaxSepFilter_h */
