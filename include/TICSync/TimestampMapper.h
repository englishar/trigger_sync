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
 
#ifndef TIMESTAMPMAPPER_H_
#define TIMESTAMPMAPPER_H_

#include "TICSync/Core.h"
#include "TICSync/dataTypes.h"


namespace TICSync
{


/**
 * This class is used to perform the mapping between client times and server times, given a
 * pair of points lying on alpha*x + beta
 * It tries to maintain as much precision as possible for integer types.
 */
template <class T>
class TimestampMapper
{
public:
  /**
   * The line segment represents the linear mapping between two clocks.  Server on the X axis
   * and Client on the Y axis.  The end points of the line represent two valid mappings
   * between the clocks.  Assuming the mapping is linear, this class can then obtain the
   * mapping for any other time.
   *
   * @param seg A line segment to be used for mapping Time Stamps from one clock to another
   * @return
   */
  TimestampMapper(Line<T> seg) : m_seg(seg)
  {}

  virtual ~TimestampMapper()
  {}

  /**
   * Converts server clock times to client clock times, using the mapping
   * supplied to the constructor.
   * @param Ts Server time
   * @return Client time
   */
  T serverTimeToClientTime(const T& Ts) const
  {
    T px = m_seg.getP2().x();
    T py = m_seg.getP2().y();
    T dx = m_seg.dx();
    T dy = m_seg.dy();

    // Temporarily cast to double so that we can do a floating point operation
    // All of the cast quantities are relative, not absolute, so we shouldn't lose
    // precision on 64-bit integer epoch times.  Let's hope all these casts
    // aren't too slow!
    double alpha = static_cast<double> (dy) / static_cast<double> (dx);
    double xdiff = static_cast<double> (Ts - px);
    T ydiff = static_cast<T> (alpha * xdiff);
    T Tc = Ts + py + ydiff;

    return Tc;
  }


  /**
   * Converts client clock times to server clock times, using the mapping
   * supplied to the constructor.
   * @param Ts Server time
   * @return Client time
   */
  T clientTimeToServerTime(const T& Tc) const
  {
    T px = m_seg.getP2().x();
    T py = m_seg.getP2().y();
    T dx = m_seg.dx();
    T dy = m_seg.dy();

    // Temporarily cast to double so that we can do a floating point operation
    // All of the cast quantities are relative, not absolute, so we shouldn't lose
    // precision on 64-bit integer epoch times.  Let's hope all these casts
    // aren't too slow!
    double alpha = static_cast<double> (dy) / static_cast<double> (dx);
    double num   = static_cast<double> (Tc - px - py);
    double den   = static_cast<double> (alpha + 1);

    double TsOffset = static_cast<T> (num/den);
    T Ts = static_cast<T> (TsOffset) + px;

    return Ts;
  }

private:

  const Line<T> m_seg;
};


} // TICSync


#endif /* TIMESTAMPMAPPER_H_ */
