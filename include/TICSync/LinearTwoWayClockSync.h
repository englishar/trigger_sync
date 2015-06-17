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
 
#ifndef __LinearTwoWayClockSync_h
#define __LinearTwoWayClockSync_h

#include "TICSync/Core.h"
#include "TICSync/MaxSepFilter.h"
#include "TICSync/TimestampMapper.h"

namespace TICSync
{

/**
 * Top level class for synchronizing clocks using two-way timing information.
 * This class assumes a linear mapping between clocks (ie a constant frequency difference)
 * If your clocks have appreciable drift (2nd order temperature effects) then you might
 * want to use the SwitchingTwoWayClockSync filter instead.
 */
template <class T>
class LinearTwoWayClockSync
{
public:

  LinearTwoWayClockSync()
  {
  }

  virtual ~LinearTwoWayClockSync()
  {
  }

  /**
   * Feed in a new set of ping-pong time stamp measurements
   *
   * @param clientRequestTime Time at which the client requested the server time
   * (measured by client clock)
   * @param serverTime Time at which the server received the request
   * (measured by server clock)
   * @param clientReceiptTime Time at which the client received the response
   * (measured by client clock)
   */
  void update(const T& clientRequestTime, const T& serverTime, const T& clientReceiptTime)
  {
    T offsetLB = clientRequestTime - serverTime;
    T offsetUB = clientReceiptTime - serverTime;

    m_maxSep.addPointPair(serverTime, offsetUB, offsetLB);
  }


  /**
   * Feed in a new set of one-way time stamp measurements for client request time
   * and server time, which provide a lower bound on the true offset.
   * To be honest, it's quite unlikely you'll use this function, but it's there
   * if you need it!
   *
   * @param clientRequestTime Time at which the client requested the server time
   * (measured by client clock)
   * @param serverTime Time at which the server received the request
   * (measured by server clock)
   */
  void updateWithRequestTimeOnly(const T& clientRequestTime, const T& serverTime)
  {
    T offsetLB = clientRequestTime - serverTime;
    m_maxSep.addLowerPoint(serverTime, offsetLB);
  }


  /**
   * Feed in a new set of one-way time stamp measurements for server time
   * and client receipt time, which provide an upper bound on the true offset.
   * This is the measurement pair that you'd normally provide for the
   * OneWayClockSync class.
   *
   * @param serverTime Time at which the server received the request
   * (measured by server clock)
   * @param clientReceiptTime Time at which the client received the response
   * (measured by client clock)
   */
   void updateWithReceiptTimeOnly(const T& serverTime, const T& clientReceiptTime)
   {
     T offsetUB = clientReceiptTime - serverTime;
     m_maxSep.addUpperPoint(serverTime, offsetUB);
   }


  /**
   * Converts server clock times to client clock times, using the currently estimated
   * linear mapping.
   * @param Ts Server time
   * @return Client time
   */
  T serverTimeToClientTime(const T& Ts) const
  {
    Line<T> seg;
    TimestampMapper<T> mapper(m_maxSep.getMidLine(seg));

    T Tc = mapper.serverTimeToClientTime(Ts);
    return Tc;
  }


  /**
   * Converts client clock times to server clock times, using the currently estimated
   * linear mapping.
   * @param Tc Client time
   * @return Server time
   */
  T clientTimeToServerTime(const T& Tc) const
  {
    Line<T> seg;
    TimestampMapper<T> mapper(m_maxSep.getMidLine(seg));

    T Ts = mapper.clientTimeToServerTime(Tc);
    return Ts;
  }


  /**
   * Reset the filter and throw away all existing data.
   */
  void reset()
  {
    m_maxSep.reset();
  }

  void swap(LinearTwoWayClockSync<T>& o)
  {
    m_maxSep.swap(o.m_maxSep);
  }


  /**
   * Indicates whether the underlying convex hulls have enough points to make a prediction.
   * @return
   */
  bool isStable() const
  {
    return m_maxSep.isStable();
  }

  /**
   *
   * @return Difference between latest and earliest measurements
   */
  T span() const
  {
    return m_maxSep.span();
  }

  /**
   * Returns the (current) estimated skew between the two clocks.
   * @return
   */
  double skew() const
  {
    Line<T> seg;
    m_maxSep.getMidLine(seg);

    double a = static_cast<double> (seg.dy()) / static_cast<double> (seg.dx());
    return a;
  }

  /**
   * Returns the estimated offset between the two clocks.
   * For clocks with non-zero skew, offset is always changing,
   * so this function returns the offset at the time the the most
   * recent measurement was received.
   * @return
   */
  T mostRecentOffset() const
  {
    Line<T> seg;
    m_maxSep.getMidLine(seg);

    T serverTime = m_maxSep.latestPointTop();
    T clientTime = serverTimeToClientTime(serverTime);
    T off = clientTime-serverTime;

    return off;
  }

  /**
   * Writes debugging information to stdout
   */
  void dump() const
  {
    m_maxSep.dump();
  }

private:
  MaxSepFilter<T> m_maxSep;
};


} // namespace TICSync

#endif /* __LinearTwoWayClockSync_h */
