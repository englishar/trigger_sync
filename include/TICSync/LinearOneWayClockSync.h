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
 
#ifndef __LinearOneWayClockSync_h
#define __LinearOneWayClockSync_h

#include "TICSync/Core.h"
#include "TICSync/TimingEnvelope.h"
#include "TICSync/TimestampMapper.h"
#include "TICSync/dataTypes.h"
#include <iostream>

namespace TICSync
{

/**
 * Class for correcting time stamps gathered against data obtained from an
 * external device, where only one-way timing messages are possible.
 *
 * This is an efficient implementation of the filter described by Moon, Skelly
 * and Towsley in their paper "Estimation and Removal of Clock Skew from
 * Network Delay Measurements.", INFOCOM 99
 */
template< class T >
class LinearOneWayClockSync
{
public:

  LinearOneWayClockSync() :
    m_env(ConvexEnvelopeTypes::below)
  {
  }

  virtual ~LinearOneWayClockSync()
  {
  }


  /**
   * Corrects noisy and skewed time stamps for data from an external device
   * with its own clock (or perhaps a sample counter that increases linearly
   * with time).  This method simply aggregates calls to the 'update'
   * method and the 'deviceTimeToClientTime' method, for convenience.
   *
   * @param deviceTime The time stamp provided by the device from its own
   *        clock.  This could simply be a sample counter, if
   *        that is available and the values are known to
   *        increase linearly with time.
   * @param receiptTime Time stamp taken on the client machine when the data
   *        arrived.
   * @return A more accurate local time stamp which has been filtered for the
   *         noise caused by variable transport delays.
   */
  T filterTimestamp(const T& deviceTime, const T& receiptTime)
  {
    update(deviceTime, receiptTime);
    if (m_env.isStable())
    {
      return deviceTimeToClientTime(deviceTime);
    }
    else
    {
      return receiptTime;
    }
  }


  /**
   * Feed in a new set of time stamp measurements
   *
   * @param deviceTime Time at which the device sent the message.
   * (measured by device clock)
   * @param receiptTime Time at which the client received the message.
   * (measured by client clock)
   */
  void update(const T& deviceTime, const T& receiptTime)
  {
    T offsetUB = receiptTime - deviceTime; // Upper bound on offset
    m_env.addPoint(deviceTime, offsetUB);
  }


  /**
   * Converts device clock times to client clock times, using the currently estimated
   * linear mapping.
   * @param Ts Device time
   * @return Client time
   */
  T deviceTimeToClientTime(const T& Ts) const
  {
    Line<T> seg;
    TimestampMapper<T> mapper(m_env.getMidpointSeg(seg));

    T Tc = mapper.serverTimeToClientTime(Ts);
    return Tc;
  }


  /**
   * Converts client clock times to device clock times, using the currently estimated
   * linear mapping.
   * @param Tc Client time
   * @return Device time
   */
  T clientTimeToDeviceTime(const T& Tc) const
  {
    Line<T> seg;
    TimestampMapper<T> mapper(m_env.getMidpointSeg(seg));
    T Ts = mapper.clientTimeToServerTime(Tc);
    return Ts;
  }



  /**
   * Clear all data and start from scratch
   */
  void reset()
  {
    m_env.reset();
  }


  /**
   * Cheap way to swap contents with another object of the same class.
   * @param o Object you want to swap contents with
   */
  void swap(LinearOneWayClockSync< T >& o)
  {
    m_env.swap(o.m_env);
  }


  /**
   * Returns the skew (difference in rate) between the device clock
   * and the local clock
   * @return skew value
   */
  double skew() const
  {
    Line< T > seg;
    m_env.getMidpointSeg(seg);
    return static_cast< double > (seg.dy())
        / static_cast< double > (seg.dx());
  }

  /**
   * Debugging output
   */
  void dump() const
  {
    m_env.dump();
  }


  /**
   * Indicates whether the underlying convex hulls have enough points to make a prediction.
   * @return
   */
  bool isStable() const
  {
    return m_env.isStable();
  }

  /**
   *
   * @return Difference between latest and earliest measurements
   */
  T span() const
  {
    return m_env.span();
  }


  /**
   *
   * @return Time stamp of earliest point in filter
   */
  T earliestPoint() const
  {
      return m_env.earliestPoint();
  }


  /**
   *
   * @return Time stamp of latest point in filter
   */
  T latestPoint() const
  {
      return m_env.latestPoint();
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
    m_env.getMidpointSeg(seg);
    T deviceTime = m_env.latestPoint();
    T clientTime = deviceTimeToClientTime(deviceTime);
    T off = clientTime-deviceTime;

    return off;
  }


public:

  template< class TT >
  friend std::ostream
  & operator<<(std::ostream& os, const LinearOneWayClockSync< TT >& filt);

private:

  // This will do the main estimation work
  TimingEnvelope< T > m_env;
};


template<class T>
inline std::ostream& operator<<(std::ostream& os, const LinearOneWayClockSync<T>& filt)
{
  os << filt.m_env;
  return os;
}

} // TICSync


#endif /* __LinearOneWayClockSync_h */
