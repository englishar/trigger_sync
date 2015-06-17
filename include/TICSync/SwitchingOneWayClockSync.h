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
 
#ifndef __SwitchingOneWayClockSync_h
#define __SwitchingOneWayClockSync_h

#include "TICSync/Core.h"
#include "TICSync/LinearOneWayClockSync.h"

namespace TICSync
{

/**
 * Top level class for synchronizing clocks using one-way timing information.
 */
template< class T >
class SwitchingOneWayClockSync
{
public:

  SwitchingOneWayClockSync(const T& switchPeriod) :
    m_switchPeriod(switchPeriod)
  {}

  virtual ~SwitchingOneWayClockSync()
  {}


  /**
   * Corrects noisy and skewed time stamps for data from an external device
   * with its own clock (or perhaps a sample counter that increases linearly
   * with time)
   *
   * @param deviceTime The time stamp provided by the device from its own
   *        clock.  This could simply be a sample counter, if
   *        that is available and the values are known to
   *        increase linearly with time.
   * @param clientReceiptTime Time stamp taken on the client machine when the data
   *        arrived.
   * @return A more accurate client time stamp which has been filtered for the
   *         noise caused by variable transport delays.
   */
  T filterTimestamp(const T& deviceTime, const T& clientReceiptTime)
  {
    this->update(deviceTime, clientReceiptTime);
    if (m_activeFilter.isStable())
    {
      return m_activeFilter.deviceTimeToClientTime(deviceTime);
    }
    else
    {
      return clientReceiptTime;
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
    m_activeFilter.update(deviceTime, receiptTime);
    m_pendingFilter.update(deviceTime, receiptTime);

    if (m_pendingFilter.span() > m_switchPeriod)
    {
      m_activeFilter.swap(m_pendingFilter);
      m_pendingFilter.reset();
    }
  }

  /**
   * Set period between filter replacement
   * @param v
   */
  void setSwitchPeriod(const T& v)
  {
      m_switchPeriod = v;
  }


  /**
   * Converts device clock times to client clock times, using the currently estimated
   * linear mapping.
   * @param Ts Device time
   * @return Client time
   */
  T deviceTimeToClientTime(const T& Ts) const
  {
    return m_activeFilter.deviceTimeToClientTime(Ts);
  }

  /**
   * Converts client clock times to device clock times, using the currently estimated
   * linear mapping.
   * @param Tc Client time
   * @return Device time
   */
  T clientTimeToDeviceTime(const T& Tc) const
  {
    return m_activeFilter.clientTimeToDeviceTime(Tc);
  }


  /**
   * Reset the filter and throw away all existing data.
   */
  void reset()
  {
    m_activeFilter.reset();
    m_pendingFilter.reset();
  }

  void swap(SwitchingOneWayClockSync< T >& o)
  {
    m_activeFilter.swap(o.m_activeFilter);
    m_pendingFilter.swap(o.m_pendingFilter);
  }

  /**
   *
   * @return Difference between latest and earliest measurements
   */
  T span() const
  {
    return m_activeFilter.span();
  }


  /**
   * Indicates whether the underlying convex hulls have enough points to make a prediction.
   * @return
   */
  bool isStable() const
  {
    return m_activeFilter.isStable();
  }

  /**
   * Returns the estimated skew between the two clocks.
   * @return
   */
  double skew() const
  {
    return m_activeFilter.skew();
  }

  /**
   * Returns the estimated offset between the two clocks.
   * For clocks with non-zero skew, offset is always changing,
   * so this function return the offset at the time the the most
   * recent measurement was received.
   * @return
   */
  T mostRecentOffset() const
  {
    return m_activeFilter.mostRecentOffset();
  }

  /**
   *
   * @return Time stamp of earliest point in filter
   */
  T earliestPoint() const
  {
      return m_activeFilter.earliestPoint();
  }


  /**
   *
   * @return Time stamp of latest point in filter
   */
  T latestPoint() const
  {
      return m_activeFilter.latestPoint();
  }


private:
  LinearOneWayClockSync< T > m_activeFilter;
  LinearOneWayClockSync< T > m_pendingFilter;
  T m_switchPeriod;
};

} // namespace TICSync

#endif /* __SwitchingOneWayClockSync_h */
