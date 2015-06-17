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
 
#ifndef __SwitchingTwoWayClockSync_h
#define __SwitchingTwoWayClockSync_h

#include "TICSync/Core.h"
#include "TICSync/MaxSepFilter.h"
#include "TICSync/TimestampMapper.h"
#include "TICSync/LinearTwoWayClockSync.h"

namespace TICSync
{

/**
 * Top level class for synchronizing clocks using two-way timing information.
 */
template< class T >
class SwitchingTwoWayClockSync
{
public:

  SwitchingTwoWayClockSync(const T& switchPeriod) :
    m_switchPeriod(switchPeriod)
  {
  }

  virtual ~SwitchingTwoWayClockSync()
  {
  }

  /**
   * Feed in a new set of time stamp measurements
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
    m_activeFilter.update(clientRequestTime, serverTime, clientReceiptTime);
    m_pendingFilter.update(clientRequestTime, serverTime, clientReceiptTime);

    doHousekeeping();
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
    m_activeFilter.updateWithRequestTimeOnly(clientRequestTime, serverTime);
    m_pendingFilter.updateWithRequestTimeOnly(clientRequestTime, serverTime);

    doHousekeeping();
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
    m_activeFilter.updateWithReceiptTimeOnly(serverTime, clientReceiptTime);
    m_pendingFilter.updateWithReceiptTimeOnly(serverTime, clientReceiptTime);

    doHousekeeping();
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
   * Get period between filter replacement
   * @return
   */
  T getSwitchPeriod()
  {
    return m_switchPeriod;
  }

  /**
   * Converts server clock times to client clock times, using the currently estimated
   * linear mapping.
   * @param Ts Server time
   * @return Client time
   */
  T serverTimeToClientTime(const T& Ts) const
  {
    return m_activeFilter.serverTimeToClientTime(Ts);
  }

  /**
   * Converts client clock times to server clock times, using the currently estimated
   * linear mapping.
   * @param Tc Client time
   * @return Server time
   */
  T clientTimeToServerTime(const T& Tc) const
  {
    return m_activeFilter.clientTimeToServerTime(Tc);
  }


  /**
   * Reset the filter and throw away all existing data.
   */
  void reset()
  {
    m_activeFilter.reset();
    m_pendingFilter.reset();
  }

  /**
   * Efficient way of exchanging data with another object of the same type
   * @param o Object to exchange data with
   */
  void swap(SwitchingTwoWayClockSync< T >& o)
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

private:

  void doHousekeeping()
  {
    if (m_pendingFilter.isStable())
    {
      if (m_pendingFilter.span() > m_switchPeriod)
      {
        m_activeFilter.swap(m_pendingFilter);
        m_pendingFilter.reset();
      }
    }
  }

private:
  LinearTwoWayClockSync< T > m_activeFilter;
  LinearTwoWayClockSync< T > m_pendingFilter;
  T m_switchPeriod;
};

} // namespace TICSync

#endif /* __SwitchingTwoWayClockSync_h */
