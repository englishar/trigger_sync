#ifndef TRIGGERSYNC_H
#define TRIGGERSYNC_H

#include "ros/ros.h"
#include <boost/thread/mutex.hpp>

#include <ros/callback_queue.h>

#include "message_filters/synchronizer.h"
#include <message_filters/subscriber.h>
#include "message_filters/sync_policies/approximate_time.h"
//#include "trigger_sync/Trigger.h" // indulde trigger messages
#include "TICSync/LinearOneWayClockSync.h"
#include "TICSync/LinearTwoWayClockSync.h"

#include "TICSync/SwitchingOneWayClockSync.h"
#include "TICSync/SwitchingTwoWayClockSync.h"
#include "trigger_sync/Event.h"
#include "trigger_sync/EventStamped.h"
//#include <trigger_sync/Trigger.h>
//#include "trigger_sync/ClockMapping.h"

#include <cstdio>

#define LOCAL_QUEUE 0
#define REMOTE_QUEUE 1

using namespace message_filters;
using namespace message_filters::sync_policies;


// Todo:
//   -- allow out of order adding of points with a buffer.
//   -- packet matching for trigger events


// Note only need event_id if you are using trigger matching. Otherwise just transmits events

// Will subscribe to /sync topic to listen for other mappings that might be better than this one.


class TriggerSync
{
public:

  TriggerSync(void);
  TriggerSync(std::string device_clock_id, std::string local_clock_id);
  TriggerSync(std::string device_clock_id, std::string local_clock_id, std::string trigger_event_id);
  TriggerSync(const std::string trigger_name);

  // For one way clock sync
  ros::Time update(
      ros::Time device_time,  //  Time at time server or sensor
      ros::Time local_time //
      );

  // One way clock update and publish/subscribe to event_id
  ros::Time update(
      ros::Time device_time,  //  Time at time server or sensor
      ros::Time local_time, //
      std::string event_id   // Name of the event

      );

  // For two way clock sync
  ros::Time updateTwoWay(
      ros::Time local_request_time, //
      ros::Time device_time,  //  Time at time server or sensor
      ros::Time local_response_time //
      );

  // For two way clock sync with event_id
  ros::Time updateTwoWay(
      ros::Time local_request_time, //
      ros::Time device_time,  //  Time at time server or sensor
      ros::Time local_response_time, //
      std::string event_id
      );

  ros::Time deviceTimeToClientTime(ros::Time device_time );

  ros::Time convertTime(ros::Time from_time, std::string from_clock_id, std::string to_clock_id ); // Will currently throw an exception

  void setSwitchPeriod(ros::Duration);

  ros::Time correctReceiveTime(trigger_sync::Event& event_msg);

  bool bufferMsgs(trigger_sync::Event& event_msg);
  void addMsgToEstimators(trigger_sync::Event add_msg);
  void addMsgToApproximateSync(const trigger_sync::Event::ConstPtr& event_msg , int queue);


  void setPublishEvents(bool);
  void setMatchEvents(bool);
  double skew();
  bool  isStable();

  uint buff_size;
  static const ros::Time UNKNOWN_TIME;
  void updateMsg(trigger_sync::Event & event_msg);


//  static const uint64_t Unknown = 0;


private:

  void init();

  bool publish_events_;
  bool match_events_;
  double min_transport_delay_;


  std::string device_clock_id_;
  std::string local_clock_id_;

// // Follows naming conventions from Zhang 2002
// struct boundPoint {
//   int64_t t_i;  // t_i PC time
//   int64_t d_i;  // d_i PC time - sensor time
// };

// bool aboveHull( std::vector<boundPoint> & hull  , boundPoint& point) ;
// bool belowHull( std::vector<boundPoint> & hull  , boundPoint& point) ;

 void trigger_msg_cb_(const trigger_sync::Event::ConstPtr& trigger);
 void trigger_match_cb_(const trigger_sync::EventStampedConstPtr& p, const trigger_sync::EventStampedConstPtr& q);

 ros::Publisher event_pub_;
 ros::Publisher event_pub_matched_;

 ros::Time estimator_min_time_;
 std::vector<trigger_sync::Event> events_;  // Queue for allowing out of order events


  boost::mutex trigger_mutex_;       // Lock this mutex before touching the triggers vector. This includes reads
  std::string trigger_event_id_;           // Do not change this after initialisation - its value is not protected by the mutex
  ros::CallbackQueue trigger_cb_queue_;               // Callback queue for /trigger messages
  boost::shared_ptr<ros::Subscriber> trigger_sub_;    // Subscriber for /trigger messages
  boost::shared_ptr<ros::AsyncSpinner> trigger_spin_; //

  boost::shared_ptr<TICSync::SwitchingOneWayClockSync<int64_t> > one_way_clock_sync_;
  boost::shared_ptr<TICSync::SwitchingTwoWayClockSync<int64_t> > two_way_clock_sync_;

  typedef Synchronizer<ApproximateTime<trigger_sync::EventStamped, trigger_sync::EventStamped> > Sync2;
//  Sync2 sync_;

  typedef  std::map<std::string, boost::shared_ptr<Sync2> > message_matching_map_t;
  message_matching_map_t message_matching_map;




};

class TimeException : public std::exception
{
public:
  TimeException(const char* format, ...)
  {
    va_list list;
    va_start(list,format);
    vsprintf(message, format, list);
    va_end(list);
  }

  virtual const char* what() const throw()
  {
    return message;
  }

private:
  char message[1000];
};

#endif
