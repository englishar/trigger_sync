/*
 * copyright {
 * TriggerSync - A Clock Synchronization Library for ROS
 * Copyright 2015
 * Queensland University of Technology
 *
 * Authors: Andrew English
 *          a.english@qut.edu.au
 *
 * See included LICENSE file for license details
 * }
 */


#ifndef TRIGGERSYNC_H
#define TRIGGERSYNC_H

#include "ros/ros.h"
#include <boost/thread/mutex.hpp>
#include <ros/callback_queue.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "TICSync/SwitchingOneWayClockSync.h"
#include "TICSync/SwitchingTwoWayClockSync.h"
#include "trigger_sync/EventStamped.h"

#define LOCAL_QUEUE 0
#define REMOTE_QUEUE 1

using namespace message_filters;
using namespace message_filters::sync_policies;



// Note only need event_id if you are using trigger matching. Otherwise just transmits events
// Could subscribe to /sync topic to listen for other mappings that might be better than this one.


class TriggerSync
{
public:

    // TODO: Add extra option here for just device_clock_id

    TriggerSync(void);   // Get rid of this one?
    TriggerSync(std::string device_clock_id, std::string local_clock_id);
    TriggerSync(std::string device_clock_id, std::string local_clock_id, std::string trigger_event_id);
    TriggerSync(const std::string trigger_name);


    /**
     * @brief update
     *
     * For one way clock sync
     *
     * @param device_time Time at sensor
     * @param local_time  Local receive time
     * @return Corrected local recieve time with communication delay removed
     */
    ros::Time update(
            ros::Time device_time,
            ros::Time local_time    //
            );

    // One way clock update and publish/subscribe to event_id
    // don't really need this one
    /**
     * @brief update
     * @param device_time
     * @param local_time
     * @param event_id
     * @return
     */
    ros::Time update(
            ros::Time device_time,  // Time at sensor
            ros::Time local_time,   // Local recieve time
            std::string event_id    // Name of the event
            );

    // For two way clock sync
    /**
     * @brief updateTwoWay
     * @param local_request_time
     * @param device_time
     * @param local_response_time
     * @return
     */
    ros::Time updateTwoWay(
            ros::Time local_request_time,  //
            ros::Time device_time,         //  Time at time server or sensor
            ros::Time local_response_time  //
            );

    // For two way clock sync with event_id
    ros::Time updateTwoWay(
            ros::Time local_request_time, //
            ros::Time device_time,  //  Time at time server or sensor
            ros::Time local_response_time, //
            std::string event_id
            );

    /**
     * @brief deviceTimeToClientTime
     *
     * Convert without adding event to
     * @param device_time
     * @return
     */
    ros::Time deviceTimeToClientTime(
            ros::Time device_time
            );

    //    ros::Time convertTime(ros::Time from_time, std::string from_clock_id, std::string to_clock_id ); // TODO: implement


    void setSwitchPeriod(ros::Duration);
    void setPublishEvents(bool);
    void setMatchEvents(bool);

    static const ros::Time UNKNOWN_TIME;



    double skew();
    bool  isStable();


private:




    void addMsgToEstimators(trigger_sync::Event add_msg);
    void addMsgToApproximateSync(const trigger_sync::Event::ConstPtr& event_msg , int queue);

    ros::Time correctLocalReceiveTime(trigger_sync::Event& event_msg);
    bool bufferMsgs(trigger_sync::Event& event_msg);
    void addUnorderedEventToEstimators(trigger_sync::Event & event_msg);

    void init();

    bool publish_events_;             // Whether or not to publish events on the /event topic
    bool match_events_;               // Whether or not to match local events against events recieved on the /events topic
    double min_transport_delay_;      //
    std::string device_clock_id_;     //
    std::string local_clock_id_;      //
    uint buff_size_;                  // Size of buffer for re-ordering events

    ros::Publisher event_pub_;
    ros::Publisher event_pub_matched_;   // Publish the matched events on a different topic (for visualisation purposes only)

    void trigger_msg_cb_(const trigger_sync::Event::ConstPtr& trigger);
    void trigger_match_cb_(const trigger_sync::EventStampedConstPtr& p, const trigger_sync::EventStampedConstPtr& q);

    ros::Time estimator_min_time_;
    std::vector<trigger_sync::Event> events_;            // Queue for buffereing events to allowing out of order events

    boost::mutex trigger_mutex_;                        // Lock this mutex before touching the triggers vector. This includes reads
    std::string trigger_event_id_;                      // Do not change this after initialisation - its value is not protected by the mutex
    ros::CallbackQueue trigger_cb_queue_;               // Callback queue for /event messages
    boost::shared_ptr<ros::Subscriber> trigger_sub_;    // Subscriber for /event messages
    boost::shared_ptr<ros::AsyncSpinner> trigger_spin_; // Asyncronous spinner to recieve /event messages in a separate thread (so the user doesn't need to callros::spin() ))

    boost::shared_ptr<TICSync::SwitchingOneWayClockSync<int64_t> > one_way_clock_sync_;
    boost::shared_ptr<TICSync::SwitchingTwoWayClockSync<int64_t> > two_way_clock_sync_;

    typedef Synchronizer<ApproximateTime<trigger_sync::EventStamped, trigger_sync::EventStamped> > Sync2;
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

const ros::Time TriggerSync::UNKNOWN_TIME(0);
#define NUM_FORMAT std::setiosflags(std::ios::fixed) << std::setprecision(3) << std::setw(7)

#endif
