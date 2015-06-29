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

// Class for perfomring time syncronisation routines

#include "trigger_sync/trigger_sync.h"


// Todo:: perhaps get rid of the two way stuff and put in separate class - makes it difficult to read
// TODO: perhaps change to just one constructor with default arguments for publish_events and match_events and trigger_id and get rid of reading params

//--------------------------
// Public methods


TriggerSync::TriggerSync()
{
    publish_events_ = false;
    match_events_ = false;

    ros::Time temp;
    init();
}

// Constructor with named clocks
// Allows sending event messages on /event so other time syncronisers can make use of these events.
TriggerSync::TriggerSync(std::string device_clock_id, std::string local_clock_id)
{
    publish_events_ = true;
    match_events_ = false;

    init();

    device_clock_id_ = device_clock_id;
    local_clock_id_ =  local_clock_id;

    // Advertise event topic
    ros::NodeHandle nh;
    event_pub_ = nh.advertise<trigger_sync::Event>("/event", 10);
    event_pub_matched_ = nh.advertise<trigger_sync::Event>("/event_matched",10);

}


// Constructor with named clocks and trigger_event_id
// This will set up a subscriber to /event topic and listen for events caused by the events named trigger_event_id
TriggerSync::TriggerSync(std::string device_clock_id, std::string recv_clock_id, std::string trigger_event_id)
{
    publish_events_ = true;
    match_events_ = true;

    init();

    device_clock_id_ = device_clock_id;
    local_clock_id_ =  recv_clock_id;
    trigger_event_id_ = trigger_event_id;

    // Create the subscriber to the topic trigger_id
    ros::NodeHandle nh;
    nh.setCallbackQueue(&trigger_cb_queue_);

    // Advertise /event topic
    event_pub_ = nh.advertise<trigger_sync::Event>("/event", 10);
    event_pub_matched_ = nh.advertise<trigger_sync::Event>("/event_matched",10);

    // Subscibe to /event topic
    trigger_sub_.reset(new ros::Subscriber(nh.subscribe("/event", 10, &TriggerSync::trigger_msg_cb_, this)));

    // Start new thread to recieve /event messages (so we don't need to call ros::spin() )
    trigger_spin_.reset(new ros::AsyncSpinner(1, &trigger_cb_queue_));
    trigger_spin_->start();
}

// One way update with defual event_id
ros::Time TriggerSync::update( ros::Time device_time, ros::Time local_receive_time )
{
    return updateTwoWay(UNKNOWN_TIME, device_time, local_receive_time, trigger_event_id_);
}

// One way update with event_id
ros::Time TriggerSync::update(ros::Time device_time,ros::Time response_receive_time, std::string event_id ){
    return updateTwoWay(UNKNOWN_TIME, device_time, response_receive_time, event_id );
}

// Two way update
ros::Time TriggerSync::updateTwoWay(ros::Time local_request_time,ros::Time device_time, ros::Time local_receive_time ){
    return updateTwoWay(local_request_time, device_time, local_receive_time, trigger_event_id_);
}

// Two way update with device id
ros::Time TriggerSync::updateTwoWay(ros::Time local_request_time,ros::Time device_time, ros::Time local_receive_time, std::string event_id ){

    boost::mutex::scoped_lock lock(trigger_mutex_);

    // Create the event message
    trigger_sync::Event event_msg;
    event_msg.local_request_time = local_request_time;
    event_msg.event_name = event_id;
    event_msg.device_clock_id = device_clock_id_;
    event_msg.device_time = device_time ;
    event_msg.local_clock_id = local_clock_id_;
    event_msg.local_receive_time = local_receive_time;
    event_msg.corrected_local_time = UNKNOWN_TIME;
    event_msg.clock_skew = skew();
    event_msg.min_transport_delay = ros::Duration(min_transport_delay_);


    // Correct timestamp
    ros::Time corrected_local_time = correctLocalReceiveTime(event_msg);

    // Publish the event (for other processes to use);
    if (publish_events_)
    {
        //ROS_INFO("Publishing message");
        event_pub_.publish(event_msg);
    }

    // Add message to approximate syncronisers
    if (match_events_)
    {
        ROS_DEBUG("Adding update message to approximate syncronisers");
        trigger_sync::EventPtr event_msg_ptr;
        event_msg_ptr.reset(new trigger_sync::Event(event_msg));
        addMsgToApproximateSync(event_msg_ptr,LOCAL_QUEUE);
    }

    // Add message to estimators
    ROS_DEBUG("Calling updateMsg");
    addUnorderedEventToEstimators(event_msg);


    return corrected_local_time;

}

void TriggerSync::setPublishEvents(bool publish_events)
{
    publish_events_ = publish_events;
}


void TriggerSync::setMatchEvents(bool match_events)
{
    match_events_ = match_events;
}

ros::Time TriggerSync::deviceTimeToClientTime(ros::Time device_time )
{

    ros::Time corrected_time ;
    if(one_way_clock_sync_->isStable()) {
        corrected_time = ros::Time(one_way_clock_sync_->deviceTimeToClientTime(device_time.toNSec()) / 1.0e9);
    }
    else
    {
        corrected_time = ros::Time::now();
    }

    return corrected_time;
}

//-------------------------------------
// Private methods
//-------------------------------------


void TriggerSync::init() {


    double switching_time; // seconds // Note switching time is measured in device time. Could be an issue if using sequence numbers as clocks

    ros::NodeHandle local_nh("~");
    local_nh.getParam("triggersync_publish_events", publish_events_);
    local_nh.getParam("triggersync_match_events", match_events_);
    double switch_period;
    local_nh.param("triggersync_switch_period",switching_time, 60.0 );
    local_nh.param("triggersync_transport_delay", min_transport_delay_, 0.0);

    ROS_INFO_STREAM("Triggersync params: triggersync_publish_events: " << publish_events_ << ", triggersync_match_events:" << match_events_ << "triggersync_switch_period:" << switching_time << "timesync_transport_delay:" << min_transport_delay_ );
    one_way_clock_sync_.reset(new TICSync::SwitchingOneWayClockSync<int64_t>(switching_time *1e9 ));
    two_way_clock_sync_.reset(new TICSync::SwitchingTwoWayClockSync<int64_t>(switching_time *1e9 ));

    trigger_event_id_ = "";
    buff_size_ = 20;             // TODO: Make this value configurable.

    estimator_min_time_ = ros::Time(0);


}


// Callback for when a /event message is recieved
void TriggerSync::trigger_msg_cb_(const trigger_sync::Event::ConstPtr& event_msg)
{

    boost::mutex::scoped_lock lock(trigger_mutex_);

    if (!match_events_)
        return;

    ROS_DEBUG_STREAM("Received message on /event. Event name= " << event_msg->event_name << ", devcice_clock_id= " << event_msg->device_clock_id << ", device_time= " << event_msg->device_time);

    // Check if event is relevant to our estimator
    if (event_msg->event_name != trigger_event_id_ || event_msg->local_clock_id != local_clock_id_ )
    {
        //ROS_WARN_STREAM("Rejecting packet.  '" << event_msg->local_clock_id  << "' != '"  << local_clock_id_ << "' or " << event_msg->event_name << "' != '" <<  trigger_event_id_ );
        return;
    }

    if(device_clock_id_ == event_msg->device_clock_id)
    {
        ROS_DEBUG("Ignoring packet it probably came from us");
        return;
    }

    addMsgToApproximateSync(event_msg, REMOTE_QUEUE);

}


void TriggerSync::addMsgToApproximateSync(const trigger_sync::Event::ConstPtr& event_msg , int queue)
{

    if(event_msg->event_name != trigger_event_id_ || trigger_event_id_ == "" )
        return;

    if (event_msg->corrected_local_time == TriggerSync::UNKNOWN_TIME)
    {
        ROS_WARN("Not adding packet to queue %d. Corrected time unkonwn", queue);
        return;
    }

    // Queue 0 is for all the local packets.
    // If queue is 0 then we need to add  the event to all the available syncronisers
    if (queue == LOCAL_QUEUE)
    {

        for(message_matching_map_t::iterator it = message_matching_map.begin(); it != message_matching_map.end(); ++it)
        {
            // Add the mesage to the packet matcher
            //      ROS_DEBUG_STREAM("Added device time " << event_msg->device_time << " to queue (0) with "  << event_msg->device_clock_id);
            ROS_DEBUG_STREAM("Appoximate sync. About to add event_name " << event_msg->event_name << ". Send time = " << event_msg->device_time <<" local_receive_time = (" << queue << ") " << event_msg->local_receive_time << "to approxsync " << it->first);
            boost::shared_ptr<trigger_sync::EventStamped>  p(new trigger_sync::EventStamped);
            p->event = *event_msg;
            p->header.stamp = std::min(event_msg->corrected_local_time ,event_msg->local_receive_time - ros::Duration(min_transport_delay_));
            p->header.frame_id="test";
            it->second->add<0>(p);
        }
        return;
    }

    if (queue == REMOTE_QUEUE)
    {

        if ( message_matching_map.count(event_msg->device_clock_id) == 0)
        {
            ROS_DEBUG_STREAM("Creating aproximate syncroniser for device_clock_id " << event_msg->device_clock_id );
            message_matching_map[event_msg->device_clock_id].reset(new Sync2(20));
            message_matching_map[event_msg->device_clock_id]->registerCallback(boost::bind(&TriggerSync::trigger_match_cb_, this, _1, _2));
        }

        // Add the mesage to the packet matcher
        boost::shared_ptr<trigger_sync::EventStamped>  p(new trigger_sync::EventStamped);
        p->event = *event_msg;

        // Fill out the header stamp - this is the time that the approximate sync matches with
        p->header.stamp = std::min(event_msg->corrected_local_time, event_msg->local_receive_time - event_msg->min_transport_delay);
//        p->header.frame_id="test";

        ROS_DEBUG_STREAM("About to add " << event_msg->event_name << ". Send time = " << event_msg->device_time <<" local_receive_time = (" << queue << ") " << event_msg->local_receive_time << "to approxsync " << event_msg->device_clock_id);
        message_matching_map[event_msg->device_clock_id]->add<REMOTE_QUEUE>(p);

        return;

    }

    ROS_ERROR("Attempting to add message to invalid apprixomate synchroniser queue - %d. (Valid values are 0 and 1)",queue);
    exit(1);
}


void TriggerSync::trigger_match_cb_(const trigger_sync::EventStampedConstPtr& local_message, const trigger_sync::EventStampedConstPtr& remote_message){

    if ( fabs( (remote_message->event.local_receive_time -  local_message->event.local_receive_time).toSec() ) > 0.1  ){
        ROS_ERROR_STREAM("Packet matchis greater that 100ms " << remote_message->event.local_receive_time -  local_message->event.local_receive_time);
        //    exit(1);
    }

    // Create a message that gets added to the estimators
    if ( remote_message->event.local_receive_time <=  local_message->event.local_receive_time )
    {

        // Remote message as earlier recieve time. Replace recieve time and add to estimators
        trigger_sync::Event event_msg;
        event_msg = local_message->event;
        event_msg.local_receive_time = remote_message->event.local_receive_time;

        static double min_receive_delta = std::numeric_limits<double>::max();

        min_receive_delta = std::min(min_receive_delta,(local_message->event.local_receive_time - remote_message->event.local_receive_time).toSec()*1000.0 );
//        ROS_INFO_STREAM(" Match( Match dela= "  << NUM_FORMAT <<(local_message->header.stamp - remote_message->header.stamp).toSec()*1000.0
//                        << "ms, receive time delta = " << NUM_FORMAT  <<(local_message->event.local_receive_time - remote_message->event.local_receive_time).toSec()*1000.0
//                        << "ms ) - add to buffer - " << min_receive_delta << "(" << 500/min_receive_delta
//                        << "Hz" );

        // TODO: should only do this if we are sure its a good match - perhaps wait until stable
        addUnorderedEventToEstimators(event_msg);
    } else {
//        ROS_INFO_STREAM(" Match( Match dela= " << NUM_FORMAT << (local_message->header.stamp - remote_message->header.stamp).toSec()*1000.0
//                        << "ms, receive time delta = " << NUM_FORMAT  <<(local_message->event.local_receive_time - remote_message->event.local_receive_time).toSec()*1000.0
//                        << "ms ) - not adding to buffer");
    }




    // " << message->header.stamp  << "," << message->timestamp << "," << trigger_message->header.stamp);
    //  ROS_DEBUG_STREAM("About to update ticsync via match:, send time: " << trigger_message->timestamp.toNSec() << " receive time:" << message->timestamp.toNSec());

    //  one_way_clock_sync_->update( trigger_message->timestamp.toNSec(), message->timestamp.toNSec() );

}



ros::Time TriggerSync::correctLocalReceiveTime(trigger_sync::Event& event_msg)
{

    // TODO: lock mutex?

    if (event_msg.device_clock_id != device_clock_id_ ||   event_msg.local_clock_id != local_clock_id_  ) // Forward lookup
    {
        TimeException t("Invalid clock IDs. device_clock_id and recieve_clock_id both need to be either '%s'' or '%s'" , device_clock_id_.c_str(), local_clock_id_.c_str());
        throw(t);
    }

    double span_min =0.0 ;// 3 * 1e9;  // Limit on filter span - prevents filter from outputing corrected mesurements for a few moments after startup.
    double skew_max = 1.0e-2;          // Limit on the valid values for skew. Avoids outputing corrected times for a few momoents while the filter stabilises.

    ros::Time corrected_time;
    if ( event_msg.device_time == UNKNOWN_TIME )
    {
        ROS_DEBUG("Cannot correct time unknown, cannot correct");
        corrected_time = event_msg.local_receive_time;
    }
    else if (  two_way_clock_sync_->isStable() && two_way_clock_sync_->span() > span_min && fabs(two_way_clock_sync_->skew()) < skew_max )
    {
        // Use two way estimator
        ROS_DEBUG("Correcting timestamp forward lookup - two way");
        corrected_time =  ros::Time(two_way_clock_sync_->serverTimeToClientTime(event_msg.device_time.toNSec()) / 1.0e9);
    }
    else if (one_way_clock_sync_->isStable()  && one_way_clock_sync_->span() > span_min && fabs(one_way_clock_sync_->skew()) < skew_max )
    {
        // Use one way estimator
        corrected_time = ros::Time(one_way_clock_sync_->deviceTimeToClientTime(event_msg.device_time.toNSec()) / 1.0e9);
        ROS_DEBUG_STREAM("Correcting timestamp forward lookup - one way: " << corrected_time  << " <= " << event_msg.device_time);
    }
    else if  (event_msg.local_receive_time != UNKNOWN_TIME &&  event_msg.local_request_time != UNKNOWN_TIME )
    {
        // Not yet stable
        ROS_DEBUG("Correcting timestamp forward lookup - unstable");
        corrected_time = ros::Time(0.5 * (event_msg.local_receive_time.toSec() + event_msg.local_request_time.toSec() ));
    }
    else
    {
        ROS_DEBUG_STREAM("Not corrrecting returning local recieve time " );
        corrected_time = event_msg.local_receive_time;
    }
    event_msg.corrected_local_time = corrected_time;
    return corrected_time;
}


// Allows out of order adding of events to the estimaors
void TriggerSync::addUnorderedEventToEstimators(trigger_sync::Event & event_msg)
{
    // Check that our clock ID's are for our estimator
    if  (event_msg.device_clock_id != device_clock_id_ || event_msg.local_clock_id != local_clock_id_  )
    {
        TimeException t("Invalid clock IDs. device_clock_id and recieve_clock_id both need to be either %s or %s" , device_clock_id_.c_str(), local_clock_id_.c_str());
        throw(t);
    }

    if (event_msg.device_time == UNKNOWN_TIME || (event_msg.local_receive_time == UNKNOWN_TIME && event_msg.local_request_time == UNKNOWN_TIME))
    {
        ROS_WARN_STREAM("Cannot use message due to UNKNOWN_TIME. ("
                        "local_request_time " << event_msg.local_request_time << ", "
                        "device_time"         <<  event_msg.device_time       << ", "
                        "local_receive_time " << event_msg.local_receive_time << ")" );
        return;
    }

    if (bufferMsgs(event_msg))
    {
        //    event_pub_.publish(event_msg);
        //    ROS_ERROR_STREAM("Actually adding packet to estimatore with receive time " << event_msg.local_receive_time << " and send time " << event_msg.device_time);
        addMsgToEstimators(event_msg);
    }
}


//----------------------------------------------------------
// Add message to a queue to allow out of order addition
// (Note it is the device time that must increase each time. Local time may go forwards or backwards)
// TODO: change this to a separate class for separate testing
/**
 * @brief TriggerSync::bufferMsgs
 * Fixed size buffer for re-ordering event in chronological order
 *
 * reorderEventBuffer.addEvent();
 *
 * Our clock estimators require events to be added in chronological order (according to their device time) though local time
 * may go forwards or backwards. This function stores recent events in a fixed size buffer and once the buffer is full returns the earliest
 * event.
 *
 * If the buffer contains an event with a device time identical to the current event then the local_request_time
 * and local_recieve_time are modified to contain the timestames with highest quality timestamps (lowest commuinication delay). *
 *
 * @param event_msg event to add to the buffer. event_msg is modified to contain the most recent event.
 * @return Returns FALSE if buffer is still filling. TRUE if event_msg is the most recent event.
 */
bool TriggerSync::bufferMsgs(trigger_sync::Event& event_msg)
{

    if (events_.size()  == 0 )
    {
        estimator_min_time_ = event_msg.device_time ;
    }
    else if ( event_msg.device_time <= estimator_min_time_) // TODO set estimator min time at some point
    {
        ROS_WARN_STREAM("Cant add packet event buffer. Consider increaseing buff_size_. " << estimator_min_time_ <<  " <= " << event_msg.device_time);
        return false;
    }

    // Check through vector for duplicate device times (since TICSYNC needs an ever increaseing device time)
    // While also finding the earliest event.
    int pos = -1;  // Position of the earliest event
    ros::Time earliest_local_time = event_msg.device_time;
    for(uint i = 0; i < events_.size(); i++)
    {

        if (events_[i].device_time < earliest_local_time)
        {
            earliest_local_time = events_[i].device_time;
            pos = i;
        }
        else if (events_[i].device_time == event_msg.device_time)
        {

            ROS_DEBUG_STREAM("Swapping out packet. Already have packet with send time   " << event_msg.device_time);

            // Set local_request_time to maximum possible time (later request times are better)
            if(events_[i].local_request_time == UNKNOWN_TIME)
            {
                // Currently stored request time is unknown - take the new request time ( may also be unknown)
                events_[i].local_request_time == event_msg.local_request_time;
            }
            else if (event_msg.local_request_time != UNKNOWN_TIME)
            {
                // Both request times are known - take the latest (largest) since it had a smaller communication delay
                events_[i].local_request_time = std::max(events_[i].local_request_time, event_msg.local_request_time);
            }


            // Set local_receive_time to minimum posible time (earlier local recieve times are better)
            if(events_[i].local_receive_time == UNKNOWN_TIME)
            {
                // Currently stored recieve time is unknown - take the new receive time (may also be unknown)
                events_[i].local_receive_time == event_msg.local_receive_time;
            }
            else if (event_msg.local_receive_time != UNKNOWN_TIME)
            {
                // Both recieve times are known, take the earliest (smallest) since it had a smaller communication delay
                events_[i].local_receive_time = std::min(events_[i].local_receive_time, event_msg.local_receive_time );
            }

            // No need to return an event, since we didn't add anything to the buffer but simply updated the timestamps of the duplicate event.
            return false;
        }
    }


    // If we got to here then the event is unique. Add it to the buffer.
    if (events_.size() < buff_size_ )
    {
        events_.push_back(event_msg);
        ROS_DEBUG("Buffer not yet full");
        return false;
    } else if (pos == -1)
    {
        ROS_DEBUG("This message is already the earliest in the buffer");
         estimator_min_time_ = event_msg.device_time;
        return true;
    } else  {
        ROS_DEBUG_STREAM("Swapping out message in position  "  << pos);

        // Swap current message and earliest message in the buffer and reutrn the earliest message
        trigger_sync::Event tmp_msg;
        tmp_msg = events_[pos];
        events_[pos] = event_msg;
        event_msg = tmp_msg;

        estimator_min_time_ = event_msg.device_time;        // Remember the timestamp for this message so we can later reject messages earier than this.
        return true;
    }

}

//----------------------------------------------------------
// Give add_msg to the estimators (assumed to be in order by this stage)
void TriggerSync::addMsgToEstimators(trigger_sync::Event add_msg)
{

    event_pub_matched_.publish(add_msg);

    if (add_msg.device_time == UNKNOWN_TIME)
    {
        return;
    }

    // Add to the two way estimator:
    if(add_msg.local_request_time == UNKNOWN_TIME && add_msg.local_receive_time != UNKNOWN_TIME   )
    {
        two_way_clock_sync_->updateWithReceiptTimeOnly(add_msg.device_time.toNSec(), add_msg.local_receive_time.toNSec());
        ROS_DEBUG("Did two way update with receipt time only");
    }
    else if(add_msg.local_request_time != UNKNOWN_TIME && add_msg.local_receive_time == UNKNOWN_TIME   )
    {
        two_way_clock_sync_->updateWithRequestTimeOnly(add_msg.local_request_time.toNSec(), add_msg.device_time.toNSec());
        ROS_DEBUG("Did two way update with request time only");
    }
    else
    {
        two_way_clock_sync_->update(add_msg.local_request_time.toNSec(), add_msg.device_time.toNSec(),add_msg.local_receive_time.toNSec());
        ROS_DEBUG("Did two way update with three timestamps");
    }

    // Add to the one way estimator
    if (add_msg.local_receive_time != UNKNOWN_TIME)
    {
        one_way_clock_sync_->update(add_msg.device_time.toNSec(), add_msg.local_receive_time.toNSec());
        ROS_DEBUG_STREAM("Did one way update. device time = " << add_msg.device_time << " local receive timie = " << add_msg.local_receive_time);
    }


    trigger_sync::Event event_msg = add_msg;

    correctLocalReceiveTime(event_msg);

    // Some basic clock upset detection
    if (       event_msg.corrected_local_time  <  event_msg.local_request_time
               &&  event_msg.corrected_local_time != UNKNOWN_TIME
               &&  event_msg.local_request_time   != UNKNOWN_TIME)
    {
        ROS_ERROR("corrected_local_time is before local_request_time. Reseting filters");
        if (two_way_clock_sync_->isStable())
        {
            two_way_clock_sync_->reset();
        }
    }

    if (       event_msg.corrected_local_time  >  event_msg.local_receive_time
               &&  event_msg.corrected_local_time != UNKNOWN_TIME
               &&  event_msg.local_receive_time != UNKNOWN_TIME)
    {

        if (two_way_clock_sync_->isStable())
        {
            ROS_ERROR("corrected_local_time is after local_receive_time. Resetting filters");
            two_way_clock_sync_->reset();
        }


    }

}

double TriggerSync::skew(){

    if (two_way_clock_sync_->isStable() )
    {
        return(two_way_clock_sync_->skew());
    }
    else if (one_way_clock_sync_->isStable())
    {
        return(one_way_clock_sync_->skew());
    }
    else
    {
        return 0;
    }

}


bool TriggerSync::isStable()
{
    return two_way_clock_sync_->isStable()?two_way_clock_sync_->isStable():one_way_clock_sync_->isStable();
}

void TriggerSync::setSwitchPeriod(ros::Duration switch_period)
{
    two_way_clock_sync_->setSwitchPeriod(switch_period.toNSec());
    one_way_clock_sync_->setSwitchPeriod(switch_period.toNSec());
}
