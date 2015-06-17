// This node syncronises sets up a two way synchronise between various local clocks
// Run a copy of this node on each

// Todo: convert this to a class.

#include <ros/ros.h>
#include <trigger_sync/trigger_sync.h>

boost::shared_ptr<ros::Publisher> event_pub;
std::string clock_name = "clock";
std::map<std::string, boost::shared_ptr <TriggerSync> > sync_map;

void event_msg_callback(const trigger_sync::EventConstPtr& event_msg)
{

  ros::Time t = ros::Time::now();

  trigger_sync::Event new_event_msg = *event_msg;
  // Check for time request
  if (
         event_msg->local_clock_id     != clock_name     // Event was not sent by us
      && event_msg->local_request_time != ros::Time(0)   // Request time is defined
      && event_msg->device_time        == ros::Time(0)   // Device time is undefined
      && event_msg->local_receive_time == ros::Time(0)   // Recieve time is undefined
      )
  {

    // Fill out the device time with our local time and republish
    new_event_msg.device_clock_id = clock_name;
    new_event_msg.device_time = t;
    event_pub->publish(new_event_msg);
    return;
  }

  // Check for time request
  if (
         event_msg->local_clock_id == clock_name         // Event was sent by us
      && event_msg->local_request_time != ros::Time(0)   // Request time is defined
      && event_msg->device_time        != ros::Time(0)   // Device  time is defined
      && event_msg->local_receive_time == ros::Time(0)   // Recieve time is undefined
      )
  {

    // Fill out the local recive time field of the messge
    new_event_msg.local_receive_time = t;

    // Add event to the appropriate clock estimator
    if (sync_map.count(event_msg->device_clock_id) == 0 )
    {
      sync_map[event_msg->device_clock_id].reset(new TriggerSync(event_msg->device_clock_id,event_msg->local_clock_id));

      double switch_period;
      ros::NodeHandle local_nh("~");
      local_nh.param("switch_period", switch_period, 30.0);
      sync_map[event_msg->device_clock_id]->setSwitchPeriod(ros::Duration(30.0));
    }

    // Add the event to the TriggerSync estimator
    new_event_msg.corrected_local_time = sync_map[event_msg->device_clock_id]->updateTwoWay(
          new_event_msg.local_request_time,
          new_event_msg.device_time,
          new_event_msg.local_receive_time);


    // Publish the event with the
    event_pub->publish(new_event_msg);


    printf(" offset_lb = %+15.1fus , offset_ub = %+15.1fus , offset_estimate = %+15.1fus \r",
             (new_event_msg.local_request_time   - new_event_msg.device_time ).toSec()*1.0e6,
             (new_event_msg.local_receive_time   - new_event_msg.device_time ).toSec()*1.0e6,
             (new_event_msg.corrected_local_time - new_event_msg.device_time ).toSec()*1.0e6
             )      ;

    fflush(stdout);


  }
}


void timerCallback(const ros::TimerEvent& e)
{

  // Periodically publish an event message.
  trigger_sync::Event event_msg;
  event_msg.event_name = "";
  event_msg.local_clock_id = clock_name;
  event_msg.local_request_time = ros::Time::now();
  event_pub->publish(event_msg);
}


int main(int argc, char** argv)
{

  const char * val = ::getenv( "ROS_HOSTNAME");
  std::string hostname;
  if (val == 0)
  {
    hostname =  "ros_master";
  }
  else
  {
    hostname =  std::string(val);
    std::replace( hostname.begin(), hostname.end(), '.', '_');
  }

  ros::NodeHandle nh;

  ros::init(argc, argv, "local_clock_syncroniser_" + hostname );

  ros::NodeHandle local_nh("~");
  event_pub.reset(new ros::Publisher(nh.advertise<trigger_sync::Event>("event",1 )));

  const bool use_tcp = false;  // Use TCP or UDP. UDP seems to work better.
  if (use_tcp)
  {
      ros::Subscriber event_sub = nh.subscribe("/event",1, &event_msg_callback, ros::TransportHints().unreliable());
  } else {
      ros::Subscriber event_sub = nh.subscribe("/event",1, &event_msg_callback, ros::TransportHints().tcpNoDelay());
  }

  local_nh.param("clock_name", clock_name, std::string("clock"));
  clock_name = hostname + "_" +  clock_name;

  ROS_INFO_STREAM("clock name is: '" << clock_name << "'");

  // Setup timer for sendinding event messages
  ros::Timer timer = nh.createTimer(ros::Duration(0.2), timerCallback);
  while (ros::ok())
  {
    ros::spin();
  }
  return 0;
}
