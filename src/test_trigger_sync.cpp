#include "ros/ros.h"
#include "trigger_sync/Event.h"
#include "trigger_sync/trigger_sync.h"

TriggerSync* s;

void triggerMsgCallback(trigger_sync::EventConstPtr trigger_msg){
  ROS_INFO_STREAM("Got a message in test node: ");

  if(trigger_msg->event_name == "serial_port_trigger_line_rising"){
    ros::Time learned_time = s->update(trigger_msg->device_time, trigger_msg->local_receive_time);
    ROS_INFO_STREAM("Learned time: " << learned_time);
  }


}


int main (int argc, char** argv ){



    ros::init(argc, argv, "test_time_sync");

    ros::NodeHandle nh;

    TICSync::LinearTwoWayClockSync<double> s;


    (s.update(0,1,2));
    (s.update(1,2,3));
    (s.update(2,3,4));
    (s.update(3,4,5));
    (s.update(4,4.1,5));
    (s.update(-1,4.6,2));



    exit(1);
//    s= new TimeSync("trigger_assert_seq","local_clock","serial_DCD_falling");

//    ros::Subscriber trigger_sub = nh.subscribe<time_sync::Event>("/event",10, &triggerMsgCallback);


//    TimeSync s(10);




//    s.update(ros::Time(55),ros::Time(55),ros::Time(55));
//    s.update( 0 , 0 , 0);


    ros::spin();

}
