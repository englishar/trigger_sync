/*
 * This is an example node that uses the trigger_sync timing library
 *
 * If the trigger comes from a clock source we use ticSync to estimate its frequency
 *
 * This node connects to a /dev/pps device (usually the DCD pin on a serial port) and outputs a /trigger message
 */

#include "timepps.h"
#include <ros/ros.h>
#include <fcntl.h>    // for c-style open and close
#include <trigger_sync/trigger_sync.h>
#include <trigger_sync/Event.h>

ros::Publisher pub_trigger;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pps_trigger");

  // get the pps path from config
  ros::NodeHandle local_nh("~");
  std::string pps_path;
  std::string trigger_topic;
  std::string trigger_id;
  std::string local_clock_id;
  std::string device_clock_id;
  const std::string default_pps_path = "/dev/pps0";
  bool publish_rising_edge;
  bool publish_falling_edge;
  bool measure_frequency_on_startup;
  double measure_frequency_on_startup_time;
  bool trigger_is_clock_source;

  bool use_user_level_timestamps;

  std::string rising_edge_event_id;
  std::string falling_edge_event_id;


  if (!local_nh.hasParam("pps_path"))
  {
    ROS_WARN_STREAM("No valid pps_path parameter set. Defaulting to  '" << default_pps_path << "'");
  } else {
    local_nh.param("pps_path", pps_path, default_pps_path);
    ROS_INFO_STREAM("Using serial port PPS device " << pps_path);
  }
  local_nh.param("pps_path", pps_path, default_pps_path);
  local_nh.param("trigger_topic" , trigger_topic ,std::string("/event"));
  local_nh.param("rising_edge_event_id", rising_edge_event_id, std::string("serial_trigger_rising"));
  local_nh.param("falling_edge_event_id", falling_edge_event_id, std::string("serial_trigger_falling"));
  local_nh.param("device_clock_id", device_clock_id, std::string("trigger_clock"));
  local_nh.param("local_clock_id", local_clock_id, std::string("local_clock"));
  local_nh.param("publish_rising_edge", publish_rising_edge, true);
  local_nh.param("publish_falling_edge", publish_falling_edge, false);
  local_nh.param("trigger_is_clock_source",trigger_is_clock_source, false);
  local_nh.param("use_user_level_timestamps",use_user_level_timestamps, false);

  ROS_INFO_STREAM("device_clock_id = " << device_clock_id);

  TriggerSync* sync_rising_edge_ptr;
  TriggerSync* sync_falling_edge_ptr;

  if (publish_rising_edge )
  {
    sync_rising_edge_ptr = new TriggerSync(device_clock_id,local_clock_id,rising_edge_event_id);
  } else
  {
    sync_rising_edge_ptr = new TriggerSync(device_clock_id,local_clock_id);   
    sync_rising_edge_ptr->setPublishEvents(false);
  }
//   sync_rising_edge_ptr->setMatchEvents(false);

  if (publish_falling_edge)
  {
    sync_falling_edge_ptr = new  TriggerSync(device_clock_id, local_clock_id,falling_edge_event_id);
  } else {
      sync_falling_edge_ptr = new  TriggerSync(device_clock_id, local_clock_id);
//      sync_falling_edge_ptr->setMatchEvents(false);
  }
//  sync_falling_edge_ptr ->setMatchEvents(false);

  // Advertise /trigger topic
  pub_trigger = local_nh.advertise<trigger_sync::Event>(trigger_topic, 10);

  // open the pps file
  int pps_file_descriptor = ::open(pps_path.c_str(), O_RDWR);
  if (pps_file_descriptor < 0)
  {
    ROS_ERROR_STREAM("Failed to open pps path with return code: " << pps_file_descriptor << " and error: " << strerror(errno) <<
                     ".\nEnsure serial port is mapped to to PPS device (e.g. run 'ldattach PPS /dev/ttyS0' to create /dev/pps0");
    return EXIT_FAILURE;
  }

  // open the pps
  pps_handle_t pps_handle;
  if (time_pps_create(pps_file_descriptor, &pps_handle) < 0)
  {
    ROS_ERROR_STREAM("Failed to create pps handle: " << strerror(errno));
    return EXIT_FAILURE;
  }
  ROS_DEBUG_STREAM("Opened pps handle on " << pps_path);

  // check the capabilities and settings of the pps
  int avaliable_modes;
   pps_params_t params;
  if (time_pps_getcap(pps_handle, &avaliable_modes) < 0 )
  {
    ROS_ERROR_STREAM("Failed to get avaliable modes from pps handle: " << strerror(errno));
  } else if ( time_pps_getparams(pps_handle, &params) < 0) {

  } else {
    ROS_INFO("\n"
             "\n                   PPS capabilities and enabled modes"
             "\n        Name                 Description                  Avaiable    Enabled"
             "\n  ==========================================================================="
             "\n   PPS_CAPTUREASSERT   Capture assert events:                 %s      %s"
             "\n   PPS_CAPTURECLEAR    Capture clear events:                  %s      %s"
             "\n   PPS_OFFSETASSERT    Apply compenstion for assert event     %s      %s"
             "\n   PPS_OFFSETCLEAR     Apply compensation for clear event     %s      %s"
             "\n   PPS_ECHOASSERT      Feed back assert event to output       %s      %s"
             "\n   PPS_ECHOCLEAR       Feed back clear event to output        %s      %s"
             "\n   PPS_CANWAIT         Able to  wait for an event             %s      %s\n",
             (avaliable_modes&PPS_CAPTUREASSERT) ?"true ":"false",  (params.mode&PPS_CAPTUREASSERT) ?"true ":"false",
             (avaliable_modes&PPS_CAPTURECLEAR)  ?"true ":"false",  (params.mode&PPS_CAPTURECLEAR)  ?"true ":"false",
             (avaliable_modes&PPS_OFFSETASSERT)  ?"true ":"false",  (params.mode&PPS_OFFSETASSERT)  ?"true ":"false",
             (avaliable_modes&PPS_OFFSETCLEAR)   ?"true ":"false",  (params.mode&PPS_OFFSETCLEAR)   ?"true ":"false",
             (avaliable_modes&PPS_ECHOASSERT)    ?"true ":"false",  (params.mode&PPS_ECHOASSERT)    ?"true ":"false",
             (avaliable_modes&PPS_ECHOCLEAR)     ?"true ":"false",  (params.mode&PPS_ECHOCLEAR)     ?"true ":"false",
             (avaliable_modes&PPS_CANWAIT)       ?"true ":"false",  (params.mode&PPS_CANWAIT)       ?"true ":"false"
                                                                                                     );
  }

  if ((avaliable_modes & PPS_CAPTUREASSERT) == 0 && publish_rising_edge == true)
  {
    ROS_ERROR_STREAM("cannot PPS_CAPTUREASSERT. This capability is required to capture rising edge");
    return EXIT_FAILURE;
  }

  if ((avaliable_modes & PPS_CAPTURECLEAR) == 0 && publish_falling_edge == true)
  {
    ROS_ERROR_STREAM("cannot PPS_CAPTURECLEAR. This capability is required to capture rising edge");
    return EXIT_FAILURE;
  }

  if ((avaliable_modes & PPS_CANWAIT) == 0)
  {
    ROS_ERROR_STREAM("cannot PPS_CANWAIT. This capability is required");
    return EXIT_FAILURE;
  }
  
  if ( !(params.mode & PPS_CAPTUREASSERT) && publish_rising_edge == true)
  {
      ROS_ERROR_STREAM("PPS_CAPTUREASSERT is required to caputre rising edges but is not enabled. Please run the included pps_config tool on " << pps_path  << " as root");
      return EXIT_FAILURE;
  }

  if ( !(params.mode & PPS_CAPTURECLEAR) && publish_falling_edge == true)
  {
      ROS_ERROR_STREAM("PPS_CAPTURECLEAR is required to capture falling edges bus is not enabled. Please run the included pps_config tool on " << pps_path  << " as root");
      return EXIT_FAILURE;
  }


  struct pps_fdata pps_data;
  pps_data.timeout.sec = 1;
  pps_data.timeout.nsec = 0;
  pps_data.timeout.flags = ~PPS_TIME_INVALID;


//  trigger_sync::Event trigger_msg;

//  ros::Time start_time;
  uint count = 0;
  bool first_time = true;
  bool finished_freq_estimate = false;

  printf("  Assert Seq.  Assert Time            Clear Seq      Clear Time\n");

  while (ros::ok())
  {
    if (::ioctl(pps_handle, PPS_FETCH, &pps_data) < 0)
    {
      ROS_ERROR_STREAM("pps_fetch failed with " << strerror(errno));
      continue;
    }


    ros::Time timestamp = ros::Time::now();

    static unsigned int prev_assert_sequence = pps_data.info.assert_sequence ;
    static unsigned int prev_clear_sequence  = pps_data.info.clear_sequence ;

    if (prev_assert_sequence != pps_data.info.assert_sequence && publish_rising_edge)
    {

      // Decide which local timestamp to use
      ros::Time local_time;
      if(use_user_level_timestamps)
      {
        // Use less accurate time (for debugging and experimental purposes)
        local_time = timestamp;
      }
      else
      {
        // Use more accurate time from PPS kernel (defualt option)
        local_time = ros::Time(pps_data.info.assert_tu.sec ,  pps_data.info.assert_tu.nsec);
      }

      // Add rising edge to the time syncroniser.
      ros::Time corrected_time;
      corrected_time = sync_rising_edge_ptr->update(
            ros::Time(pps_data.info.assert_sequence/10.0),                            // Device Time
            local_time,                                                               // Local Time
            rising_edge_event_id                                                      // Event ID
      );

//      trigger_msg.event_name = trigger_id + "_rising";
//      trigger_msg.send_clock = trigger_id;
//      trigger_msg.send_time = ros::Time((double)pps_data.info.assert_sequence / nominal_frequency);
//      trigger_msg.receive_clock = local_clock_id;
//      trigger_msg.receive_time = ros::Time(pps_data.info.assert_tu.sec , pps_data.info.assert_tu.nsec);
//      pub_trigger.publish(trigger_msg);

      if( pps_data.info.assert_sequence - prev_assert_sequence > 1) {
        printf("\n");
        ROS_WARN("Missed assert");
//        exit(1);
      }
      prev_assert_sequence = pps_data.info.assert_sequence;
    }


    if (prev_clear_sequence != pps_data.info.clear_sequence && publish_falling_edge)
    {

      // Decide which local timestamp to use
      ros::Time local_time;
      if(use_user_level_timestamps)
      {
        // Use less accurate time (for debugging and experimental purposes)
        local_time = timestamp;
      }
      else
      {
        // Use more accurate time from PPS kernel (defualt option)
        local_time = ros::Time(pps_data.info.clear_tu.sec , pps_data.info.clear_tu.nsec);    // Local Time
      }

      if (trigger_is_clock_source) {
        sync_falling_edge_ptr->update(
              ros::Time((double)pps_data.info.clear_sequence/10.0),                   // Device Time
              local_time,                                                             // Local Time
              falling_edge_event_id);                                                 // Event ID
      }
      else
      {
        sync_falling_edge_ptr->update(
              TriggerSync::UNKNOWN_TIME,                                                 // Device Time
              local_time,                                                             // Local Time
              falling_edge_event_id);                                                 // Event ID
      }


      if( pps_data.info.clear_sequence - prev_clear_sequence > 1) {
        ROS_WARN("Missed clear %d" ,pps_data.info.clear_sequence - prev_clear_sequence );
      }
      prev_clear_sequence = pps_data.info.clear_sequence;

    }

    // Print times and sequence numbers.
    printf(   "\r   %d    %lld.%09d       %d    %lld.%09d ",
            pps_data.info.assert_sequence,
            pps_data.info.assert_tu.sec, pps_data.info.assert_tu.nsec,
            pps_data.info.clear_sequence,
            pps_data.info.clear_tu.sec, pps_data.info.clear_tu.nsec);


     if(sync_rising_edge_ptr->isStable() )
     {
       double skew = sync_rising_edge_ptr->skew();
       double freq = 1/(skew+1);
       double ppm  = 1e6*fabs(round(freq)/freq-1);
       printf(" Rising: %10.4fHz(%5.1f ppm) ",  freq, ppm);
     } else {
       printf(" Rising: --");
     }


     if(sync_falling_edge_ptr->isStable() )
     {
       double skew = sync_falling_edge_ptr->skew();
       double freq = 1/(skew+1);
       double ppm  = 1e6*fabs(round(freq)/freq-1);
       printf(" Falling %10.4fHz(%5.1f ppm) ",  freq, ppm);
     }
     else
     {
       printf(" Falling --");
     }

     fflush(stdout);


//    prev_assert_sequence = pps_data.info.assert_sequence;
//    prev_clear_sequence =  pps_data.info.clear_sequence;


  }

  time_pps_destroy(pps_handle);
  return EXIT_SUCCESS;
}







