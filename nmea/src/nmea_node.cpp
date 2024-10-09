// /////////////////////////////////////////////////////////////////////////////
// nmea message publisher
/// TODO: the sync slowly drifts by miliseconds
/// TODO: does not recover when message publisher becomes very out-of-sync
///       -- if becomes out of sync, shorten sync wait time by amount of out-of-sync?
// /////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <chrono>
#include <thread>

#include <nmea/nmea.h>

using namespace drivers::epson;

// /////////////////////////////////////////////////////////////////////////////
// nmea message publisher entrypoint
// /////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv){
  ros::init(argc, argv, "nmea_node");
  ros::NodeHandle np("~");
  
  /// time interval
  int time_interval = 1;
  np.param<int>("time_interval", time_interval, 1);
  
  /// initialize the nmea connection
  typedef UDPSend conn_t;
  auto velodyne_conn = NmeaConnection<conn_t>(np);
  NmeaMessage nmea = NmeaMessage();
  
  /// setup the timer
  Timer timer = Timer();
  // reset before, to find difference in seconds
  timer.reset();
  // delay for initializing on the second
  bool delay_init = false;
  // delay for reducing the time spent on the cpu
  ros::Rate rate(100000.);

  /// forever loop
  while(ros::ok()) {
    
    /// initialize: delay until reached on second integral aligned
    if (delay_init == false) {
      std::cout << "\tinitializing " << std::endl;
      
      // print now time
      timer.print_time(std::chrono::system_clock::now());

      /// delay
      int delay = timer.delay(std::chrono::system_clock::now());
      std::this_thread::sleep_for(std::chrono::milliseconds(delay));
      
      // print now time
      // timer.print_time(system_clock::now());

      // reset timer now
      timer.reset();
      delay_init = true;
    }

    /// get the number of seconds
    double seconds = timer.seconds();

    if ( seconds > time_interval ) {
      // reset the timer right away to take into account the delay other operations
      timer.reset();

      // reset the timer of the time
      timer.print_time(std::chrono::system_clock::now());
      // std::cout << "seconds time is: " << timer.seconds() << std::endl;
      // std::cout << "\ttimer interval = " << seconds << std::endl;

      // setup the nmea message with the current time as the timestamp
      std::string msg = nmea.create(ros::Time::now().toSec());
      // send the message
      velodyne_conn.send(msg);
    }

    // sleep at rate
    rate.sleep();

  }
  return 0;
}
