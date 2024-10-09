////////////////////////////////////////////////////////////////////////
/// @file nmea connection & message
/// @brief Handles storing nmea message to send to velodyne
////////////////////////////////////////////////////////////////////////
#pragma once

#include <memory>
#include <ros/ros.h>
#include <string>
#include <iostream>

#include <nmea/utils.h>
#include <nmea/chrony.h>
#include <nmea/udp.h>
#include <nmea/timeoutserial.h>

namespace drivers {
namespace epson {

// /////////////////////////////////////////////////////////////////////////////
/// @brief the connection to send the nmea message to the velodyne  
// /////////////////////////////////////////////////////////////////////////////
template<typename conn_t >
class NmeaConnection {
public:
  
  /// @brief me ptr
  typedef std::shared_ptr<NmeaConnection> Ptr;

  /// @brief me ptr creator
  static Ptr MakeShared() { return Ptr(new NmeaConnection()); }

  /// @brief default constructor
  NmeaConnection(const ros::NodeHandle& nh);

  /// @brief default destructor
  ~NmeaConnection() {} ;

  /// @brief exposed send, hiding the connection type
  void send(std::string msg);

private:

  /// @brief create the connection type
  void create();

  /// @brief create the connection type
  void _send_message(std::string msg);
  
  /// @brief setup the rosparams
  void set_rosparms(const ros::NodeHandle& nh);

  /// @brief connection type
  conn_t * conn_;

  /// == chrony setup ==
  Chrony * chrony_;
  bool use_chrony_;
  std::string chrony_sock_;
  double chrony_latency_; 

  /// == veldoyne setup ==
  
  // use ethernet to connect to velodyne
  bool use_ethernet_;
  std::string velodyne_ip_;
  int velodyne_port_;

  /// == nmea setup ==
  int nmea_baudrate_;
  std::string nmea_port_;
};

// /////////////////////////////////////////////////////////////////////////////
/// @brief the NMEA message to send to the velodyne
// /////////////////////////////////////////////////////////////////////////////
class NmeaMessage {
public:

  /// @brief me ptr
  typedef std::shared_ptr<NmeaMessage> Ptr;

  /// @brief me ptr creator
  static Ptr MakeShared() { return Ptr(new NmeaMessage()); }

  /// @brief default constructor
  NmeaMessage() { tb_ = TimeBookkeeper::MakeShared(); }

  /// @brief default deconstructor
  ~NmeaMessage() {}

  /// @brief craete a time string to send as
  std::string create(int computer_sec) {
    // update the bookkeeper times
    tb_->updateTime(computer_sec);

    // clear the string buffers
    memset(buffer_, 0, sizeof(char));
    memset(nmea_, 0, sizeof(char));

    // setup the checksum to append to the nmea string
    char checksum = 0;

    // create the nmea string message (raw char *), without checksum
    int length = snprintf (
      buffer_,
      70,
      "GPRMC,%02i%02i%02i,A,0000,00,N,00000,00,W,000.0,000.0,%02i0115,000.0,E",
      tb_->get_hour(),
      tb_->get_minute(),
      tb_->get_second(),
      tb_->get_day()
    );

    // create the checksum
    for (int i = 0; i < length; i++) { checksum = char(checksum^buffer_[i]); }
    /// create the nmea string message (raw char *), with checksum
    int length2 = snprintf (nmea_, 70, "$%s*%02x\r\n", buffer_, checksum);
    
    return std::string(nmea_);
  }

private:
  /// handles setting different types of time units
  TimeBookkeeper::Ptr tb_;

  /// nmea message character buffers
  char buffer_[70];
  char nmea_[70];
};


}; // namespace epson
}; // namespace drivers

/// include the templated implementations
#include <nmea/inl/nmea.inl>
