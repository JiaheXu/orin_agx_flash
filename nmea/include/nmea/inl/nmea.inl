namespace drivers {
namespace epson {

/// @brief default constructor
template<typename T>
NmeaConnection<T>::NmeaConnection(const ros::NodeHandle& nh)
  : use_chrony_(false) {

  // set the rosparams
  set_rosparms(nh);

  // create the connection type
  create();
}

 /// @brief exposed send, hiding the connection type
template<typename T>
void NmeaConnection<T>::send(std::string msg) {
  // send the message over the connection
  _send_message(msg);
  
  /// TODO
  // if using chrony, send chrony message
  // if (use_chrony_) {
  //   struct timeval tv;
  //   tv.tv_sec = floor(computerTime);
  //   tv.tv_usec = fmod(computerTime, 1.0) * 1.0e6;
  //   chrony->Send(tv, computerSec - computerTime + chrony_latency);
  // }
  return;
}

/// @brief setup the rosparams
template<typename T>
void NmeaConnection<T>::set_rosparms(const ros::NodeHandle& nh) {
  // velodyne params
  nh.param<bool>("use_ethernet", use_ethernet_, "true");
  nh.param<std::string>("velodyne_ip", velodyne_ip_, "192.168.1.201");
  nh.param<int>("velodyne_port", velodyne_port_, 10110);
  // nmea params
  nh.param<int>("nmea_baudrate", nmea_baudrate_, 9600);
  nh.param<std::string>("nmea_port", nmea_port_, "/dev/ttyUSB1");
  /// chrony params
  nh.param<bool>("use_chrony", use_chrony_, "true");
  nh.param<std::string>("chrony_sock", chrony_sock_, "/var/run/chrony.sock");
  nh.param<double>("chrony_latency", chrony_latency_, 0.01);
}

// /////////////////////////////////////////////////////////////////////////////
/// @brief create the connection type
// /////////////////////////////////////////////////////////////////////////////

/// @brief udp connection
template <> void NmeaConnection<UDPSend>::create() {
  conn_ = new UDPSend(velodyne_ip_, velodyne_port_);
}
/// @brief serial connection
template <> void NmeaConnection<TimeoutSerial>::create() {
  conn_ = new TimeoutSerial(nmea_port_, nmea_baudrate_);
  conn_->setTimeout(boost::posix_time::seconds(5));  	
}

// /////////////////////////////////////////////////////////////////////////////
/// @brief send the nmea message over the connection
// /////////////////////////////////////////////////////////////////////////////

/// @brief over udp connection
template <> void NmeaConnection<UDPSend>::_send_message(std::string msg) {
  std::cout << "Sending a UDP connection in IP : " 
				    << velodyne_ip_ << " and on port: " << velodyne_port_ << std::endl;
  std::cout << "sending: " << msg << std::endl;
  conn_->send(msg);
}

/// @brief over serial connection
template <> void NmeaConnection<TimeoutSerial>::_send_message(std::string msg) {
  std::cout << "Sending a SERIAL connection in port : " 
				    << nmea_port_ << " and on baudrate: " << nmea_baudrate_ << std::endl;
  std::cout << "sending: " << msg << std::endl;          
  conn_->writeString(msg);
}


}; // namespace epson
}; // namespace drivers

