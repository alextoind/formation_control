/*  Copyright (C) 2015 Alessandro Tondo
 *  email: tondo.codes+ros <at> gmail.com
 *
 *  This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 *  License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any
 *  later version.
 *  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 *  details.
 *  You should have received a copy of the GNU General Public License along with this program.
 *  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef GUARD_PACKET_MANAGER_CORE_H
#define GUARD_PACKET_MANAGER_CORE_H

#include "commons.h"
#include "serial/serial.h"

extern "C" {
  #include "packet_agent.h"
  #include "packet_bs_telemetry.h"
  #include "packet_ids.h"
  #include "packet_manager.h"

  // forward declaration of 'private' functions defined in ./src/packet_manager.c
  short _pm_send_byte(unsigned char id, unsigned char sender, unsigned char receiver, char *sent);
  void _pm_process_byte(unsigned char ch);
}
void errorDeserialize(unsigned char header, unsigned char errno);
void errorSerialize(unsigned char header, unsigned char errno);
void newPacket(unsigned char header, unsigned char sender, unsigned char receiver);

// default values for ROS params (if not specified by the user)
#define DEFAULT_SERIAL_PORT "/dev/ttyACM0"
#define DEFAULT_SERIAL_BAUDRATE 115200
#define DEFAULT_SERIAL_TIMEOUT 5.0  // expressed in seconds
#define DEFAULT_BUFFER_LENGTH 256  // number of bytes per packet

// this parameter is just a hypothesis (see the final version of ./include/packet_manager/packet_manager.h)
#define PM_BASESTATION 0  // id of the base station (agent ids from 1 to 9)
#define PM_BROADCAST 255


class PacketManagerCore {
 public:
  PacketManagerCore();
  ~PacketManagerCore();

 private:
  ros::NodeHandle node_handle_;
  ros::NodeHandle *private_node_handle_;
  ros::Publisher stats_publisher_;
  ros::Subscriber stats_subscriber_;
  ros::Subscriber target_stats_subscriber_;
  ros::Publisher agent_poses_publisher_;
  ros::Timer algorithm_timer_;

  int topic_queue_length_;
  std::string shared_stats_topic_name_;
  std::string target_stats_topic_name_;
  std::string agent_poses_topic_name_;

  std::string frame_map_;
  std::string frame_agent_prefix_;
  std::string frame_virtual_suffix_;

  serial::Serial *serial_;
  double serial_timeout_;
  int buffer_length_;

  std::queue<Agent> packet_queue_;
  ros::Time time_last_packet_;
  std::vector<uint8_t> real_agents_;

  double sample_time_;
  int verbosity_level_;


  void targetStatsCallback(const formation_control::FormationStatisticsStamped &target);
  void receivedStatsCallback(const formation_control::FormationStatisticsStamped &received);
  void algorithmCallback(const ros::TimerEvent &timer_event);

  void serialReceivePacket();
  void serialSendPacket(const uint8_t &header, const uint8_t &sender, const uint8_t &receiver);

  void console(const std::string &caller_name, std::stringstream &message, const int &log_level) const;

  void processPackets();
  geometry_msgs::Pose setPose(const double &x, const double &y, const double &theta) const;
};

#endif
