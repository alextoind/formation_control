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

#ifndef GUARD_GROUND_STATION_CORE_H
#define GUARD_GROUND_STATION_CORE_H

// Standard libraries
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <random>
#include <algorithm>
// ROS libraries
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
// Auto-generated from msg/ directory libraries
#include "agent_test/FormationStatistics.h"
#include "agent_test/FormationStatisticsStamped.h"
#include "agent_test/FormationStatisticsArray.h"
#include "agent_test/Sync.h"
// license info to be displayed at the beginning
#define LICENSE_INFO "\n*\n* Copyright (C) 2015 Alessandro Tondo\n* This program comes with ABSOLUTELY NO WARRANTY.\n* This is free software, and you are welcome to\n* redistribute it under GNU GPL v3.0 conditions.\n* (for details see <http://www.gnu.org/licenses/>).\n*\n\n"
// default values for ROS params (if not specified by the user)
#define DEFAULT_SAMPLE_TIME 0.25  // expressed in seconds
#define DEFAULT_TOPIC_QUEUE_LENGTH 1
#define DEFAULT_SHARED_STATS_TOPIC "shared_stats"
#define DEFAULT_RECEIVED_STATS_TOPIC "received_stats"
#define DEFAULT_TARGET_STATS_TOPIC "target_stats"
#define DEFAULT_SYNC_SERVICE "sync_agent"
#define DEFAULT_GROUND_STATION_FRAME "ground_station"
#define DEFAULT_NUMBER_OF_AGENTS 1
#define DEFAULT_SYNC_DELAY 5.0  // expressed in seconds


class GroundStationCore {
 public:
  GroundStationCore();
  ~GroundStationCore();

 private:
  ros::NodeHandle node_handle_;
  ros::NodeHandle *private_node_handle_;
  ros::Publisher target_stats_publisher_;
  ros::Publisher stats_publisher_;
  ros::Subscriber stats_subscriber_;
  ros::Timer algorithm_timer_;
  ros::ServiceServer sync_server_;
  ros::Time sync_time_;
  ros::Duration sync_delay_;
  double sample_time_;
  int number_of_agents_;

  int topic_queue_length_;
  std::string shared_stats_topic_name_;
  std::string received_stats_topic_name_;
  std::string target_stats_topic_name_;
  std::string sync_service_name_;
  std::string ground_station_frame_;

  agent_test::FormationStatistics target_statistics_;
  std::vector<agent_test::FormationStatisticsStamped> shared_statistics_grouped_;
  std::vector<int> connected_agents_;  // TODO add critical failure handler


  void algorithmCallback(const ros::TimerEvent &timer_event);
  void sharedStatsCallback(const agent_test::FormationStatisticsStamped &shared);
  bool syncAgentCallback(agent_test::Sync::Request &request, agent_test::Sync::Response &response);

  void waitForSync();

  bool checkCollision(const int &id);
  int extractFirstID();

  agent_test::FormationStatistics statsVectorToMsg(const std::vector<double> &vector);
  agent_test::FormationStatisticsStamped statsVectorToMsg(const std::string &frame, const std::vector <double> &vector);
};

#endif
