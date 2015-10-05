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

#ifndef GUARD_AGENT_CORE_H
#define GUARD_AGENT_CORE_H

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
#include <agent_test/Sync.h>
// default values for ROS params (if not specified by the user)
#define DEFAULT_NUMBER_OF_STATS 5  // see FormationStatistics.msg (mx, my, mxx, mxy, myy)
#define DEFAULT_NUMBER_OF_VELOCITIES 2  // virtual planar linear twist (virtual_x_dot, virutal_y_dot)
#define DEFAULT_AGENT_ID 0  // if not setted by the user, the Ground Station will choose an unique value
#define DEFAULT_SAMPLE_TIME 0.25  // expressed in seconds
#define DEFAULT_VELOCITY_VIRTUAL_THRESHOLD 4.0  // expressed in meters/second
#define DEFAULT_LOS_DISTANCE_THRESHOLD 4.0
#define DEFAULT_SPEED_MIN 0.0  // expressed in meters/second
#define DEFAULT_SPEED_MAX 4.0  // expressed in meters/second
#define DEFAULT_STEER_MIN -0.52  // expressed in radiants
#define DEFAULT_STEER_MAX 0.52  // expressed in radiants
#define DEFAULT_K_P_SPEED 15.0
#define DEFAULT_K_I_SPEED 0.05
#define DEFAULT_K_P_STEER 0.5
#define DEFAULT_VEHICLE_LENGTH 0.4  // expressed in meters
#define DEFAULT_WORLD_LIMIT 5.0  // in meters, considering a "square world"
#define DEFAULT_TOPIC_QUEUE_LENGTH 1
#define DEFAULT_SHARED_STATS_TOPIC "shared_stats"
#define DEFAULT_RECEIVED_STATS_TOPIC "received_stats"
#define DEFAULT_TARGET_STATS_TOPIC "target_stats"
#define DEFAULT_SYNC_SERVICE_NAME "sync_agent"
#define DEFAULT_SYNC_TIMEOUT 10.0  // expressed in seconds

class AgentCore {
 public:
  AgentCore();
  ~AgentCore();

 private:
  ros::NodeHandle *private_node_handle_;
  ros::Publisher stats_publisher_;
  ros::Subscriber stats_subscriber_;
  ros::Subscriber target_stats_subscriber_;
  ros::Timer algorithm_timer_;
  ros::ServiceClient sync_client_;

  int topic_queue_length_;
  std::string shared_stats_topic_name_;
  std::string received_stats_topic_name_;
  std::string target_stats_topic_name_;
  std::string sync_service_name_;
  ros::Duration sync_timeout_;
  double sample_time_;

  int agent_id_;  // it must be set with a unique value among all agents
  std::vector<int> neighbours_; // agents connected through an arbitrary path (i.e. in the spanning tree)
  geometry_msgs::Pose pose_;
  geometry_msgs::Pose pose_virtual_;
  geometry_msgs::Twist twist_;
  geometry_msgs::Twist twist_virtual_;
  agent_test::FormationStatistics target_statistics_;
  agent_test::FormationStatistics estimated_statistics_;
  std::vector<agent_test::FormationStatistics> received_statistics_;  // initialized with Ground Station first msg

  int number_of_stats_;
  int number_of_velocities_;
  // consensus
  Eigen::RowVectorXd phi_dot_;
  // control law
  Eigen::MatrixXd gamma_;
  Eigen::MatrixXd lambda_;
  Eigen::MatrixXd b_;
  Eigen::MatrixXd jacob_phi_;

  double velocity_virtual_threshold_;
  double los_distance_threshold_;
  double speed_min_;
  double speed_max_;
  double steer_min_;
  double steer_max_;

  double los_distance_;
  double los_angle_;
  double speed_error_;
  double speed_integral_;
  double speed_command_sat_;
  double steer_command_sat_;
  double k_p_speed_;
  double k_i_speed_;
  double k_p_steer_; // Ackermann

  double vehicle_length_;
  double world_limit_;


  void targetStatsCallback(const agent_test::FormationStatistics &target);
  void receivedStatsCallback(const agent_test::FormationStatisticsArray &received);
  void algorithmCallback(const ros::TimerEvent &timer_event);

  void dynamics();
  void consensus();
  void control();
  void guidance();

  Eigen::VectorXd statsMsgToVector(const agent_test::FormationStatistics &msg);
  Eigen::MatrixXd statsMsgToMatrix(const std::vector <agent_test::FormationStatistics> &msg);
  agent_test::FormationStatistics statsVectorToMsg(const Eigen::VectorXd &vector);
  agent_test::FormationStatistics statsVectorToMsg(const std::vector <double> &vector);

  double integrator(const double &out_old, const double &in_old, const double &in_new, const double &k);
  double saturation(const double &value, const double &min, const double &max);

  double getTheta(const geometry_msgs::Quaternion &quat);
  void setTheta(geometry_msgs::Quaternion &quat, const double &theta);

  void waitForSyncTime();
};

#endif
