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
// ROS libraries
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
// Auto-generated from msg/ directory libraries
#include "agent_test/FormationStatistics.h"
#include "agent_test/FormationStatisticsStamped.h"
// license info to be displayed at the beginning
#define LICENSE_INFO "\n*\n* Copyright (C) 2015 Alessandro Tondo\n* This program comes with ABSOLUTELY NO WARRANTY.\n* This is free software, and you are welcome to\n* redistribute it under GNU GPL v3.0 conditions.\n* (for details see <http://www.gnu.org/licenses/>).\n*\n\n"
// default values for ROS params (if not specified by the user)
#define DEFAULT_NUMBER_OF_STATS 5  // see FormationStatistics.msg (mx, my, mxx, mxy, myy)
#define DEFAULT_NUMBER_OF_VELOCITIES 2  // virtual planar linear twist (virtual_x_dot, virutal_y_dot)
#define DEFAULT_SAMPLE_TIME 0.010  // expressed in seconds
#define DEFAULT_VELOCITY_VIRTUAL_THRESHOLD 4

class AgentCore {
 public:
  AgentCore();
  ~AgentCore();

 private:
  ros::NodeHandle *private_node_handle_;
  ros::NodeHandle node_handle_;

  int agent_id_;
  geometry_msgs::Pose pose_;
  geometry_msgs::Pose pose_virtual_;
  geometry_msgs::Twist twist_;
  geometry_msgs::Twist twist_virtual_;
  agent_test::FormationStatistics target_statistics_;
  agent_test::FormationStatistics estimated_statistics_;
  std::vector<agent_test::FormationStatistics> received_estimated_statistics_;

  double sample_time_;
  int number_of_stats_;
  int number_of_velocities_;
  // consensus
  Eigen::RowVectorXd phi_dot_;
  // control law
  Eigen::MatrixXd gamma_;
  Eigen::MatrixXd lambda_;
  Eigen::MatrixXd b_;
  Eigen::MatrixXd jacob_phi_;
  // saturations
  double velocity_virtual_threshold_;


  void dynamics();
  void consensus();
  void control();
  void guidance();

  Eigen::VectorXd statsMsgToVector(const agent_test::FormationStatistics &msg);
  Eigen::MatrixXd statsMsgToMatrix(const std::vector <agent_test::FormationStatistics> &msg);
  agent_test::FormationStatistics statsVectorToMsg(const Eigen::VectorXd &vector);

  double integrator(const double &x_old, const double &x_dot_old, const double &x_dot_new);

};

#endif
