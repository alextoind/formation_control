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

#include "commons.h"
// default values for ROS params (if not specified by the user)
#define DEFAULT_AGENT_ID 0  // if not set by the user, the Ground Station will choose an unique value
#define DEFAULT_VELOCITY_VIRTUAL_THRESHOLD 4.0  // expressed in meters/second
#define DEFAULT_SPEED_MIN 0.0  // expressed in meters/second
#define DEFAULT_SPEED_MAX 4.0  // expressed in meters/second
#define DEFAULT_STEER_MIN -0.25  // expressed in radians
#define DEFAULT_STEER_MAX 0.25  // expressed in radians
#define DEFAULT_K_P_SPEED 0.5
#define DEFAULT_K_P_STEER 2.0
#define DEFAULT_VEHICLE_LENGTH 0.4  // expressed in meters
#define DEFAULT_WORLD_LIMIT 1.0  // in meters, considering a "square world" (only for random pose generation)
#define DEFAULT_MARKER_PATH_LIFETIME 30  // expressed in seconds


class AgentCore {
 public:
  AgentCore();
  ~AgentCore();

 private:
  ros::NodeHandle node_handle_;
  ros::NodeHandle *private_node_handle_;
  ros::Publisher stats_publisher_;
  ros::Publisher marker_publisher_;
  ros::Subscriber stats_subscriber_;
  ros::Subscriber target_stats_subscriber_;
  ros::Timer algorithm_timer_;
  tf::TransformBroadcaster tf_broadcaster_;

  bool enable_path_;
  int marker_path_id_;
  int marker_path_lifetime_;
  std::string frame_map_;
  std::string frame_agent_prefix_;
  std::string frame_virtual_suffix_;
  std::string agent_frame_;
  std::string agent_virtual_frame_;

  int topic_queue_length_;
  std::string shared_stats_topic_name_;
  std::string target_stats_topic_name_;
  std::string marker_topic_name_;
  double sample_time_;
  double slot_tdma_;

  int agent_id_;  // it must be set with a unique value among all agents
  std::vector<int> neighbours_; // agents connected through an arbitrary path (i.e. in the spanning tree)
  geometry_msgs::Pose pose_;
  geometry_msgs::Pose pose_virtual_;
  geometry_msgs::Twist twist_;
  geometry_msgs::Twist twist_virtual_;
  formation_control::FormationStatistics target_statistics_;
  formation_control::FormationStatistics estimated_statistics_;
  std::map<int, formation_control::FormationStatistics> received_statistics_;
  std::mutex received_statistics_mutex_;

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
  double speed_min_;
  double speed_max_;
  double steer_min_;
  double steer_max_;

  double speed_command_sat_;
  double steer_command_sat_;
  double k_p_speed_;
  double k_p_steer_;

  double vehicle_length_;
  double world_limit_;

  int verbosity_level_;


  void targetStatsCallback(const formation_control::FormationStatisticsStamped &target);
  void receivedStatsCallback(const formation_control::FormationStatisticsStamped &received);
  void algorithmCallback(const ros::TimerEvent &timer_event);
  void waitForSlotTDMA(const double &deadline) const;

  void dynamics();
  void consensus();
  void control();
  void guidance();

  Eigen::VectorXd statsMsgToVector(const formation_control::FormationStatistics &msg) const;
  Eigen::MatrixXd statsMsgToMatrix(const std::map<int, formation_control::FormationStatistics> &msg) const;
  formation_control::FormationStatistics statsVectorToMsg(const Eigen::VectorXd &vector) const;
  formation_control::FormationStatistics statsVectorToMsg(const std::vector<double> &vector) const;

  void floor(double &d, const int &precision) const;
  double integrator(const double &out_old, const double &in_old, const double &in_new, const double &k) const;
  double saturation(const double &value, const double &min, const double &max) const;

  Eigen::Vector3d getRPY(const geometry_msgs::Quaternion &quat) const;
  double getTheta(const geometry_msgs::Quaternion &quat) const;
  void setTheta(geometry_msgs::Quaternion &quat, const double &theta) const;

  void broadcastPath(const geometry_msgs::Pose &pose_new, const geometry_msgs::Pose &pose_old, const std::string &frame);
  void broadcastPose(const geometry_msgs::Pose &pose, const std::string &frame);
  visualization_msgs::Marker addToMarkerPath(const geometry_msgs::Point &p_old, const geometry_msgs::Point &p_new,
                                             const std::string &frame);

  void console(const std::string &caller_name, std::stringstream &message, const int &log_level) const;
};

#endif
