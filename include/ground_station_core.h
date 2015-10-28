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
#include <angles/angles.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
// Auto-generated from msg/ directory libraries
#include "agent_test/FormationStatistics.h"
#include "agent_test/FormationStatisticsStamped.h"
#include "agent_test/FormationStatisticsArray.h"
#include "agent_test/Sync.h"
// license info to be displayed at the beginning
#define LICENSE_INFO "\n*\n* Copyright (C) 2015 Alessandro Tondo\n* This program comes with ABSOLUTELY NO WARRANTY.\n* This is free software, and you are welcome to\n* redistribute it under GNU GPL v3.0 conditions.\n* (for details see <http://www.gnu.org/licenses/>).\n*\n\n"
// default values for ROS params (if not specified by the user)
#define DEFAULT_VERBOSITY_LEVEL 1
#define DEFAULT_SAMPLE_TIME 0.1  // expressed in seconds
#define DEFAULT_TOPIC_QUEUE_LENGTH 1
#define DEFAULT_SHARED_STATS_TOPIC "shared_stats"
#define DEFAULT_RECEIVED_STATS_TOPIC "received_stats"
#define DEFAULT_TARGET_STATS_TOPIC "target_stats"
#define DEFAULT_SYNC_SERVICE "sync_agent"
#define DEFAULT_MARKER_TOPIC "visualization_marker"
#define DEFAULT_GROUND_STATION_FRAME "ground_station"
#define DEFAULT_FIXED_FRAME "map"
#define DEFAULT_TARGET_FRAME "target_stats_ellipse"
#define DEFAULT_NUMBER_OF_AGENTS 1
#define DEFAULT_MARKER_DIST_MIN 0
#define DEFAULT_MARKER_DIST_MAX 0.5
#define DEFAULT_MARKER_STEER_MIN -0.52
#define DEFAULT_MARKER_STEER_MAX 0.52
#define DEFAULT_SYNC_DELAY 5.0  // expressed in seconds

#define FATAL -3
#define ERROR -2
#define WARN -1
#define INFO 0
#define DEBUG 1
#define DEBUG_V 2
#define DEBUG_VV 3
#define DEBUG_VVV 4
#define DEBUG_VVVV 5

// TODO: add critical failure handler


class GroundStationCore {
 public:
  GroundStationCore();
  ~GroundStationCore();

 private:
  ros::NodeHandle node_handle_;
  ros::NodeHandle *private_node_handle_;
  ros::Publisher target_stats_publisher_;
  ros::Publisher stats_publisher_;
  ros::Publisher marker_publisher_;
  ros::Subscriber stats_subscriber_;
  ros::Timer algorithm_timer_;
  ros::ServiceServer sync_server_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  interactive_markers::InteractiveMarkerServer *interactive_marker_server_;

  ros::Time sync_time_;
  ros::Duration sync_delay_;
  double sample_time_;
  int number_of_agents_;

  geometry_msgs::Pose target_pose_;
  double target_a_x_;
  double target_a_y_;

  int topic_queue_length_;
  std::string shared_stats_topic_name_;
  std::string received_stats_topic_name_;
  std::string target_stats_topic_name_;
  std::string sync_service_name_;
  std::string marker_topic_name_;
  std::string ground_station_frame_;
  std::string fixed_frame_;
  std::string target_frame_;

  agent_test::FormationStatisticsStamped target_statistics_;
  std::vector<agent_test::FormationStatisticsStamped> shared_statistics_grouped_;
  std::vector<int> connected_agents_;

  double marker_dist_min_;
  double marker_dist_max_;
  double marker_steer_min_;
  double marker_steer_max_;

  int verbosity_level_;

  void algorithmCallback(const ros::TimerEvent &timer_event);
  void sharedStatsCallback(const agent_test::FormationStatisticsStamped &shared);
  bool syncAgentCallback(agent_test::Sync::Request &request, agent_test::Sync::Response &response);

  void waitForSync();

  bool checkCollision(const int &id);
  int extractFirstID();

  agent_test::FormationStatistics statsVectorPhysicsToMsg(const std::vector<double> &vector);
  agent_test::FormationStatistics statsVectorToMsg(const std::vector<double> &vector);
  agent_test::FormationStatisticsStamped statsVectorToMsg(const std::string &frame, const int &id,
                                                          const std::vector <double> &vector);

  tf::Pose statsToPhysics(const agent_test::FormationStatistics &stats, double &a_x, double &a_y);
  tf::Pose statsToPhysics(const agent_test::FormationStatistics &stats, double &a_x, double &a_y, const double &theta_old);
  agent_test::FormationStatistics physicsToStats(const geometry_msgs::Pose &pose, const double &a_x, const double &a_y);
  void thetaCorrection(double &theta, const double &theta_old);

  double computeA(const double &diameter) const;
  double computeDiameter(const double &a) const;
  void updateSpanningEllipse(const agent_test::FormationStatisticsStamped &msg);
  visualization_msgs::Marker makeEllipse(const double &diameter_x, const double &diameter_y, const std::string &frame,
                                         const int &id);

  void interactiveMarkerInitialization();
  void interactiveMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void interactiveMarkerGuidance(const geometry_msgs::Pose &target, geometry_msgs::Pose &current);

  visualization_msgs::Marker makeBox(const double &scale);
  void makeBoxControl(visualization_msgs::InteractiveMarker &interactive_marker);
  void makeInteractiveMarkerAxis(const geometry_msgs::Pose &pose, const std::string &axis);
  void makeInteractiveMarkerPose(const geometry_msgs::Pose &pose);

  void updateTarget(const agent_test::FormationStatistics &target);
  void updateTargetStats(const agent_test::FormationStatistics &target);

  double saturation(const double &value, const double &min, const double &max);

  void console(const std::string &caller_name, std::stringstream &message, const int &log_level);

  void computeEffectiveEllipse(const std::string &frame_suffix);
  agent_test::FormationStatistics computeStatsFromPoses(const std::vector<geometry_msgs::Pose> &poses);
};

#endif
