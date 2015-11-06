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

#include "commons.h"
// default values for ROS params (if not specified by the user)
#define DEFAULT_MARKER_DIST_MIN 0
#define DEFAULT_MARKER_DIST_MAX 0.5
#define DEFAULT_MARKER_STEER_MIN -0.52
#define DEFAULT_MARKER_STEER_MAX 0.52

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
  ros::Subscriber matlab_poses_subscriber_;
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
  std::string matlab_poses_topic_name_;
  std::string marker_topic_name_;
  std::string sync_service_name_;

  std::string frame_map_;
  std::string frame_agent_prefix_;
  std::string frame_effective_prefix_;
  std::string frame_ellipse_suffix_;
  std::string frame_virtual_suffix_;
  std::string frame_ground_station_;
  std::string frame_target_ellipse_;

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

  void waitForSync() const;

  bool checkCollision(const int &id) const;
  int extractFirstID() const;

  agent_test::FormationStatistics statsVectorPhysicsToMsg(const std::vector<double> &vector) const;
  agent_test::FormationStatistics statsVectorToMsg(const std::vector<double> &vector) const;
  agent_test::FormationStatisticsStamped statsVectorToMsg(const std::string &frame, const int &id,
                                                          const std::vector <double> &vector) const;

  tf::Pose statsToPhysics(const agent_test::FormationStatistics &stats, double &a_x, double &a_y);
  tf::Pose statsToPhysics(const agent_test::FormationStatistics &stats, double &a_x, double &a_y, const double &theta_old);
  agent_test::FormationStatistics physicsToStats(const geometry_msgs::Pose &pose, const double &a_x, const double &a_y) const;
  void thetaCorrection(double &theta, const double &theta_old) const;

  double computeA(const double &diameter) const;
  double computeDiameter(const double &a) const;
  void updateSpanningEllipse(const agent_test::FormationStatisticsStamped &msg);
  visualization_msgs::Marker makeEllipse(const double &diameter_x, const double &diameter_y, const std::string &frame,
                                         const int &id) const;

  void interactiveMarkerInitialization();
  void interactiveMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void interactiveMarkerGuidance(const geometry_msgs::Pose &target, geometry_msgs::Pose &current);

  visualization_msgs::Marker makeBox(const double &scale) const;
  void makeBoxControl(visualization_msgs::InteractiveMarker &interactive_marker) const;
  void makeInteractiveMarkerAxis(const geometry_msgs::Pose &pose, const std::string &axis);
  void makeInteractiveMarkerPose(const geometry_msgs::Pose &pose);

  void updateTarget(const agent_test::FormationStatistics &target);
  void updateTargetStats(const agent_test::FormationStatistics &target);

  double saturation(const double &value, const double &min, const double &max) const;

  void console(const std::string &caller_name, std::stringstream &message, const int &log_level) const;

  void computeEffectiveEllipse(const std::string &frame_suffix);
  agent_test::FormationStatistics computeStatsFromPoses(const std::vector<geometry_msgs::Pose> &poses) const;

  void matlabPosesCallback(const geometry_msgs::Pose &pose);
};

#endif
