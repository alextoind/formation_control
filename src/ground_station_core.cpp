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

#include "ground_station_core.h"

GroundStationCore::GroundStationCore() {
  // handles server private parameters (private names are protected from accidental name collisions)
  private_node_handle_ = new ros::NodeHandle("~");

  private_node_handle_->param("verbosity_level", verbosity_level_, DEFAULT_VERBOSITY_LEVEL);
  private_node_handle_->param("sample_time", sample_time_, (double)DEFAULT_SAMPLE_TIME);
  private_node_handle_->param("topic_queue_length", topic_queue_length_, DEFAULT_TOPIC_QUEUE_LENGTH);
  private_node_handle_->param("shared_stats_topic", shared_stats_topic_name_, std::string(DEFAULT_SHARED_STATS_TOPIC));
  private_node_handle_->param("received_stats_topic", received_stats_topic_name_, std::string(DEFAULT_RECEIVED_STATS_TOPIC));
  private_node_handle_->param("target_stats_topic", target_stats_topic_name_, std::string(DEFAULT_TARGET_STATS_TOPIC));
  private_node_handle_->param("matlab_poses_topic", matlab_poses_topic_name_, std::string(DEFAULT_MATLAB_POSES_TOPIC));
  private_node_handle_->param("sync_service", sync_service_name_, std::string(DEFAULT_SYNC_SERVICE));
  private_node_handle_->param("marker_topic", marker_topic_name_, std::string(DEFAULT_MARKER_TOPIC));
  private_node_handle_->param("ground_station_frame", ground_station_frame_, std::string(DEFAULT_GROUND_STATION_FRAME));
  private_node_handle_->param("fixed_frame", fixed_frame_, std::string(DEFAULT_FIXED_FRAME));
  private_node_handle_->param("target_frame", target_frame_, std::string(DEFAULT_TARGET_FRAME));
  private_node_handle_->param("number_of_agents", number_of_agents_, DEFAULT_NUMBER_OF_AGENTS);
  private_node_handle_->param("marker_dist_min", marker_dist_min_, (double)DEFAULT_MARKER_DIST_MIN);
  private_node_handle_->param("marker_dist_max", marker_dist_max_, (double)DEFAULT_MARKER_DIST_MAX);
  private_node_handle_->param("marker_steer_min", marker_steer_min_, (double)DEFAULT_MARKER_STEER_MIN);
  private_node_handle_->param("marker_steer_max", marker_steer_max_, (double)DEFAULT_MARKER_STEER_MAX);
  double sync_delay;
  private_node_handle_->param("sync_delay", sync_delay, (double)DEFAULT_SYNC_DELAY);
  sync_delay_ = ros::Duration(sync_delay);

  std::vector<double> target_values;
  const std::vector<double> DEFAULT_TARGET_STATS = {0, 0, 1, 0, 1};
  private_node_handle_->param("target_statistics", target_values, DEFAULT_TARGET_STATS);
  target_statistics_.header.frame_id = "target_stats";

  bool target_from_physics;
  private_node_handle_->param("target_from_physics", target_from_physics, false);
  agent_test::FormationStatistics target_statistics;
  if (target_from_physics) {  // data in target_values is (x, y, theta, diam_x, diam_y)
    target_statistics = statsVectorPhysicsToMsg(target_values);
  }
  else {  // data in target_values is (m_x, m_y, m_xx, m_xy, m_yy)
    target_statistics = statsVectorToMsg(target_values);
  }

  marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>(marker_topic_name_, topic_queue_length_);
  target_stats_publisher_ = node_handle_.advertise<agent_test::FormationStatisticsStamped>(target_stats_topic_name_, topic_queue_length_);
  stats_publisher_ = node_handle_.advertise<agent_test::FormationStatisticsArray>(received_stats_topic_name_, topic_queue_length_);
  stats_subscriber_ = node_handle_.subscribe(shared_stats_topic_name_, number_of_agents_, &GroundStationCore::sharedStatsCallback, this);
  matlab_poses_subscriber_ = node_handle_.subscribe(matlab_poses_topic_name_, 2, &GroundStationCore::matlabPosesCallback, this);
  sync_server_ = node_handle_.advertiseService(sync_service_name_, &GroundStationCore::syncAgentCallback, this);
  interactive_marker_server_ = new interactive_markers::InteractiveMarkerServer("interactive_markers");

  waitForSync();
  algorithm_timer_ = private_node_handle_->createTimer(ros::Duration(sample_time_), &GroundStationCore::algorithmCallback, this);
  number_of_agents_ = connected_agents_.size();

  updateTarget(target_statistics);
  interactiveMarkerInitialization();
}

GroundStationCore::~GroundStationCore() {
  delete interactive_marker_server_;
  delete private_node_handle_;
}

void GroundStationCore::algorithmCallback(const ros::TimerEvent &timer_event) {
  agent_test::FormationStatisticsArray msg;
  msg.header.frame_id = ground_station_frame_;
  msg.header.stamp = ros::Time::now();
  msg.neighbours_ = connected_agents_;
  msg.vector = shared_statistics_grouped_;
  stats_publisher_.publish(msg);

  std::stringstream s;
  s << "Message published.";
  console(__func__, s, DEBUG);

  computeEffectiveEllipse("");
  computeEffectiveEllipse("_virtual");
}

bool GroundStationCore::checkCollision(const int &id) {
  return std::find(std::begin(connected_agents_), std::end(connected_agents_), id) != std::end(connected_agents_);
}

double GroundStationCore::computeA(const double &diameter) const {
  return std::pow(diameter, 2) / (4* number_of_agents_);
}

double GroundStationCore::computeDiameter(const double &a) const {
  return 2*std::sqrt(number_of_agents_*std::abs(a));
}

void GroundStationCore::computeEffectiveEllipse(const std::string &frame_suffix) {
  std::vector<geometry_msgs::Pose> agent_poses;
  for (auto const &id : connected_agents_) {
    std::string frame = "agent_" + std::to_string(id) + frame_suffix;
    if (tf_listener_.canTransform(fixed_frame_, frame, ros::Time(0))) {
      tf::StampedTransform pose_tf;
      geometry_msgs::Pose pose_msg;
      tf_listener_.lookupTransform(fixed_frame_, frame, ros::Time(0), pose_tf);
      tf::poseTFToMsg(pose_tf, pose_msg);
      agent_poses.push_back(pose_msg);
    }
  }
  
  agent_test::FormationStatisticsStamped effective_statistics;
  effective_statistics.header.frame_id = "effective_stats" + frame_suffix;
  effective_statistics.header.stamp = ros::Time::now();
  effective_statistics.stats = computeStatsFromPoses(agent_poses);
  updateSpanningEllipse(effective_statistics);
}

agent_test::FormationStatistics GroundStationCore::computeStatsFromPoses(const std::vector<geometry_msgs::Pose> &poses) {
  agent_test::FormationStatistics stats;

  for (auto const &pose : poses) {
    stats.m_x += pose.position.x / poses.size();
    stats.m_y += pose.position.y / poses.size();
    stats.m_xx += std::pow(pose.position.x, 2) / poses.size();
    stats.m_xy += pose.position.x * pose.position.y / poses.size();
    stats.m_yy += std::pow(pose.position.y, 2) / poses.size();
  }

  return stats;
}

void GroundStationCore::console(const std::string &caller_name, std::stringstream &message, const int &log_level) {
  std::stringstream s;
  s << "[GroundStationCore::" << caller_name << "]  " << message.str();

  if (log_level < WARN) {  // error messages
    ROS_ERROR_STREAM(s.str());
  }
  else if (log_level == WARN) {  // warn messages
    ROS_WARN_STREAM(s.str());
  }
  else if (log_level == INFO) {  // info messages
    ROS_INFO_STREAM(s.str());
  }
  else if (log_level > INFO && log_level <= verbosity_level_) {  // debug messages
    ROS_DEBUG_STREAM(s.str());
  }

  // clears the stringstream passed to this method
  message.clear();
  message.str(std::string());
}

int GroundStationCore::extractFirstID() {
  for (int i=1; i<=number_of_agents_; i++){
    if (!checkCollision(i)) {
      return i;
    }
  }
  std::stringstream s;
  s << "Can't find a valid ID, check the current number of agents (" << number_of_agents_ << ").";
  console(__func__, s, ERROR);
  return -1;
}

void GroundStationCore::interactiveMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  // updates and shares statistics and updates the relative spanning ellipse
  if (feedback->marker_name == "stats_modifier_pose") {
    // avoids too fast orientation changes in the target ellipse pose
    interactiveMarkerGuidance(feedback->pose, target_pose_);  // updates target_pose_
    interactive_marker_server_->setPose(feedback->marker_name, target_pose_, feedback->header);
  }
  else if (feedback->marker_name == "stats_modifier_axis_x") {
    target_a_x_ = computeA(2*feedback->pose.position.x);
  }
  else if (feedback->marker_name == "stats_modifier_axis_y") {
    target_a_y_ = computeA(2*feedback->pose.position.y);
  }
  else {
    std::stringstream s;
    s << "Wrong marker name ("<< feedback->marker_name << ").";
    console(__func__, s, ERROR);
    return;
  }

  updateTarget(physicsToStats(target_pose_, target_a_x_, target_a_y_));
  interactive_marker_server_->applyChanges();
}

void GroundStationCore::interactiveMarkerGuidance(const geometry_msgs::Pose &target, geometry_msgs::Pose &current) {
  tf::Pose current_pose, target_pose;
  tf::poseMsgToTF(current, current_pose);
  tf::poseMsgToTF(target, target_pose);

  double distance = tf::tfDistance(current_pose.getOrigin(), target_pose.getOrigin());
  double distance_sat = saturation(distance, marker_dist_min_, marker_dist_max_);
  double theta = std::atan2(target.position.y - current.position.y, target.position.x - current.position.x);

  double current_roll, current_pitch, current_yaw, target_roll, target_pitch, target_yaw;
  tf::Matrix3x3(current_pose.getRotation()).getRPY(current_roll, current_pitch, current_yaw);
  tf::Matrix3x3(target_pose.getRotation()).getRPY(target_roll, target_pitch, target_yaw);
  double angle = angles::shortest_angular_distance(current_yaw, target_yaw);
  double angle_sat = saturation(angle, marker_steer_min_, marker_steer_max_);

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(distance_sat*std::cos(theta - current_yaw), distance_sat*std::sin(theta - current_yaw), 0));
  transform.setRotation(tf::createQuaternionFromRPY(0, 0, angle_sat));
  current_pose *= transform;

  tf::poseTFToMsg(current_pose, current);

  std::stringstream s;
  s << "Distance and its satured value (" << distance << ", " << distance_sat << ").";
  console(__func__, s, DEBUG_VVVV);
  s << "Angle and its saturated value (" << angle << ", " << angle_sat << ").";
  console(__func__, s, DEBUG_VVVV);
}

void GroundStationCore::interactiveMarkerInitialization() {
  // also initializes target_a_x_ and target_a_y_
  tf::Pose target_pose = statsToPhysics(target_statistics_.stats, target_a_x_, target_a_y_);
  tf::poseTFToMsg(target_pose, target_pose_);
  makeInteractiveMarkerPose(target_pose_);

  geometry_msgs::Pose pose;
  tf::Transform translate;
  translate.setIdentity();  // null pose (it is centered in the 'target_frame_' frame)

  translate.setOrigin(tf::Vector3(computeDiameter(target_a_x_)/2, 0, 0));
  tf::poseTFToMsg(translate, pose);
  makeInteractiveMarkerAxis(pose, "x");

  translate.setOrigin(tf::Vector3(0, computeDiameter(target_a_y_)/2, 0));
  tf::poseTFToMsg(translate, pose);
  makeInteractiveMarkerAxis(pose, "y");
}

visualization_msgs::Marker GroundStationCore::makeBox(const double &scale) {
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = scale * 0.45;
  marker.scale.y = scale * 0.45;
  marker.scale.z = scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

void GroundStationCore::makeBoxControl(visualization_msgs::InteractiveMarker &interactive_marker) {
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(interactive_marker.scale));

  interactive_marker.controls.push_back(control);
}

// TODO use params for namespaces and frames
visualization_msgs::Marker GroundStationCore::makeEllipse(const double &diameter_x, const double &diameter_y,
                                                          const std::string &frame, const int &id) {
  visualization_msgs::Marker marker;

  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time(0);
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.id = id;
  marker.frame_locked = true;
  if (frame == target_frame_) {
    marker.ns = "target_spanning_ellipse";
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
  }
  else if (frame == "effective_stats_ellipse") {
    marker.ns = "effective_spanning_ellipse";
    marker.color.a = 0.1;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
  }
  else if (frame == "effective_stats_virtual_ellipse") {
    marker.ns = "effective_spanning_virtual_ellipse";
    marker.color.a = 0.1;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
  }
  else {
    marker.ns = "agent_spanning_ellipse";
    marker.color.a = 0.25;
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
  }
  // relative pose is zero: the frame is already properly centered and rotated
  marker.scale.x = (diameter_x > 0) ? diameter_x : 0.1;
  marker.scale.y = (diameter_y > 0) ? diameter_y : 0.1;
  marker.scale.z = 0.001;

  return marker;
}

void GroundStationCore::makeInteractiveMarkerAxis(const geometry_msgs::Pose &pose, const std::string &axis) {
  if (axis != "x" && axis != "y") {
    std::stringstream s;
    s << "Wrong axis string (" << axis << ").";
    console(__func__, s, ERROR);
    return;
  }

  visualization_msgs::InteractiveMarker interactive_marker;
  interactive_marker.header.frame_id = target_frame_;
  interactive_marker.name = "stats_modifier_axis_" + axis;
  interactive_marker.description = "Real-time target statistics modifier (affects only axis dimensions).";
  interactive_marker.pose = pose;
  interactive_marker.scale = 0.5;

  makeBoxControl(interactive_marker);

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1;
  if (axis == "x") {
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 0;
  }
  else {
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
  }
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;  // moves only on x or y
  interactive_marker.controls.push_back(control);

  interactive_marker_server_->insert(interactive_marker);
  interactive_marker_server_->setCallback(interactive_marker.name,
                                          std::bind(&GroundStationCore::interactiveMarkerCallback, this,
                                                    std::placeholders::_1));
  interactive_marker_server_->applyChanges();
}

void GroundStationCore::makeInteractiveMarkerPose(const geometry_msgs::Pose &pose) {
  visualization_msgs::InteractiveMarker interactive_marker;
  interactive_marker.header.frame_id = fixed_frame_;
  interactive_marker.name = "stats_modifier_pose";
  interactive_marker.description = "Real-time target statistics modifier (affects only ellipse pose).";
  interactive_marker.pose = pose;
  interactive_marker.scale = 0.5;

  makeBoxControl(interactive_marker);

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;  // moves on x,y; rotates on z
  interactive_marker.controls.push_back(control);

  interactive_marker_server_->insert(interactive_marker);
  interactive_marker_server_->setCallback(interactive_marker.name,
                                          std::bind(&GroundStationCore::interactiveMarkerCallback, this,
                                                    std::placeholders::_1));
  interactive_marker_server_->applyChanges();
}

void GroundStationCore::matlabPosesCallback(const geometry_msgs::Pose &pose) {
  tf::Pose p;
  tf::poseMsgToTF(pose, p);

  std::string frame = "agent_0";  // TODO param
  if (pose.position.z != 0) {
    frame += "_virtual";
    p.getOrigin().setZ(0);
  }

  tf_broadcaster_.sendTransform(tf::StampedTransform(p, ros::Time::now(), fixed_frame_, frame));
}

agent_test::FormationStatistics GroundStationCore::physicsToStats(const geometry_msgs::Pose &pose, const double &a_x,
                                                                  const double &a_y) {
  double roll, pitch, yaw;
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  agent_test::FormationStatistics stats;
  stats.m_x = pose.position.x;
  stats.m_y = pose.position.y;
  stats.m_xx = a_x*std::pow(std::cos(yaw), 2) + a_y*std::pow(std::sin(yaw), 2) + std::pow(stats.m_x, 2);
  stats.m_xy = (a_x - a_y)*std::sin(yaw)*std::cos(yaw) + stats.m_x*stats.m_y;
  stats.m_yy = a_x*std::pow(std::sin(yaw), 2) + a_y*std::pow(std::cos(yaw), 2) + std::pow(stats.m_y, 2);

  std::stringstream s;
  s << "RPY angles (" << roll << ", " << pitch << ", " << yaw << ").";
  console(__func__, s, DEBUG_VVVV);
  s << "Converted statistics (" << stats.m_x << ", " << stats.m_y << ", " << stats.m_xx << ", " << stats.m_xy << ", "
    << stats.m_yy << ").";
  console(__func__, s, DEBUG_VVV);

  return stats;
}

double GroundStationCore::saturation(const double &value, const double &min, const double &max) {
  return std::min(std::max(value, min), max);
}

void GroundStationCore::sharedStatsCallback(const agent_test::FormationStatisticsStamped &shared) {
  agent_test::FormationStatisticsStamped msg = shared;
  if (msg.header.frame_id == "") {  // agent from MATLAB
    msg.header.frame_id = "agent_" + std::to_string(msg.agent_id) + "_virtual";
  }

  bool updated = false;
  for (auto &stat : shared_statistics_grouped_) {
    if (msg.agent_id == stat.agent_id) {
      stat = msg;
      updated = true;
      break;
    }
  }
  if (!updated) {  // msg.agent_id is a new agent (e.g. from MATLAB)
    connected_agents_.push_back(msg.agent_id);  // can't be in conflict with other ids
    shared_statistics_grouped_.push_back(msg);
  }

  updateSpanningEllipse(msg);

  std::stringstream s;
  s << "Received statistics from " << msg.header.frame_id;
  console(__func__, s, INFO);
}

tf::Pose GroundStationCore::statsToPhysics(const agent_test::FormationStatistics &stats, double &a_x, double &a_y) {
  return statsToPhysics(stats, a_x, a_y, std::nan(""));
}

tf::Pose GroundStationCore::statsToPhysics(const agent_test::FormationStatistics &stats, double &a_x, double &a_y,
                                           const double &theta_old) {
  double m_xy = 2*(stats.m_xy - stats.m_x*stats.m_y);
  double m_xx = stats.m_xx - std::pow(stats.m_x, 2);
  double m_yy = stats.m_yy - std::pow(stats.m_y, 2);

  double theta = std::atan2(m_xy, (m_xx - m_yy))/2;
  std::stringstream s;
  s << "Theta value (" << theta << ").";
  console(__func__, s, DEBUG_VVVV);
  if (!std::isnan(theta_old)) {  // interactive markers must have a coherent pose (theta != theta + PI)
    thetaCorrection(theta, theta_old);
  }

  a_x = m_xx*std::pow(std::cos(theta), 2) + m_yy*std::pow(std::sin(theta), 2) + m_xy*std::sin(theta)*std::cos(theta);
  a_y = m_yy*std::pow(std::cos(theta), 2) + m_xx*std::pow(std::sin(theta), 2) - m_xy*std::sin(theta)*std::cos(theta);

  tf::Pose pose;
  pose.setOrigin(tf::Vector3(stats.m_x, stats.m_y, 0));
  pose.setRotation(tf::createQuaternionFromRPY(0, 0, theta));

  s << "a_x and a_y values (" << a_x << ", " << a_y << ").";
  console(__func__, s, DEBUG_VVVV);

  return pose;
}

agent_test::FormationStatistics GroundStationCore::statsVectorPhysicsToMsg(const std::vector<double> &vector) {
  geometry_msgs::Pose pose_msg;
  tf::Pose pose_tf;
  pose_tf.setOrigin(tf::Vector3(vector.at(0), vector.at(1), 0));
  pose_tf.setRotation(tf::createQuaternionFromRPY(0, 0, vector.at(2)));
  poseTFToMsg(pose_tf, pose_msg);

  return physicsToStats(pose_msg, computeA(vector.at(3)), computeA(vector.at(4)));
}

agent_test::FormationStatistics GroundStationCore::statsVectorToMsg(const std::vector<double> &vector) {
  agent_test::FormationStatistics msg;
  if (vector.size() != 5) {
    std::stringstream s;
    s << "Wrong statistics vector size (" << vector.size() << ").";
    console(__func__, s, ERROR);
    return msg;
  }
  msg.m_x = vector.at(0);
  msg.m_y = vector.at(1);
  msg.m_xx = vector.at(2);
  msg.m_xy = vector.at(3);
  msg.m_yy = vector.at(4);

  return msg;
}

agent_test::FormationStatisticsStamped GroundStationCore::statsVectorToMsg(const std::string &frame, const int &id,
                                                                           const std::vector<double> &vector) {
  agent_test::FormationStatisticsStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame;
  msg.agent_id = id;
  msg.stats = statsVectorToMsg(vector);

  return msg;
}

bool GroundStationCore::syncAgentCallback(agent_test::Sync::Request &request, agent_test::Sync::Response &response) {
  // initializes the synchronization time only at the first callback
  if (sync_time_.isZero()) {
    sync_time_ = ros::Time::now() + sync_delay_;
  }
  response.new_id = (request.agent_id == 0 || checkCollision(request.agent_id)) ? extractFirstID() : request.agent_id;
  response.sync_time = sync_time_ + ros::Duration(sample_time_/4);  // agent nodes start a bit after GS
  connected_agents_.push_back(response.new_id);
  shared_statistics_grouped_.push_back(statsVectorToMsg(request.frame_base_name + std::to_string(response.new_id),
                                                        response.new_id, std::vector<double>({0,0,0,0,0})));
  return true;
}

void GroundStationCore::thetaCorrection(double &theta, const double &theta_old) {
  std::vector<double> thetas;
  std::vector<double> increments = {0, M_PI_2, M_PI, M_PI + M_PI_2};
  for (auto const i : increments) {
    thetas.push_back(angles::normalize_angle(theta + i));
  }

  auto min_distance_theta = std::min_element(std::begin(thetas), std::end(thetas),
                                             [&theta_old](const double &t1, const double &t2){
                                               return std::abs(angles::shortest_angular_distance(t1, theta_old))
                                                      < std::abs(angles::shortest_angular_distance(t2, theta_old));
                                             });

  theta = angles::normalize_angle(thetas.at(min_distance_theta - std::begin(thetas)));

  std::stringstream s;
  s << "Theta value after correction (" << theta << ").";
  console(__func__, s, DEBUG_VVVV);
}

void GroundStationCore::updateSpanningEllipse(const agent_test::FormationStatisticsStamped &msg) {
  double a_x, a_y;  // will be initialized by statsToPhysics
  double roll_old, pitch_old, yaw_old = std::nan("");  // only yaw_old is really used (thus initialized)
  std::string frame = msg.header.frame_id + "_ellipse";

  // retrieves the old pose to correct the new one extracted from statistics (theta from [-pi/2,pi/2] to [-pi,pi])
  if (frame == target_frame_ && tf_listener_.canTransform(fixed_frame_, frame, ros::Time(0))) {
    tf::StampedTransform pose_old;
    tf_listener_.lookupTransform(fixed_frame_, frame, ros::Time(0), pose_old);
    tf::Matrix3x3(pose_old.getRotation()).getRPY(roll_old, pitch_old, yaw_old);
  }

  tf::Pose pose = statsToPhysics(msg.stats, a_x, a_y, yaw_old);
  tf_broadcaster_.sendTransform(tf::StampedTransform(pose, ros::Time::now(), fixed_frame_, frame));
  marker_publisher_.publish(makeEllipse(computeDiameter(a_x), computeDiameter(a_y), frame, msg.agent_id));
}

void GroundStationCore::updateTarget(const agent_test::FormationStatistics &target) {
  updateTargetStats(target);
  updateSpanningEllipse(target_statistics_);
}

void GroundStationCore::updateTargetStats(const agent_test::FormationStatistics &target) {
  // target frame is constant and initialized in the constructor
  target_statistics_.header.stamp = ros::Time::now();
  target_statistics_.stats = target;

  target_stats_publisher_.publish(target_statistics_);
}

void GroundStationCore::waitForSync() {
  while (sync_time_.isZero()) {
    ros::Duration(0.1).sleep();
  }
  std::stringstream s;
  s << "Wait for synchronization deadline (" << sync_time_ << ").";
  console(__func__, s, INFO);
  ros::Time::sleepUntil(sync_time_);
}
