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

#include "agent_core.h"

AgentCore::AgentCore() {
  // handles server private parameters (private names are protected from accidental name collisions)
  private_node_handle_ = new ros::NodeHandle("~");

  private_node_handle_->param("sample_time", sample_time_, (double)DEFAULT_SAMPLE_TIME);
  private_node_handle_->param("slot_tdma", slot_tdma_, (double)DEFAULT_SLOT_TDMA);
  private_node_handle_->param("agent_id", agent_id_, DEFAULT_AGENT_ID);
  private_node_handle_->param("number_of_stats", number_of_stats_, DEFAULT_NUMBER_OF_STATS);
  private_node_handle_->param("number_of_velocities", number_of_velocities_, DEFAULT_NUMBER_OF_VELOCITIES);
  private_node_handle_->param("verbosity_level", verbosity_level_, DEFAULT_VERBOSITY_LEVEL);
  private_node_handle_->param("velocity_virtual_threshold", velocity_virtual_threshold_, (double)DEFAULT_VELOCITY_VIRTUAL_THRESHOLD);
  private_node_handle_->param("speed_min", speed_min_, (double)DEFAULT_SPEED_MIN);
  private_node_handle_->param("speed_max", speed_max_, (double)DEFAULT_SPEED_MAX);
  private_node_handle_->param("steer_min", steer_min_, (double)DEFAULT_STEER_MIN);
  private_node_handle_->param("steer_max", steer_max_, (double)DEFAULT_STEER_MAX);
  private_node_handle_->param("k_p_speed", k_p_speed_, (double)DEFAULT_K_P_SPEED);
  private_node_handle_->param("k_p_steer", k_p_steer_, (double)DEFAULT_K_P_STEER);
  private_node_handle_->param("vehicle_length", vehicle_length_, (double)DEFAULT_VEHICLE_LENGTH);
  private_node_handle_->param("world_limit", world_limit_, (double)DEFAULT_WORLD_LIMIT);

  const std::vector<double> DEFAULT_DIAG_ELEMENTS_GAMMA = {100, 100, 5, 10, 5};
  const std::vector<double> DEFAULT_DIAG_ELEMENTS_LAMBDA = {0, 0, 0, 0, 0};
  const std::vector<double> DEFAULT_DIAG_ELEMENTS_B = {100, 100};
  std::vector<double> diag_elements_gamma;
  std::vector<double> diag_elements_lambda;
  std::vector<double> diag_elements_b;
  private_node_handle_->param("diag_elements_gamma", diag_elements_gamma, DEFAULT_DIAG_ELEMENTS_GAMMA);
  private_node_handle_->param("diag_elements_lambda", diag_elements_lambda, DEFAULT_DIAG_ELEMENTS_LAMBDA);
  private_node_handle_->param("diag_elements_b", diag_elements_b, DEFAULT_DIAG_ELEMENTS_B);

  gamma_ = Eigen::Map<Eigen::VectorXd>(diag_elements_gamma.data(), number_of_stats_).asDiagonal();
  lambda_ = Eigen::Map<Eigen::VectorXd>(diag_elements_lambda.data(), number_of_stats_).asDiagonal();
  b_ = Eigen::Map<Eigen::VectorXd>(diag_elements_b.data(), number_of_velocities_).asDiagonal();
  jacob_phi_ = Eigen::MatrixXd::Identity(number_of_stats_, number_of_velocities_);
  phi_dot_.resize(number_of_stats_);

  std::random_device rd;
  std::mt19937 generator(rd());
  std::uniform_real_distribution<> distrib_position(-world_limit_, world_limit_);
  std::uniform_real_distribution<> distrib_orientation(-M_PI, M_PI);
  double theta;
  // agent pose initialization (we assume a null twist at the beginning)
  private_node_handle_->param("x", pose_.position.x, distrib_position(generator));
  private_node_handle_->param("y", pose_.position.y, distrib_position(generator));
  private_node_handle_->param("theta", theta, distrib_orientation(generator));
  pose_.orientation.w = 1;
  setTheta(pose_.orientation, theta);
  pose_virtual_ = pose_;

  std::vector<double> initial_estimation = {pose_.position.x, pose_.position.y, std::pow(pose_.position.x, 2),
                                            pose_.position.x * pose_.position.y, std::pow(pose_.position.y, 2)};
  estimated_statistics_ = statsVectorToMsg(initial_estimation);
  target_statistics_ = estimated_statistics_;

  private_node_handle_->param("topic_queue_length", topic_queue_length_, DEFAULT_TOPIC_QUEUE_LENGTH);
  private_node_handle_->param("shared_stats_topic", shared_stats_topic_name_, std::string(DEFAULT_SHARED_STATS_TOPIC));
  private_node_handle_->param("target_stats_topic", target_stats_topic_name_, std::string(DEFAULT_TARGET_STATS_TOPIC));
  private_node_handle_->param("marker_topic", marker_topic_name_, std::string(DEFAULT_MARKER_TOPIC));
  private_node_handle_->param("marker_path_lifetime", marker_path_lifetime_, DEFAULT_MARKER_PATH_LIFETIME);
  private_node_handle_->param("enable_path", enable_path_, true);

  private_node_handle_->param("frame_map", frame_map_, std::string(DEFAULT_FRAME_MAP));
  private_node_handle_->param("frame_agent_prefix", frame_agent_prefix_, std::string(DEFAULT_FRAME_AGENT_PREFIX));
  private_node_handle_->param("frame_virtual_suffix", frame_virtual_suffix_, std::string(DEFAULT_FRAME_VIRTUAL_SUFFIX));

  agent_frame_ = frame_agent_prefix_ + std::to_string(agent_id_);
  agent_virtual_frame_ = agent_frame_ + frame_virtual_suffix_;

  stats_publisher_ = node_handle_.advertise<formation_control::FormationStatisticsStamped>(shared_stats_topic_name_, topic_queue_length_);
  stats_subscriber_ = node_handle_.subscribe(shared_stats_topic_name_, topic_queue_length_, &AgentCore::receivedStatsCallback, this);
  target_stats_subscriber_ = node_handle_.subscribe(target_stats_topic_name_, topic_queue_length_, &AgentCore::targetStatsCallback, this);
  marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>(marker_topic_name_, topic_queue_length_);

  waitForSlotTDMA(sample_time_);  // sync to the next sample time slot
  // must be immediately after the waitForSlotTDMA method to ensure a satisfactory synchronization with TDMA protocol
  algorithm_timer_ = private_node_handle_->createTimer(ros::Duration(sample_time_), &AgentCore::algorithmCallback, this);
}

AgentCore::~AgentCore() {
  delete private_node_handle_;
}

visualization_msgs::Marker AgentCore::addToMarkerPath(const geometry_msgs::Point &p_old, const geometry_msgs::Point &p_new,
                                                      const std::string &frame) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_map_;
  marker.header.stamp = ros::Time(0);
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = frame + "_path";
  marker.id = marker_path_id_++;
  marker.frame_locked = true;
  marker.lifetime = ros::Duration(marker_path_lifetime_);
  if (frame.find(frame_virtual_suffix_) != std::string::npos) {
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
  }
  else {
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
  }
  marker.points.push_back(p_old);
  marker.points.push_back(p_new);
  // relative pose is zero: the frame is already properly centered and rotated
  marker.scale.x = 0.05;

  return marker;
}

void AgentCore::algorithmCallback(const ros::TimerEvent &timer_event) {
  consensus();  // also clears the received statistics container
  control();  // also publishes virtual agent pose and path
  guidance();
  dynamics();  // also publishes agent pose and path

  waitForSlotTDMA(slot_tdma_*agent_id_);  // sync to the next transmission TDMA slot (agent dependent)

  // publishes the last estimated statistics in the proper TDMA slot
  formation_control::FormationStatisticsStamped msg;
  msg.header.frame_id = agent_virtual_frame_;
  msg.header.stamp = ros::Time::now();
  msg.agent_id = agent_id_;
  msg.stats = estimated_statistics_;
  stats_publisher_.publish(msg);

  std::stringstream s;
  s << "Estimated statistics published.";
  console(__func__, s, DEBUG);
}

void AgentCore::broadcastPath(const geometry_msgs::Pose &pose_new, const geometry_msgs::Pose &pose_old, const std::string &frame) {
  if (enable_path_) {
    marker_publisher_.publish(addToMarkerPath(pose_old.position, pose_new.position, frame));
  }
}

void AgentCore::broadcastPose(const geometry_msgs::Pose &pose, const std::string &frame) {
  tf::Pose p;
  tf::poseMsgToTF(pose, p);
  tf_broadcaster_.sendTransform(tf::StampedTransform(p, ros::Time::now(), frame_map_, frame));
}

void AgentCore::consensus() {
  Eigen::RowVectorXd x = statsMsgToVector(estimated_statistics_);
  received_statistics_mutex_.lock();
  Eigen::MatrixXd x_j = statsMsgToMatrix(received_statistics_);
  // clears the private variable for following callbacks
  received_statistics_.clear();
  received_statistics_mutex_.unlock();

  std::stringstream s;
  s << "Received statistics from " << x_j.rows()  << " agents.";
  console(__func__, s, INFO);
  s << "Received statistics \n" << x_j;
  console(__func__, s, DEBUG);

  // time derivative of phi(p) = [px, py, pxx, pxy, pyy]
  phi_dot_ << twist_virtual_.linear.x,
              twist_virtual_.linear.y,
              2*pose_virtual_.position.x*twist_virtual_.linear.x,
              pose_virtual_.position.y*twist_virtual_.linear.x + pose_virtual_.position.x*twist_virtual_.linear.y,
              2*pose_virtual_.position.y*twist_virtual_.linear.y;

  double convergence_consensus_limit = 1.0/(x_j.rows() + 1);  // 1/deg_max >> (I - Ts*L) is primitive
  if (sample_time_ >= convergence_consensus_limit) {
    s << "The current sample time (" << sample_time_
      << ") does not guarantee the consensus convergence (upper bound: " << convergence_consensus_limit << ").";
    console(__func__, s, ERROR);
  }

  // dynamic discrete consensus: x_k+1 = phi_k*Ts + (I - Ts*L)x_k = phi_k*Ts + x_k + Ts*sum_j(x_j_k - x_k)
  x += phi_dot_*sample_time_ + (x_j.rowwise() - x).colwise().sum()*sample_time_;

  estimated_statistics_ = statsVectorToMsg(x);

  s << "Estimated statistics (" << x << ").";
  console(__func__, s, INFO);
}

void AgentCore::console(const std::string &caller_name, std::stringstream &message, const int &log_level) const {
  std::stringstream s;
  s << "[AgentCore::" << caller_name << "::Agent_" << agent_id_ << "]  " << message.str();

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

void AgentCore::control() {
  Eigen::VectorXd stats_error = statsMsgToVector(target_statistics_) - statsMsgToVector(estimated_statistics_);
  // update non constant values of the jacobian of phi(p) = [px, py, pxx, pxy, pyy]
  jacob_phi_(2,0) = 2*pose_virtual_.position.x;
  jacob_phi_(3,0) = pose_virtual_.position.y;
  jacob_phi_(3,1) = pose_virtual_.position.x;
  jacob_phi_(4,1) = 2*pose_virtual_.position.y;

  // twist_virtual = inv(B + Jphi'*lambda*Jphi) * Jphi' * gamma * stats_error
  Eigen::VectorXd control_law = (b_ + jacob_phi_.transpose()*lambda_*jacob_phi_).inverse()
                                *jacob_phi_.transpose()*gamma_*stats_error;

  // control command saturation
  double current_velocity_virtual = std::sqrt(std::pow(control_law(0),2) + std::pow(control_law(1),2));
  if (current_velocity_virtual > velocity_virtual_threshold_) {
    control_law *= velocity_virtual_threshold_ / current_velocity_virtual;
  }

  geometry_msgs::Pose pose_old = pose_virtual_;
  pose_virtual_.position.x = integrator(pose_virtual_.position.x, twist_virtual_.linear.x, control_law(0), 1);
  pose_virtual_.position.y = integrator(pose_virtual_.position.y, twist_virtual_.linear.y, control_law(1), 1);
  setTheta(pose_virtual_.orientation, std::atan2(control_law(1), control_law(0)));
  twist_virtual_.linear.x = control_law(0);
  twist_virtual_.linear.y = control_law(1);

  std::stringstream s;
  s << "Statistics error (" << (Eigen::RowVectorXd)stats_error << ").";
  console(__func__, s, DEBUG_V);
  s << "Control commands (" << (Eigen::RowVectorXd)control_law << ").";
  console(__func__, s, DEBUG_VV);
  s << "Virtual pose (" << pose_virtual_.position.x << ", " << pose_virtual_.position.y << ").";
  console(__func__, s, DEBUG_VVV);
  s << "Virtual twist (" << twist_virtual_.linear.x << ", " << twist_virtual_.linear.y << ").";
  console(__func__, s, DEBUG_VVV);

  broadcastPose(pose_virtual_, agent_virtual_frame_);
  broadcastPath(pose_virtual_, pose_old, agent_virtual_frame_);
}

void AgentCore::dynamics() {
  double theta = getTheta(pose_.orientation);
  double x_dot_new = speed_command_sat_ * std::cos(theta);
  double y_dot_new = speed_command_sat_ * std::sin(theta);
  double theta_dot_new = speed_command_sat_ / vehicle_length_ * std::tan(steer_command_sat_);

  geometry_msgs::Pose pose_old = pose_;
  pose_.position.x = integrator(pose_.position.x, twist_.linear.x, x_dot_new, 1);
  pose_.position.y = integrator(pose_.position.y, twist_.linear.y, y_dot_new, 1);
  setTheta(pose_.orientation, integrator(theta, twist_.angular.z, theta_dot_new, 1));
  twist_.linear.x = x_dot_new;
  twist_.linear.y = y_dot_new;
  twist_.angular.z = theta_dot_new;

  std::stringstream s;
  s << "Pose (" << pose_.position.x << ", " << pose_.position.y << ").";
  console(__func__, s, DEBUG_VVV);
  s << "Twist (" << twist_.linear.x << ", " << twist_.linear.y << ").";
  console(__func__, s, DEBUG_VVV);
  s << "System state (" << x_dot_new << ", " << y_dot_new << ", " << theta_dot_new << ").";
  console(__func__, s, DEBUG_VVVV);

  broadcastPose(pose_, agent_frame_);
  broadcastPath(pose_, pose_old, agent_frame_);
}

void AgentCore::floor(double &d, const int &precision) const {
  d = std::floor(d*std::pow(10, precision)) / std::pow(10, precision);
}

Eigen::Vector3d AgentCore::getRPY(const geometry_msgs::Quaternion &quat) const {
  Eigen::Quaterniond eigen_quat;
  tf::quaternionMsgToEigen(quat, eigen_quat);
  Eigen::Vector3d rpy = eigen_quat.normalized().matrix().eulerAngles(0, 1, 2);
  return rpy;
}

double AgentCore::getTheta(const geometry_msgs::Quaternion &quat) const {
  Eigen::Vector3d rpy = getRPY(quat);
  return rpy(2);
}

void AgentCore::guidance() {
  double los_distance = std::sqrt(std::pow(pose_virtual_.position.x - pose_.position.x, 2)
                                  + std::pow(pose_virtual_.position.y - pose_.position.y, 2));
  // std::atan2 automatically handle the los_distance == 0 case >> los_angle = 0
  double los_angle = std::atan2(pose_virtual_.position.y - pose_.position.y,
                                pose_virtual_.position.x - pose_.position.x);

  double speed_command = k_p_speed_*los_distance;
  speed_command_sat_ = saturation(speed_command, speed_min_, speed_max_);

  double steer_command = k_p_steer_*angles::shortest_angular_distance(getTheta(pose_.orientation), los_angle);
  steer_command_sat_ = saturation(steer_command, steer_min_, steer_max_);

  // there is no need to get sub-centimeter accuracy
  floor(speed_command_sat_, 1);

  std::stringstream s;
  s << "Guidance commands (" << speed_command_sat_ << ", " << steer_command_sat_ << ").";
  console(__func__, s, DEBUG_VV);
  s << "LOS distance and angle (" << los_distance << ", " << los_angle << ").";
  console(__func__, s, DEBUG_VVVV);
}

double AgentCore::integrator(const double &out_old, const double &in_old, const double &in_new, const double &k) const {
  return out_old + k*sample_time_*(in_old + in_new)/2;
}

void AgentCore::receivedStatsCallback(const formation_control::FormationStatisticsStamped &received) {
  if (agent_id_ != received.agent_id) {
    received_statistics_mutex_.lock();
    received_statistics_[received.agent_id] = received.stats;
    received_statistics_mutex_.unlock();

    std::stringstream s;
    s << "Received statistics from " << received.header.frame_id  << ".";
    console(__func__, s, DEBUG_VV);
    s << "Received statistics (" << received.stats.m_x << ", " << received.stats.m_y << ", " << received.stats.m_xx
      << ", " << received.stats.m_xy << ", " << received.stats.m_yy << ").";
    console(__func__, s, DEBUG_VVVV);
  }
}

double AgentCore::saturation(const double &value, const double &min, const double &max) const {
  return std::min(std::max(value, min), max);
}

void AgentCore::setTheta(geometry_msgs::Quaternion &quat, const double &theta) const {
  Eigen::Vector3d rpy = getRPY(quat);
  Eigen::Quaterniond eigen_quat = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX())
                                  * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY())
                                  * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
  tf::quaternionEigenToMsg(eigen_quat, quat);

  std::stringstream s;
  s << "Quaternion updated (" << quat.x << ", " << quat.y << ", " << quat.z << ", " << quat.w << ").";
  console(__func__, s, DEBUG_VVVV);
}

Eigen::MatrixXd AgentCore::statsMsgToMatrix(const std::map<int, formation_control::FormationStatistics> &msg) const {
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(msg.size(), number_of_stats_);
  int id = 0;
  for (auto const &stat : msg) {
    matrix.row(id++) = statsMsgToVector(stat.second);
  }
  return matrix;
}

Eigen::VectorXd AgentCore::statsMsgToVector(const formation_control::FormationStatistics &msg) const {
  Eigen::VectorXd vector(number_of_stats_);
  vector << msg.m_x, msg.m_y, msg.m_xx, msg.m_xy, msg.m_yy;
  return vector;
}

formation_control::FormationStatistics AgentCore::statsVectorToMsg(const Eigen::VectorXd &vector) const {
  formation_control::FormationStatistics msg;
  if (vector.size() != number_of_stats_) {
    std::stringstream s;
    s << "Wrong statistics vector size (" << vector.size() << ").";
    console(__func__, s, ERROR);
    return msg;
  }
  msg.m_x = vector(0);
  msg.m_y = vector(1);
  msg.m_xx = vector(2);
  msg.m_xy = vector(3);
  msg.m_yy = vector(4);

  return msg;
}

formation_control::FormationStatistics AgentCore::statsVectorToMsg(const std::vector<double> &vector) const {
  std::vector<double> v = vector;  // can't use 'data()' method on const std::vector
  return statsVectorToMsg(Eigen::Map<Eigen::VectorXd>(v.data(), v.size()));
}

void AgentCore::targetStatsCallback(const formation_control::FormationStatisticsStamped &target) {
  target_statistics_ = target.stats;

  std::stringstream s;
  s << "Target statistics has been changed.";
  console(__func__, s, INFO);
  s << "New target statistics (" << target.stats.m_x << ", " << target.stats.m_y << ", " << target.stats.m_xx << ", "
    << target.stats.m_xy << ", " << target.stats.m_yy << ").";
  console(__func__, s, DEBUG_VVVV);
}

void AgentCore::waitForSlotTDMA(const double &deadline) const{
  ros::Time slot;
  // rounds to the current exact decisecond (hhmmss:x00) plus the proper deadline (agent dependent)
  slot.fromSec(std::floor(ros::Time::now().toSec()*10)/10 + deadline);

  std::stringstream s;
  s << "Wait for TDMA slot (" << slot << ").";
  console(__func__, s, INFO);

  ros::Time::sleepUntil(slot);
}
