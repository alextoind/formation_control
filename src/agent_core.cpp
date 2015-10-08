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

  private_node_handle_->param("agent_id", agent_id_, DEFAULT_AGENT_ID);
  private_node_handle_->param("number_of_stats", number_of_stats_, DEFAULT_NUMBER_OF_STATS);
  private_node_handle_->param("number_of_velocities", number_of_velocities_, DEFAULT_NUMBER_OF_VELOCITIES);
  private_node_handle_->param("sample_time", sample_time_, (double)DEFAULT_SAMPLE_TIME);
  private_node_handle_->param("velocity_virtual_threshold_", velocity_virtual_threshold_, (double)DEFAULT_VELOCITY_VIRTUAL_THRESHOLD);
  private_node_handle_->param("los_distance_threshold", los_distance_threshold_, (double)DEFAULT_LOS_DISTANCE_THRESHOLD);
  private_node_handle_->param("speed_min", speed_min_, (double)DEFAULT_SPEED_MIN);
  private_node_handle_->param("speed_max", speed_max_, (double)DEFAULT_SPEED_MAX);
  private_node_handle_->param("steer_min", steer_min_, (double)DEFAULT_STEER_MIN);
  private_node_handle_->param("steer_max", steer_max_, (double)DEFAULT_STEER_MAX);
  private_node_handle_->param("k_p_speed", k_p_speed_, (double)DEFAULT_K_P_SPEED);
  private_node_handle_->param("k_i_speed", k_i_speed_, (double)DEFAULT_K_I_SPEED);
  private_node_handle_->param("k_p_steer", k_p_steer_, (double)DEFAULT_K_P_STEER);
  private_node_handle_->param("vehicle_length", vehicle_length_, (double)DEFAULT_VEHICLE_LENGTH);
  private_node_handle_->param("world_limit", world_limit_, (double)DEFAULT_WORLD_LIMIT);

  const std::vector<double> DEFAULT_DIAG_ELEMENTS_GAMMA = {500, 500, 5, 5, 5};
  const std::vector<double> DEFAULT_DIAG_ELEMENTS_LAMBDA = {0, 0, 0, 0, 0};
  const std::vector<double> DEFAULT_DIAG_ELEMENTS_B = {500, 500};
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

  private_node_handle_->param("topic_queue_length", topic_queue_length_, DEFAULT_TOPIC_QUEUE_LENGTH);
  private_node_handle_->param("shared_stats_topic", shared_stats_topic_name_, std::string(DEFAULT_SHARED_STATS_TOPIC));
  private_node_handle_->param("received_stats_topic", received_stats_topic_name_, std::string(DEFAULT_RECEIVED_STATS_TOPIC));
  private_node_handle_->param("target_stats_topic", target_stats_topic_name_, std::string(DEFAULT_TARGET_STATS_TOPIC));
  private_node_handle_->param("sync_service", sync_service_name_, std::string(DEFAULT_SYNC_SERVICE));
  private_node_handle_->param("path_topic", path_topic_name_, std::string(DEFAULT_PATH_TOPIC));
  private_node_handle_->param("path_virtual_topic", path_virtual_topic_name_, std::string(DEFAULT_PATH_VIRTUAL_TOPIC));
  private_node_handle_->param("path_max_length", path_max_length_, DEFAULT_PATH_MAX_LENGTH);
  private_node_handle_->param("fixed_frame", fixed_frame_, std::string(DEFAULT_FIXED_FRAME));
  private_node_handle_->param("frame_base_name", frame_base_name_, std::string(DEFAULT_FRAME_BASE_NAME));
  double sync_timeout;
  private_node_handle_->param("sync_timeout", sync_timeout, (double)DEFAULT_SYNC_TIMEOUT);
  sync_timeout_ = ros::Duration(sync_timeout);

  stats_publisher_ = node_handle_.advertise<agent_test::FormationStatisticsStamped>(shared_stats_topic_name_, topic_queue_length_);
  stats_subscriber_ = node_handle_.subscribe(received_stats_topic_name_, topic_queue_length_, &AgentCore::receivedStatsCallback, this);
  target_stats_subscriber_ = node_handle_.subscribe(target_stats_topic_name_, topic_queue_length_, &AgentCore::targetStatsCallback, this);
  sync_client_ = node_handle_.serviceClient<agent_test::Sync>(sync_service_name_);

  path_publisher_ = node_handle_.advertise<nav_msgs::Path>(path_topic_name_, topic_queue_length_);
  path_virtual_publisher_ = node_handle_.advertise<nav_msgs::Path>(path_virtual_topic_name_, topic_queue_length_);

  waitForSyncTime();

  algorithm_timer_ = private_node_handle_->createTimer(ros::Duration(sample_time_), &AgentCore::algorithmCallback, this);
};

AgentCore::~AgentCore() {
  delete private_node_handle_;
}

void AgentCore::algorithmCallback(const ros::TimerEvent &timer_event) {
  consensus();  // also publishes estimated statistics
  control();  // also publishes virtual agent pose and path
  guidance();
  dynamics();  // also publishes agent pose and path
}

void AgentCore::broadcastPath(const geometry_msgs::Pose &pose, std::vector<geometry_msgs::PoseStamped> &path,
                              const ros::Publisher &publisher) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = fixed_frame_;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.pose = pose;

  path.push_back(pose_msg);
  if (path.size() > path_max_length_) {
    path.erase(path.begin());
  }

  nav_msgs::Path msg;
  msg.header.frame_id = fixed_frame_;
  msg.header.stamp = ros::Time::now();
  msg.poses = path;
  publisher.publish(msg);
}

void AgentCore::broadcastPose(const geometry_msgs::Pose &pose, const std::string &frame) {
  tf::Pose p;
  tf::poseMsgToTF(pose, p);
  tf_broadcaster_.sendTransform(tf::StampedTransform(p, ros::Time::now(), fixed_frame_, frame));
}

void AgentCore::consensus() {
  Eigen::RowVectorXd x = statsMsgToVector(estimated_statistics_);
  Eigen::MatrixXd x_j = statsMsgToMatrix(received_statistics_);
  // clears the private variable for following callbacks
  received_statistics_.clear();
  // time derivative of phi(p) = [px, py, pxx, pxy, pyy]
  phi_dot_ << twist_virtual_.linear.x,
              twist_virtual_.linear.y,
              2*pose_virtual_.position.x*twist_virtual_.linear.x,
              pose_virtual_.position.y*twist_virtual_.linear.x + pose_virtual_.position.x*twist_virtual_.linear.y,
              2*pose_virtual_.position.y*twist_virtual_.linear.y;

  // dynamic discrete consensus: x_k+1 = z_dot_k*S + (I - S*L)x_k = z_dot_k*S + x_k + S*sum_j(x_j_k - x_k)
  x += phi_dot_*sample_time_ + (x_j.rowwise() - x).colwise().sum()*sample_time_;

  estimated_statistics_ = statsVectorToMsg(x);
  ROS_DEBUG_STREAM("[AgentCore::consensus] Estimated statistics: [" << x << "].");

  agent_test::FormationStatisticsStamped msg;
  msg.header.frame_id = agent_virtual_frame_;
  msg.header.stamp = ros::Time::now();
  msg.agent_id = agent_id_;
  msg.stats = estimated_statistics_;
  stats_publisher_.publish(msg);
}

void AgentCore::control() {  // TODO fix
  Eigen::VectorXd stats_error = statsMsgToVector(target_statistics_) - statsMsgToVector(estimated_statistics_);
  // update non constant values of the jacobian of phi(p) = [px, py, pxx, pxy, pyy]
  jacob_phi_(2,0) = 2*pose_virtual_.position.x;
  jacob_phi_(3,0) = pose_virtual_.position.y;
  jacob_phi_(3,1) = pose_virtual_.position.x;
  jacob_phi_(4,1) = 2*pose_virtual_.position.y;

  // twist_virtual = inv(B + Jphi'*lambda*Jphi) * Jphi' * gamma * stats_error
  Eigen::VectorXd control_law = (b_ + jacob_phi_.transpose()*lambda_*jacob_phi_).inverse()
                                * jacob_phi_.transpose()*gamma_*stats_error;

  // control command saturation
  double current_velocity_virtual = std::sqrt(std::pow(control_law(0),2) + std::pow(control_law(1),2));
  if (current_velocity_virtual > velocity_virtual_threshold_) {
    control_law *= velocity_virtual_threshold_ / current_velocity_virtual;
  }

  pose_virtual_.position.x = integrator(pose_virtual_.position.x, twist_virtual_.linear.x, control_law(0), 1);
  pose_virtual_.position.y = integrator(pose_virtual_.position.y, twist_virtual_.linear.y, control_law(1), 1);
  setTheta(pose_virtual_.orientation, std::atan2(control_law(1), control_law(0)));
  twist_virtual_.linear.x = control_law(0);
  twist_virtual_.linear.y = control_law(1);

  ROS_DEBUG_STREAM("[AgentCore::control] Virtual agent pose (x: " << pose_virtual_.position.x
                   << ", y: " << pose_virtual_.position.y << ")");
  ROS_DEBUG_STREAM("[AgentCore::control] Virtual agent twist (x: " << twist_virtual_.linear.x
                   << ", y: " << twist_virtual_.linear.y << ")");

  broadcastPose(pose_virtual_, agent_virtual_frame_);
  broadcastPath(pose_virtual_, path_virtual_, path_virtual_publisher_);  // TODO use Marker instead of Path
}

void AgentCore::dynamics() {
  double theta = getTheta(pose_.orientation);
  double x_dot_new = speed_command_sat_ * std::cos(theta);
  double y_dot_new = twist_.linear.y = speed_command_sat_ * std::sin(theta);
  double theta_dot_new = speed_command_sat_ / vehicle_length_ * std::tan(steer_command_sat_);

  pose_.position.x = integrator(pose_.position.x, twist_.linear.x, x_dot_new, 1);
  pose_.position.y = integrator(pose_.position.y, twist_.linear.y, y_dot_new, 1);
  setTheta(pose_.orientation, integrator(theta, twist_.angular.z, theta_dot_new, 1));
  twist_.linear.x = x_dot_new;
  twist_.linear.y = y_dot_new;
  twist_.angular.z = theta_dot_new;

  ROS_DEBUG_STREAM("[AgentCore::dynamics] Agent pose (x: " << pose_.position.x << ", y: " << pose_.position.y << ")");
  ROS_DEBUG_STREAM("[AgentCore::dynamics] Agent twist (x: " << twist_.linear.x << ", y: " << twist_.linear.y << ")");

  broadcastPose(pose_, agent_frame_);
  broadcastPath(pose_, path_, path_publisher_);  // TODO use Marker instead of Path
}

Eigen::Vector3d AgentCore::getRPY(const geometry_msgs::Quaternion &quat) {
  Eigen::Quaterniond eigen_quat;
  tf::quaternionMsgToEigen(quat, eigen_quat);
  Eigen::Vector3d rpy = eigen_quat.normalized().matrix().eulerAngles(0, 1, 2);
  return rpy;
}

double AgentCore::getTheta(const geometry_msgs::Quaternion &quat) {
  Eigen::Vector3d rpy = getRPY(quat);
  return rpy(2);
}

void AgentCore::guidance() {  // TODO fix
  los_distance_ = std::sqrt(std::pow(pose_virtual_.position.x - pose_.position.x, 2)
                            + std::pow(pose_virtual_.position.y - pose_.position.y, 2));
  // std::atan2 automatically handle the los_distance_ == 0 case >> los_angle_ = 0 TODO: try with velocty instead of position
  los_angle_ = std::atan2(pose_virtual_.position.y - pose_.position.y, pose_virtual_.position.x - pose_.position.x);

  // the speed reference is a proportional value based on the LOS distance (with a saturation)
  double speed_reference = std::min(speed_max_*los_distance_/los_distance_threshold_, speed_max_);
  double speed_error_old = speed_error_;
  speed_error_ = speed_reference - std::sqrt(std::pow(twist_.linear.x, 2) + std::pow(twist_.linear.y, 2));
  speed_integral_ = integrator(speed_integral_, speed_error_old, speed_error_, k_i_speed_);
  double speed_command = k_p_speed_*(speed_error_ + speed_integral_);
  speed_command_sat_ = saturation(speed_command, speed_min_, speed_max_);
  ROS_DEBUG_STREAM("[AgentCore::guidance] Speed command: " << speed_command_sat_);

  double steer_command = k_p_steer_*std::fmod(los_angle_ - getTheta(pose_.orientation), M_PI);
  steer_command_sat_ = saturation(steer_command, steer_min_, steer_max_);
  ROS_DEBUG_STREAM("[AgentCore::guidance] Steer command: " << steer_command_sat_);
}

double AgentCore::integrator(const double &out_old, const double &in_old, const double &in_new, const double &k) {
  return out_old + k*sample_time_*(in_old + in_new)/2;
}

void AgentCore::receivedStatsCallback(const agent_test::FormationStatisticsArray &received) {
  if (!received_statistics_.empty()) {
    ROS_ERROR_STREAM("[AgentCore::receivedStatsCallback] Last received statistics has not been used.");
    received_statistics_.clear();
  }
  neighbours_ = received.neighbours_;  // updates current neighbourhood
  for (auto const &data : received.vector) {
    if (std::find(std::begin(neighbours_), std::end(neighbours_), data.agent_id) != std::end(neighbours_)) {
      received_statistics_.push_back(data.stats);
    }
  }
  ROS_DEBUG_STREAM("[AgentCore::receivedStatsCallback] Received statistics from " << received_statistics_.size() << " agents.");
}

double AgentCore::saturation(const double &value, const double &min, const double &max) {
  return std::min(std::max(value, min), max);
}

void AgentCore::setTheta(geometry_msgs::Quaternion &quat, const double &theta) {
  Eigen::Vector3d rpy = getRPY(quat);
  Eigen::Quaterniond eigen_quat = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX())
                                  * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY())
                                  * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
  tf::quaternionEigenToMsg(eigen_quat, quat);
  ROS_DEBUG_STREAM("[AgentCore::setTheta] Quaternion updated (theta " << theta << "): [" << quat.x << ", " << quat.y
                   << ", " << quat.z << ", " << quat.w << "].");
}

Eigen::MatrixXd AgentCore::statsMsgToMatrix(const std::vector<agent_test::FormationStatistics> &msg) {
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(msg.size(), number_of_stats_);
  for (int i=0; i<msg.size(); i++) {
    matrix.row(i) = statsMsgToVector(msg.at(i));
  }
  ROS_DEBUG_STREAM("[AgentCore::statsMsgToMatrix] Received statistics:  [" << matrix << "].");
  return matrix;
}

Eigen::VectorXd AgentCore::statsMsgToVector(const agent_test::FormationStatistics &msg) {
  Eigen::VectorXd vector(number_of_stats_);
  vector << msg.m_x, msg.m_y, msg.m_xx, msg.m_xy, msg.m_yy;
  return vector;
}

agent_test::FormationStatistics AgentCore::statsVectorToMsg(const Eigen::VectorXd &vector) {
  agent_test::FormationStatistics msg;
  if (vector.size() != number_of_stats_) {
    ROS_ERROR_STREAM("[AgentCore::statsVectorToMsg] Wrong statistics vector size (" << vector.size() << ")");
    return msg;
  }
  msg.m_x = vector(0);
  msg.m_y = vector(1);
  msg.m_xx = vector(2);
  msg.m_xy = vector(3);
  msg.m_yy = vector(4);

  return msg;
}

agent_test::FormationStatistics AgentCore::statsVectorToMsg(const std::vector<double> &vector) {
  std::vector<double> v = vector;  // can't use 'data()' method on const std::vector
  return statsVectorToMsg(Eigen::Map<Eigen::VectorXd>(v.data(), v.size()));
}

void AgentCore::targetStatsCallback(const agent_test::FormationStatisticsStamped &target) {
  target_statistics_ = target.stats;

  ROS_INFO_STREAM("[AgentCore::targetStatsCallback] Target statistics has been changed.");
  ROS_DEBUG_STREAM("[AgentCore::targetStatsCallback] New values: Mx=" << target.stats.m_x << ", My=" << target.stats.m_y
                   << ", Mxx=" << target.stats.m_xx << ", Mxy=" << target.stats.m_xy << ", Myy=" << target.stats.m_yy);
}

void AgentCore::waitForSyncTime() {
  sync_client_.waitForExistence(sync_timeout_);
  agent_test::Sync srv;
  srv.request.agent_id = agent_id_;
  srv.request.frame_base_name = frame_base_name_;
  if (sync_client_.call(srv)) {
    agent_id_ = srv.response.new_id;  // always update, even if it does not change
    agent_frame_ = frame_base_name_ + std::to_string(agent_id_);
    agent_virtual_frame_ = agent_frame_ + "_virtual";

    ROS_DEBUG_STREAM("[AgentCore::waitForSyncTime] Wait for synchronization deadline (" << srv.response.sync_time << ")");
    ros::Time::sleepUntil(srv.response.sync_time);
  }
  else {
    ROS_ERROR_STREAM("[AgentCore::waitForSyncTime] Can't get synchronization time from the server (Ground Station)");
  }
}
