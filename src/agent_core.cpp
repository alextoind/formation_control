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

  const Eigen::VectorXd ONES_STATS = Eigen::VectorXd::Ones(number_of_stats_);
  const std::vector<double> STD_ONES_STATS(ONES_STATS.data(), ONES_STATS.data() + ONES_STATS.size());
  const Eigen::VectorXd ZEROS_STATS = Eigen::VectorXd::Zero(number_of_stats_);
  const std::vector<double> STD_ZEROS_STATS(ZEROS_STATS.data(), ZEROS_STATS.data() + ZEROS_STATS.size());
  const Eigen::VectorXd ONES_VELOCITIES = Eigen::VectorXd::Ones(number_of_velocities_);
  const std::vector<double> STD_ONES_VELOCITIES(ONES_VELOCITIES.data(), ONES_VELOCITIES.data() + ONES_VELOCITIES.size());

  std::vector<double> diag_elements_gamma;
  std::vector<double> diag_elements_lambda;
  std::vector<double> diag_elements_b;
  private_node_handle_->param("diag_elements_gamma", diag_elements_gamma, STD_ONES_STATS);
  private_node_handle_->param("diag_elements_lambda", diag_elements_lambda, STD_ZEROS_STATS);
  private_node_handle_->param("diag_elements_b", diag_elements_b, STD_ONES_VELOCITIES);

  gamma_ = Eigen::Map<Eigen::VectorXd>(diag_elements_gamma.data(), number_of_stats_).asDiagonal();
  lambda_ = Eigen::Map<Eigen::VectorXd>(diag_elements_lambda.data(), number_of_stats_).asDiagonal();
  b_ = Eigen::Map<Eigen::VectorXd>(diag_elements_b.data(), number_of_velocities_).asDiagonal();
  jacob_phi_ = Eigen::MatrixXd::Identity(number_of_stats_, number_of_velocities_);
  jacob_phi_(3,1) = 2*pose_virtual_.position.x;
  jacob_phi_(4,1) = pose_virtual_.position.y;
  jacob_phi_(4,2) = pose_virtual_.position.x;
  jacob_phi_(5,2) = 2*pose_virtual_.position.y;

  // time derivative of phi(p) = [px, py, pxx, pxy, pyy], used for the dynamic consensus
  phi_dot_ << twist_virtual_.linear.x,
              twist_virtual_.linear.y,
              2*pose_virtual_.position.x*twist_virtual_.linear.x,
              pose_virtual_.position.y*twist_virtual_.linear.x + pose_virtual_.position.x*twist_virtual_.linear.y,
              2*pose_virtual_.position.y*twist_virtual_.linear.y;


}

AgentCore::~AgentCore() {

}

// TODO: extend the algorithm to work in 3D even if our approx is in 2D

void AgentCore::control() {
  Eigen::VectorXd stats_error = statsMsgToVector(target_statistics_) - statsMsgToVector(estimated_statistics_);

  // twist_virtual = inv(B + Jphi'*lambda*Jphi) * Jphi' * gamma * stats_error
  Eigen::VectorXd control_law = (b_ + jacob_phi_.transpose()*lambda_*jacob_phi_).inverse()
                                * jacob_phi_.transpose()*gamma_*stats_error;

  // control command saturation
  double current_velocity_virtual = std::sqrt(std::pow(control_law(1),2) + std::pow(control_law(2),2));
  if (current_velocity_virtual > velocity_virtual_threshold_) {
    control_law *= velocity_virtual_threshold_ / current_velocity_virtual;
  }

  pose_virtual_.position.x = integrator(pose_virtual_.position.x, twist_virtual_.linear.x, control_law(1), 1);
  pose_virtual_.position.y = integrator(pose_virtual_.position.y, twist_virtual_.linear.y, control_law(2), 1);
  twist_virtual_.linear.x = control_law(1);
  twist_virtual_.linear.y = control_law(2);

  ROS_DEBUG_STREAM("[AgentCore::control] Virtual Agent velocities (x: " << control_law(1) << ", y: " << control_law(2) << ")");
}

void AgentCore::consensus() {
  Eigen::RowVectorXd x = statsMsgToVector(estimated_statistics_);
  Eigen::MatrixXd x_j = statsMsgToMatrix(received_estimated_statistics_);

  // dynamic discrete consensus: x_k+1 = z_dot_k*S + (I - S*L)x_k = z_dot_k*S + x_k + S*sum_j(x_j_k - x_k)
  x += phi_dot_*sample_time_ + (x_j.rowwise() - x).colwise().sum()*sample_time_;

  estimated_statistics_ = statsVectorToMsg(x);

  ROS_DEBUG_STREAM("[AgentCore::consensus] Estimated statistics:\n" << x);
}

void AgentCore::dynamics() {
  double theta = getTheta(pose_.orientation);
  double x_dot_new = speed_command_sat_ * std::cos(theta);
  double y_dot_new = twist_.linear.y = speed_command_sat_ * std::sin(theta);
  double theta_dot_new = speed_command_sat_ / vehicle_length_ * std::tan(steer_command_sat_);

  pose_.position.x = integrator(pose_.position.x, twist_.linear.x, x_dot_new, 1);
  pose_.position.y = integrator(pose_.position.y, twist_.linear.y, y_dot_new, 1);
  setTetha(pose_.orientation, integrator(theta, twist_.angular.z, theta_dot_new, 1));
  twist_.linear.x = x_dot_new;
  twist_.linear.y = y_dot_new;
  twist_.angular.z = theta_dot_new;
}

void AgentCore::guidance() {
  los_distance_ = std::sqrt(std::pow(pose_virtual_.position.x - pose_.position.x, 2)
                            + std::pow(pose_virtual_.position.y - pose_.position.y, 2));
  // std::atan2 automatically handle the los_distance_ == 0 case >> los_angle_ = 0 TODO: try with velocty instead of position
  los_angle_ = std::atan2(pose_virtual_.position.y - pose_.position.y, pose_virtual_.position.x - pose_.position.x);

  // TODO: comment the code
  double speed_reference = std::min(speed_max_*los_distance_/los_distance_threshold_, speed_max_);
  double speed_error_old = speed_error_;
  speed_error_ = speed_reference - std::sqrt(std::pow(twist_.linear.x, 2) + std::pow(twist_.linear.y, 2));
  speed_integral_ = integrator(speed_integral_, speed_error_old, speed_error_, k_i_speed_);
  double speed_command = k_p_speed_*(speed_error_ + speed_integral_);
  speed_command_sat_ = saturation(speed_command, speed_min_, speed_max_);
  ROS_DEBUG_STREAM("[AgentCore::guidance] Speed command: " << speed_command_sat_);

  // TODO: comment the code
  double steer_command = k_p_steer_*std::fmod(los_angle_ - getTheta(pose_.orientation), M_PI);
  steer_command_sat_ = saturation(steer_command, steer_min_, steer_max_);
  ROS_DEBUG_STREAM("[AgentCore::guidance] Steer command: " << steer_command_sat_);
}

Eigen::VectorXd AgentCore::statsMsgToVector(const agent_test::FormationStatistics &msg) {
  Eigen::VectorXd vector(number_of_stats_);
  vector << msg.m_x, msg.m_y, msg.m_xx, msg.m_xy, msg.m_yy;
  return vector;
}

Eigen::MatrixXd AgentCore::statsMsgToMatrix(const std::vector<agent_test::FormationStatistics> &msg) {
  Eigen::MatrixXd matrix(number_of_stats_, number_of_stats_);
  for (int i=0; i<number_of_stats_; i++) {
    matrix.row(i) = statsMsgToVector(msg.at(i));
  }
  return matrix;
}

agent_test::FormationStatistics AgentCore::statsVectorToMsg(const Eigen::VectorXd &vector) {
  agent_test::FormationStatistics msg;
  if (vector.size() != number_of_stats_) {
    ROS_ERROR_STREAM("[AgentCore::statsVectorToMsg] Wrong statistics vector size (" << vector.size() << ")");
    return msg;
  }
  msg.m_x = vector(1);
  msg.m_y = vector(2);
  msg.m_xx = vector(3);
  msg.m_xy = vector(4);
  msg.m_yy = vector(5);

  return msg;
}

double AgentCore::integrator(const double &out_old, const double &in_old, const double &in_new, const double &k) {
  return out_old + k*sample_time_*(in_old + in_new)/2;
}

double AgentCore::saturation(const double &value, const double &min, const double &max) {
  return std::min(std::max(value, min), max);
}

double AgentCore::getTheta(const geometry_msgs::Quaternion &quat) {
  Eigen::Quaterniond eigen_quat;
  tf::quaternionMsgToEigen(quat, eigen_quat);
  Eigen::Vector3d rpy = eigen_quat.normalized().matrix().eulerAngles(0, 1, 2);
  return rpy(2);
}

void AgentCore::setTetha(geometry_msgs::Quaternion &quat, const double &theta) {
  Eigen::Quaterniond eigen_quat;
  tf::quaternionMsgToEigen(quat, eigen_quat);
  Eigen::Vector3d rpy = eigen_quat.normalized().matrix().eulerAngles(0, 1, 2);

  eigen_quat = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX())
               * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY())
               * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());

  tf::quaternionEigenToMsg(eigen_quat, quat);
}
