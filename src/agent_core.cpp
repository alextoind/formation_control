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

  pose_virtual_.position.x = integrator(pose_virtual_.position.x, twist_virtual_.linear.x, control_law(1));
  pose_virtual_.position.y = integrator(pose_virtual_.position.y, twist_virtual_.linear.y, control_law(2));
  twist_virtual_.linear.x = control_law(1);
  twist_virtual_.linear.y = control_law(2);

  ROS_DEBUG_STREAM("[AgentCore::control] Virtual Agent velocities (x: " << control_law(1) << ", y: " << control_law(2) << ")");
}

void AgentCore::consensus() {
  Eigen::RowVectorXd x = statsMsgToVector(estimated_statistics_);
  Eigen::MatrixXd x_j = statsMsgToMatrix(received_estimated_statistics_);

  // dynamic discrete consensus: x_k+1 = z_dot_k*S + (I - S*L)x_k = z_dot_k*S + x_k + S*sum_j(x_j_k - x_k)
  x += phi_dot_ *sample_time_ + (x_j.rowwise() - x).colwise().sum()*sample_time_;

  estimated_statistics_ = statsVectorToMsg(x);

  ROS_DEBUG_STREAM("[AgentCore::consensus] Estimated statistics:\n" << x);
}

void AgentCore::dynamics() {

}

void AgentCore::guidance() {

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

double AgentCore::integrator(const double &x_old, const double &x_dot_old, const double &x_dot_new) {
  return x_old + sample_time_*(x_dot_old + x_dot_new)/2;
}
