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

#include <geometry_msgs/Pose.h>
#include "agent_core.h"

AgentCore::AgentCore() {

}

AgentCore::~AgentCore() {

}

void AgentCore::control() {

}

void AgentCore::consensus() {
  // time derivative of phi(p) = [px, py, pxx, pxy, pyy]'
  std::vector<double> z_dot = {twist_virtual_.linear.x,
                               twist_virtual_.linear.y,
                               2*pose_virtual_.position.x*twist_virtual_.linear.x,
                               pose_virtual_.position.y*twist_virtual_.linear.x + pose_virtual_.position.x*twist_virtual_.linear.y,
                               2*pose_virtual_.position.y*twist_virtual_.linear.y};

  // discrete consensus: (I - S*L)xk = xk - S*sum_j(xk - xk_j) = xk + S*sum_j(xk_j - xk)
  std::vector<double> x = statsMsgToVector(estimated_statistics_);
  std::vector<double> x_j, I_SLx;
  std::copy(x.begin(), x.end(), std::back_inserter(I_SLx));
  for (auto const &received_stat : received_estimated_statistics_) {
    x_j = statsMsgToVector(received_stat);
    std::transform(x_j.begin(), x_j.end(), x.begin(), x_j.begin(), std::minus<double>());
    std::transform(x_j.begin(), x_j.end(), x_j.begin(), [this](double d){ return d*sample_time_; });
    std::transform(I_SLx.begin(), I_SLx.end(), x_j.begin(), I_SLx.begin(), std::plus<double>());
  }

  // dynamic discrete consensus: xk+1 = zk_dot*S + (I - S*L)xk
  std::transform(z_dot.begin(), z_dot.end(), z_dot.begin(), [this](double d){ return d*sample_time_; });
  std::transform(z_dot.begin(), z_dot.end(), I_SLx.begin(), x.begin(), std::plus<double>());
  estimated_statistics_ = statsVectorToMsg(x);
}

void AgentCore::dynamics() {

}

void AgentCore::guidance() {

}

std::vector<double> AgentCore::statsMsgToVector(const agent_test::FormationStatistics &msg) {
  return std::vector<double>({msg.m_x, msg.m_y, msg.m_xx, msg.m_xy, msg.m_yy});
}

agent_test::FormationStatistics AgentCore::statsVectorToMsg(const std::vector<double> &vector) {
  agent_test::FormationStatistics msg;
  if (vector.size() != 5) {
    ROS_ERROR_STREAM("[AgentCore::statsVectorToMsg] Wrong statistics vector size (" << vector.size() << ")");
    return msg;
  }

  msg.m_x = vector.at(1);
  msg.m_y = vector.at(2);
  msg.m_xx = vector.at(3);
  msg.m_xy = vector.at(4);
  msg.m_yy = vector.at(5);

  return msg;
}
