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

#include <agent_core.h>
#include "ground_station_core.h"

GroundStationCore::GroundStationCore() {
  // handles server private parameters (private names are protected from accidental name collisions)
  private_node_handle_ = new ros::NodeHandle("~");

  private_node_handle_->param("sample_time", sample_time_, (double)DEFAULT_SAMPLE_TIME);
  private_node_handle_->param("topic_queue_length", topic_queue_length_, DEFAULT_TOPIC_QUEUE_LENGTH);
  private_node_handle_->param("shared_stats_topic", shared_stats_topic_name_, std::string(DEFAULT_SHARED_STATS_TOPIC));
  private_node_handle_->param("received_stats_topic", received_stats_topic_name_, std::string(DEFAULT_RECEIVED_STATS_TOPIC));
  private_node_handle_->param("target_stats_topic", target_stats_topic_name_, std::string(DEFAULT_TARGET_STATS_TOPIC));
  private_node_handle_->param("sync_service", sync_service_name_, std::string(DEFAULT_SYNC_SERVICE));
  private_node_handle_->param("marker_topic", marker_topic_name_, std::string(DEFAULT_MARKER_TOPIC));
  private_node_handle_->param("ground_station_frame", ground_station_frame_, std::string(DEFAULT_GROUND_STATION_FRAME));
  private_node_handle_->param("fixed_frame", fixed_frame_, std::string(DEFAULT_FIXED_FRAME));
  private_node_handle_->param("number_of_agents", number_of_agents_, DEFAULT_NUMBER_OF_AGENTS);
  double sync_delay;
  private_node_handle_->param("sync_delay", sync_delay, (double)DEFAULT_SYNC_DELAY);
  sync_delay_ = ros::Duration(sync_delay);

  std::vector<double> target_statistics;
  const std::vector<double> DEFAULT_TARGET_STATS = {0, 0, 1, 0, 1};
  private_node_handle_->param("target_statistics", target_statistics, DEFAULT_TARGET_STATS);
  target_statistics_ = statsVectorToMsg(target_statistics);

  marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>(marker_topic_name_, topic_queue_length_);
  target_stats_publisher_ = node_handle_.advertise<agent_test::FormationStatisticsStamped>(target_stats_topic_name_, topic_queue_length_);
  stats_publisher_ = node_handle_.advertise<agent_test::FormationStatisticsArray>(received_stats_topic_name_, topic_queue_length_);
  stats_subscriber_ = node_handle_.subscribe(shared_stats_topic_name_, topic_queue_length_, &GroundStationCore::sharedStatsCallback, this);
  sync_server_ = node_handle_.advertiseService(sync_service_name_, &GroundStationCore::syncAgentCallback, this);

  waitForSync();
  number_of_agents_ = connected_agents_.size();

  agent_test::FormationStatisticsStamped msg;
  msg.header.frame_id = "target_stats";
  msg.header.stamp = ros::Time::now();
  msg.stats = target_statistics_;
  target_stats_publisher_.publish(msg);
  updateSpanningEllipse(msg);

  algorithm_timer_ = private_node_handle_->createTimer(ros::Duration(sample_time_), &GroundStationCore::algorithmCallback, this);
}

GroundStationCore::~GroundStationCore() {
  delete private_node_handle_;
}

void GroundStationCore::algorithmCallback(const ros::TimerEvent &timer_event) {
  agent_test::FormationStatisticsArray msg;
  msg.header.frame_id = ground_station_frame_;
  msg.header.stamp = ros::Time::now();
  msg.neighbours_ = connected_agents_;
  msg.vector = shared_statistics_grouped_;
  stats_publisher_.publish(msg);
  ROS_DEBUG_STREAM("[GroundStationCore::algorithmCallback] Message published.");

  // clears the private variable for following callbacks
  shared_statistics_grouped_.clear();
}

bool GroundStationCore::checkCollision(const int &id) {
  return std::find(std::begin(connected_agents_), std::end(connected_agents_), id) != std::end(connected_agents_);
}

int GroundStationCore::extractFirstID() {
  for (int i=1; i<=number_of_agents_; i++){
    if (!checkCollision(i)) {
      return i;
    }
  }
  ROS_ERROR_STREAM("[GroundStationCore::extractFirstID] Can't find a valid ID: check the current number of agents ("
                   << number_of_agents_ << ").");
  return -1;
}

void GroundStationCore::updateSpanningEllipse(const agent_test::FormationStatisticsStamped &msg) {
  std::string frame = msg.header.frame_id + "_ellipse";
  double m_xy = 2*(msg.stats.m_xy - msg.stats.m_x*msg.stats.m_y);
  double m_xx = msg.stats.m_xx - std::pow(msg.stats.m_x, 2);
  double m_yy = msg.stats.m_yy - std::pow(msg.stats.m_y, 2);

  double theta = std::atan2(m_xy, m_xx - m_yy)/2;
  double a_x = m_xx*std::pow(std::cos(theta), 2) + m_yy*std::pow(std::sin(theta), 2) + m_xy*std::sin(theta)*std::cos(theta);
  double a_y = m_yy*std::pow(std::cos(theta), 2) + m_xx*std::pow(std::sin(theta), 2) - m_xy*std::sin(theta)*std::cos(theta);

  tf::Pose pose;
  pose.setOrigin(tf::Vector3(msg.stats.m_x, msg.stats.m_y, 0));
  pose.setRotation(tf::createQuaternionFromRPY(0, 0, theta));
  tf_broadcaster_.sendTransform(tf::StampedTransform(pose, ros::Time::now(), fixed_frame_, frame));

  double diameter_x = 2*std::sqrt(number_of_agents_*std::abs(a_x));
  double diameter_y = 2*std::sqrt(number_of_agents_*std::abs(a_y));
  marker_publisher_.publish(buildMarker(diameter_x, diameter_y, frame, msg.agent_id));
}

visualization_msgs::Marker GroundStationCore::buildMarker(const double &diameter_x, const double &diameter_y,
                                                          const std::string &frame, const int &id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time(0);
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.ns = "spanning_ellipse";
  marker.id = id;
  marker.frame_locked = true;
  if (frame == "target_stats_ellipse") {
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
  // relative pose is zero: the frame is already properly centered and rotated
  marker.scale.x = (diameter_x > 0) ? diameter_x : 0.1;
  marker.scale.y = (diameter_y > 0) ? diameter_y : 0.1;
  marker.scale.z = 0.001;

  return marker;
}

void GroundStationCore::sharedStatsCallback(const agent_test::FormationStatisticsStamped &shared) {
  shared_statistics_grouped_.push_back(shared);
  updateSpanningEllipse(shared);
  ROS_DEBUG_STREAM("[GroundStationCore::sharedStatsCallback] Received statistics " << shared.header.frame_id);
}

agent_test::FormationStatistics GroundStationCore::statsVectorToMsg(const std::vector<double> &vector) {
  agent_test::FormationStatistics msg;
  if (vector.size() != 5) {
    ROS_ERROR_STREAM("[AgentCore::statsVectorToMsg] Wrong statistics vector size (" << vector.size() << ")");
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

void GroundStationCore::waitForSync() {
  while (sync_time_.isZero()) {
    ros::Duration(0.1).sleep();
  }
  ros::Time::sleepUntil(sync_time_);
}
