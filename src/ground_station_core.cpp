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

  private_node_handle_->param("sample_time", sample_time_, (double)DEFAULT_SAMPLE_TIME);
  private_node_handle_->param("topic_queue_length", topic_queue_length_, DEFAULT_TOPIC_QUEUE_LENGTH);
  private_node_handle_->param("shared_stats_topic", shared_stats_topic_name_, std::string(DEFAULT_SHARED_STATS_TOPIC));
  private_node_handle_->param("received_stats_topic", received_stats_topic_name_, std::string(DEFAULT_RECEIVED_STATS_TOPIC));
  private_node_handle_->param("sync_service_name", sync_service_name_, std::string(DEFAULT_SYNC_SERVICE_NAME));
  private_node_handle_->param("ground_station_id", ground_station_id_, std::string(DEFAULT_GROUND_STATION_ID));
  private_node_handle_->param("number_of_agents", number_of_agents_, DEFAULT_NUMBER_OF_AGENTS);
  double sync_delay;
  private_node_handle_->param("sync_delay", sync_delay, (double)DEFAULT_SYNC_DELAY);
  sync_delay_ = ros::Duration(sync_delay);

  stats_publisher_ = node_handle_.advertise<agent_test::FormationStatisticsArray>(received_stats_topic_name_, topic_queue_length_);
  stats_subscriber_ = node_handle_.subscribe(shared_stats_topic_name_, topic_queue_length_, &GroundStationCore::sharedStatsCallback, this);
  sync_server_ = node_handle_.advertiseService(sync_service_name_, &GroundStationCore::syncAgentCallback, this);
  waitForSync();
  algorithm_timer_ = private_node_handle_->createTimer(ros::Duration(sample_time_), &GroundStationCore::algorithmCallback, this);
}

GroundStationCore::~GroundStationCore() {
  delete private_node_handle_;
}

void GroundStationCore::algorithmCallback(const ros::TimerEvent &timer_event) {
  agent_test::FormationStatisticsArray msg;
  msg.header.frame_id = ground_station_id_;
  msg.header.stamp = ros::Time::now();
  msg.neighbours_ = connected_agents_;
  msg.vector = shared_statistics_grouped_;
  stats_publisher_.publish(msg);
  ROS_DEBUG_STREAM("[GroundStationCore::algorithmCallback] Message published.");

  // clears the private variable for following callbacks
  shared_statistics_grouped_.clear();
}

bool GroundStationCore::checkCollision(const int &id) {
  if (std::find(std::begin(connected_agents_), std::end(connected_agents_), id) != std::end(connected_agents_)) {
    return true;
  }
  return false;
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

void GroundStationCore::sharedStatsCallback(const agent_test::FormationStatisticsStamped &shared) {
  shared_statistics_grouped_.push_back(shared);
  ROS_DEBUG_STREAM("[GroundStationCore::sharedStatsCallback] Received statistics from agent id " << shared.header.frame_id);
}

agent_test::FormationStatisticsStamped GroundStationCore::statsVectorToMsg(const int &id, const std::vector<double> &vector) {
  agent_test::FormationStatisticsStamped msg;
  if (vector.size() != 5) {
    ROS_ERROR_STREAM("[AgentCore::statsVectorToMsg] Wrong statistics vector size (" << vector.size() << ")");
    return msg;
  }
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = std::to_string(id);
  msg.stats.m_x = vector.at(0);
  msg.stats.m_y = vector.at(1);
  msg.stats.m_xx = vector.at(2);
  msg.stats.m_xy = vector.at(3);
  msg.stats.m_yy = vector.at(4);

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
  shared_statistics_grouped_.push_back(statsVectorToMsg(response.new_id, std::vector<double>({0,0,0,0,0})));
  return true;
}

void GroundStationCore::waitForSync() {
  while (sync_time_.isZero()) {
    ros::Duration(0.1).sleep();
  }
  ros::Time::sleepUntil(sync_time_);
}
