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

#ifndef GUARD_COMMONS_H
#define GUARD_COMMONS_H

// standard libraries
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>
#include <map>
#include <random>
#include <algorithm>
#include <mutex>
// ROS libraries
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <angles/angles.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
// auto-generated from ./msg directory libraries
#include <formation_control/FormationStatistics.h>
#include <formation_control/FormationStatisticsStamped.h>

// license info to be displayed at the beginning
#define LICENSE_INFO "\n*\n* Copyright (C) 2015 Alessandro Tondo\n* This program comes with ABSOLUTELY NO WARRANTY.\n* This is free software, and you are welcome to\n* redistribute it under GNU GPL v3.0 conditions.\n* (for details see <http://www.gnu.org/licenses/>).\n*\n\n"
// default values for ROS params (if not specified by the user) for both AgentCore and GroundStationCore classes
#define DEFAULT_SAMPLE_TIME 0.1  // expressed in seconds
#define DEFAULT_SLOT_TDMA 0.01  // duration of each tdma slot (expressed in seconds)
#define DEFAULT_NUMBER_OF_AGENTS 9
#define DEFAULT_NUMBER_OF_STATS 5  // see FormationStatistics.msg (mx, my, mxx, mxy, myy)
#define DEFAULT_NUMBER_OF_VELOCITIES 2  // virtual planar linear twist (virtual_x_dot, virutal_y_dot)
#define DEFAULT_VERBOSITY_LEVEL 1
#define DEFAULT_TOPIC_QUEUE_LENGTH 1
#define DEFAULT_SHARED_STATS_TOPIC "shared_stats"
#define DEFAULT_TARGET_STATS_TOPIC "target_stats"
#define DEFAULT_AGENT_POSES_TOPIC "agent_poses"
#define DEFAULT_MATLAB_POSES_TOPIC "matlab_poses"
#define DEFAULT_MARKER_TOPIC "visualization_marker"
#define DEFAULT_SYNC_SERVICE "sync_agent"
#define DEFAULT_SYNC_DELAY 5.0  // expressed in seconds
#define DEFAULT_FRAME_MAP "map"
#define DEFAULT_FRAME_AGENT_PREFIX "agent_"
#define DEFAULT_FRAME_EFFECTIVE_PREFIX "effective_stats"
#define DEFAULT_FRAME_ELLIPSE_SUFFIX "_ellipse"
#define DEFAULT_FRAME_VIRTUAL_SUFFIX "_virtual"
#define DEFAULT_FRAME_GROUND_STATION "ground_station"
// console output level
#define FATAL -3
#define ERROR -2
#define WARN -1
#define INFO 0
#define DEBUG 1
#define DEBUG_V 2
#define DEBUG_VV 3
#define DEBUG_VVV 4
#define DEBUG_VVVV 5

#endif
