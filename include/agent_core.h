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

#ifndef GUARD_AGENT_CORE_H
#define GUARD_AGENT_CORE_H

#include "commons.h"
// default values for ROS params (if not specified by the user)
#define DEFAULT_AGENT_ID 0  // if not set by the user, the Ground Station will choose an unique value
#define DEFAULT_VELOCITY_VIRTUAL_THRESHOLD 4.0  // expressed in meters/second
#define DEFAULT_SPEED_MIN 0.0  // expressed in meters/second
#define DEFAULT_SPEED_MAX 4.0  // expressed in meters/second
#define DEFAULT_STEER_MIN -0.25  // expressed in radians
#define DEFAULT_STEER_MAX 0.25  // expressed in radians
#define DEFAULT_K_P_SPEED 0.5
#define DEFAULT_K_P_STEER 2.0
#define DEFAULT_VEHICLE_LENGTH 0.4  // expressed in meters
#define DEFAULT_WORLD_LIMIT 1.0  // in meters, considering a "square world" (only for random pose generation)
#define DEFAULT_MARKER_PATH_LIFETIME 30  // expressed in seconds

/*  This class purpose is to provide a ROS interface which lets to simulate a multi-agent completely distribute
 *  consensus and abstraction based control algorithm. Each object (i.e. each ROS node) represents an independent
 *  agent which shares its estimated statistics with all the others and nothing more (e.g. not its pose) and updates
 *  its estimate statistics based on the estimates of the other agents and on its own pose. Target statistics are
 *  provided to all the agents in the form of 5 floats (first and second order momentum) representing an oriented
 *  ellipsoid in the 2D space. It has to be notice that to get satisfactory results the initial distribution of the
 *  agents has to be in the proximity of the initial target ellipsoid. Furthermore the id and the pose of each agent
 *  has to be provided with the specific ROS params (as many other settings) on node initialization (using roslaunch
 *  or rosrun commands): the agent id must be a unique number, especially when using also real vehicles connected
 *  with the simulated ones through the ROS implementation of the Packet Manager.
 *
 *  For more info on this class usage, check the README.md in the package folder.
 *
 *  If you edit this class, please try to follow these C++ style guidelines: http://wiki.ros.org/CppStyleGuide.
 *
 *  ROS params:
 *    + sample_time
 *    + slot_tdma
 *    + agent_id
 *    + number_of_stats
 *    + number_of_velocities
 *    + verbosity_level
 *    + velocity_virtual_threshold
 *    + speed_min
 *    + speed_max
 *    + steer_min
 *    + steer_max
 *    + k_p_speed
 *    + k_p_steer
 *    + vehicle_length
 *    + world_limit
 *    + diag_elements_gamma
 *    + diag_elements_lambda
 *    + diag_elements_b
 *    + x
 *    + y
 *    + theta
 *    + topic_queue_length
 *    + shared_stats_topic
 *    + target_stats_topic
 *    + marker_topic
 *    + marker_path_lifetime
 *    + enable_path
 *    + frame_map
 *    + frame_agent_prefix
 *    + frame_virtual_suffix
 */
class AgentCore {
 public:
  /*  The constructor retrieves the agent parameters from ROS params if specified by the user (default values
   *  otherwise), initializes all the structures for the algorithm (e.g. estimated_statistics_) and the publishers
   *  and subscribers needed, and finally waits for the initial TDMA slot (sample_time_ dependent).
   *
   *  Other methods called:
   *    + setTheta
   *    + statsVectorToMsg
   *    + waitForSlotTDMA
   */
  AgentCore();
  ~AgentCore();

 private:
  ros::NodeHandle node_handle_;
  ros::NodeHandle *private_node_handle_;
  ros::Publisher stats_publisher_;
  ros::Publisher marker_publisher_;
  ros::Subscriber stats_subscriber_;
  ros::Subscriber target_stats_subscriber_;
  ros::Timer algorithm_timer_;
  tf::TransformBroadcaster tf_broadcaster_;

  bool enable_path_;
  int marker_path_id_;
  int marker_path_lifetime_;
  std::string frame_map_;
  std::string frame_agent_prefix_;
  std::string frame_virtual_suffix_;
  std::string agent_frame_;
  std::string agent_virtual_frame_;

  int topic_queue_length_;
  std::string shared_stats_topic_name_;
  std::string target_stats_topic_name_;
  std::string marker_topic_name_;
  double sample_time_;
  double slot_tdma_;

  int agent_id_;  // it must be set with a unique value among all agents
  geometry_msgs::Pose pose_;
  geometry_msgs::Pose pose_virtual_;
  geometry_msgs::Twist twist_;
  geometry_msgs::Twist twist_virtual_;
  formation_control::FormationStatistics target_statistics_;
  formation_control::FormationStatistics estimated_statistics_;
  std::map<int, formation_control::FormationStatistics> received_statistics_;
  std::mutex received_statistics_mutex_;

  int number_of_stats_;
  int number_of_velocities_;
  // consensus
  Eigen::RowVectorXd phi_dot_;
  // control law
  Eigen::MatrixXd gamma_;
  Eigen::MatrixXd lambda_;
  Eigen::MatrixXd b_;
  Eigen::MatrixXd jacob_phi_;

  double velocity_virtual_threshold_;
  double speed_min_;
  double speed_max_;
  double steer_min_;
  double steer_max_;

  double speed_command_sat_;
  double steer_command_sat_;
  double k_p_speed_;
  double k_p_steer_;

  double vehicle_length_;
  double world_limit_;

  int verbosity_level_;

  /*  Creates a new marker LINE_STRIP which links the two poses passed as parameters, with the aim to show the path
   *  of the agent from the pose k-1 to the current pose k. The marker lays in a namespace which depends on the given
   *  frame parameter, which is also used to vary the marker color (this is done to distinguish between real and
   *  virtual agent paths). The marker as a specified lifetime to reproduce a fading effect of the oldest samples.
   *
   *  Parameters:
   *    + p_old: last pose of the agent;
   *    + p_new: current pose of the agent;
   *    + frame: specifies the agent id and distinguishes between real and virtual paths (e.g. agent_1_virtual).
   *  Return value:
   *    + LINE_STRIP marker with ADD property and a unique pair <namespace, id>.
   */
  visualization_msgs::Marker addToMarkerPath(const geometry_msgs::Point &p_old, const geometry_msgs::Point &p_new,
                                             const std::string &frame);

  /*  This is the main method of the algorithm and it is automatically called by a timer event every sample_time_.
   *  Every single time it calls in order the core methods: those for the virtual agent (consensus and control) and
   *  those for the real one (guidance and dynamics). At the end it waits for the proper TDMA slot (agent_id_
   *  dependent) and publishes its estimated statistics in the shared topic.
   *
   *  Parameters:
   *    + timer_event: a ros::TimerEvent variable automatically filled by ROS (not used, but necessary).
   *  Other methods called:
   *    + consensus
   *    + control
   *    + dynamics
   *    + guidance
   *    + waitForSlotTDMA
   */
  void algorithmCallback(const ros::TimerEvent &timer_event);

  /*  If path visualization is enabled (settable through a ROS param) it publishes a new marker representing the
   *  updated path for the virtual or real agent in a predefined topic (settable through another ROS param).
   *
   *  Parameters:
   *    + pose_new: current pose of the agent;
   *    + pose_old: last pose of the agent;
   *    + frame: specifies the agent id and distinguishes between real and virtual paths (e.g. agent_1_virtual).
   *  Other methods called:
   *    + addToMarkerPath
   */
  void broadcastPath(const geometry_msgs::Pose &pose_new, const geometry_msgs::Pose &pose_old, const std::string &frame);

  /*  Broadcasts the (current) pose of the given frame (virtual or real agent) to the TF ROS environment.
   *
   *  Parameters:
   *    + pose: current pose of the agent;
   *    + frame: specifies the agent id and distinguishes between real and virtual paths (e.g. agent_1_virtual).
   */
  void broadcastPose(const geometry_msgs::Pose &pose, const std::string &frame);

  /*  Computes the current estimate of statistics based on a dynamic discrete consensus algorithm: it updates the
   *  previous estimate using the current pose and twist of the virtual agent and the estimated statistics received
   *  from all the other agents (which appear in the Laplacian matrix L). Brefly: x_k+1 = phi_dot_k*Ts + (I - Ts*L)x_k, 
   *  where phi_dot is the (analytic) time derivative of phi = [px, py, pxx, pxy, pyy]. In addition, there is a check
   *  on the sample time which must be smaller enough to guarantee the convergence (depends on the number of agents).
   *
   *  Other methods called:
   *    + statsMsgToMatrix
   *    + statsMsgToVector
   *    + statsVectorToMsg
   */
  void consensus();

  /*  Displays a message with a specific log level, using the proper ROS MACRO. This method is used by all the others
   *  to show errors, warnings and useful info about the state of the node in a homogeneous format; it provides also
   *  five distinct verbosity levels for debug info (e.g. to investigate specific variables and the flow of the code).
   *
   *  Parameters:
   *    + caller_name: name of the method which calls this one;
   *    + message: stream to be sent to the console;
   *    + log_level: integer in range [-3, 5] respectively from fatal to very verbose debug messages.
   */
  void console(const std::string &caller_name, std::stringstream &message, const int &log_level) const;

  /*  Computes a simple control action for the virtual agent based on the (analytic) Jacobian matrix of phi and the
   *  estimation error w.r.t. the target statistics: control_twist = inv(B + Jphi'*lambda*Jphi)*Jphi'*gamma*stats_error,
   *  where phi = [px, py, pxx, pxy, pyy] and B(2x2), lambda(5x5) and gamma(5x5) are diagonal square matrices which
   *  can be tuned by the user using the given ROS params. The control action is then saturated with a threshold (also
   *  tunable with a ROS param) and the pose and twist of the virtual agent are updated properly; lastly, the pose and
   *  the new segment of the path (from the previous pose to the current one) are broadcasted for visualization in rviz
   *  respectively to the TF framework and to a specific marker topic (whose name can be set with another ROS param).
   *
   *  Other methods called:
   *    + broadcastPath
   *    + broadcastPose
   *    + integrator
   *    + setTheta
   *    + statsMsgToVector
   */
  void control();

  /*  Computes the dynamics of a simplified simulated 4-wheel vehicle using speed and steer commands and only the
   *  legnth between the the front and rear axes (which is adjustable through a ROS param). Pose and twist of the
   *  simulated agent are then updated and the pose and the new segment of the path (from the previous pose to the
   *  current one) are broadcasted for visualization in rviz respectively to the TF framework and to a specific
   *  marker topic (whose name can be set with another ROS param).
   *
   *  Other methods called:
   *    + broadcastPath
   *    + broadcastPose
   *    + getTheta
   *    + integrator
   *    + setTheta
   */
  void dynamics();

  /*  Updates the vaule passed by reference with its floored value to the N-th decimals.
   *
   *  Parameters:
   *    + d: updated value passed by reference (floor(d*10^N)/10^N).
   *    + precision: number of decimals .
   */
  void floor(double &d, const int &precision) const;

  /*  Returns the roll pitch and yaw angles of the given quaternion.
   *
   *  Parameters:
   *    + quat: quaternion representing a pose::orientation.
   *  Return value:
   *    + vector containing roll pitch and yaw angles in radians.
   */
  Eigen::Vector3d getRPY(const geometry_msgs::Quaternion &quat) const;

  /*  Returns the z-axis orientation of the given quaternion (we assume agents motion in the 2D xy plane).
   *
   *  Parameters:
   *    + quat: quaternion representing a pose::orientation.
   *  Other methods called:
   *    + getRPY
   *  Return value:
   *    + z-axis orientation in radians.
   */
  double getTheta(const geometry_msgs::Quaternion &quat) const;

  /*  Computes a simple LOS guidance for the simulated agent with the aim to pursuit the virtual one: LOS distance and
   *  LOS angle are evaluated from the knowledge of the poses of the two agents; then the speed and the steer commands
   *  for the simulated agent come from two basic proportional controllers whose gains are tunable using ROS params.
   *  Both these commands are saturated with thresholds (also adjustable with ROS params) for plausibility.
   *
   *  Other methods called:
   *    + saturation
   */
  void guidance();

  /*  Computes the integration following the Tustin (trapezoidal) formulation: out_k = out_k-1 + KT(in_k-1 + in_k)/2,
   *  where T is the sample_time_ and the other variables are the given parameters.
   *
   *  Parameters:
   *    + out_old: previous output.
   *    + in_old: previous input.
   *    + in_new: current input (new measure).
   *    + k: gain.
   *  Return value:
   *    + integrated value.
   */
  double integrator(const double &out_old, const double &in_old, const double &in_new, const double &k) const;

  /*  Unless the message recived from the shared topic has the same id of the receiver (i.e. a message previously
   *  sent by itself), it stores the data received in a map using as hash key the sender id. Note that it is necessary
   *  to use a mutex protection on the map because it is shared among threads.
   *
   *  Parameters:
   *    + received: a ROS custom message which carries the estimated statistics of a certain agent.
   */
  void receivedStatsCallback(const formation_control::FormationStatisticsStamped &received);

  /*  Computes the saturation of the given value w.r.t. the provided thresholds.
   *
   *  Parameters:
   *    + value: value to be processed;
   *    + min: lower bound threshold;
   *    + max: upper bound threshold.
   *  Return value:
   *    + saturated value.
   */
  double saturation(const double &value, const double &min, const double &max) const;

  /*  Updates the z-axis orientation of the given quaternion with the provided value.
   *
   *  Parameters:
   *    + quat: updated quaternion passed by reference;
   *    + theta: new value of orientation w.r.t. the z-axis.
   *  Other methods called:
   *    + getRPY
   */
  void setTheta(geometry_msgs::Quaternion &quat, const double &theta) const;

  /*  Convetrs a set of statistics stored in formation_control::FormationStatistics ROS messages to a Eigen::MatrixXd
   *  data matrix.
   *
   *  Parameters:
   *    + msg: each pair of the map is composed by an agent id and its estimated statistics.
   *  Other methods called:
   *    + statsMsgToVector
   *  Return value:
   *    + matrix filled with the given values (each row is a distinct statistic).
   */
  Eigen::MatrixXd statsMsgToMatrix(const std::map<int, formation_control::FormationStatistics> &msg) const;

  /*  Convetrs statistics from formation_control::FormationStatistics ROS message to Eigen::VectorXd data vector.
   *
   *  Parameters:
   *    + msg: structure containing the statistics (mx, my, mxx, mxy, myy).
   *  Return value:
   *    + vector filled with the given values.
   */
  Eigen::VectorXd statsMsgToVector(const formation_control::FormationStatistics &msg) const;

  /*  Convetrs statistics from Eigen::VectorXd data vector to formation_control::FormationStatistics ROS message.
   *
   *  Parameters:
   *    + vector: data is stored in the following order (mx, my, mxx, mxy, myy).
   *  Return value:
   *    + message filled with the given values.
   */
  formation_control::FormationStatistics statsVectorToMsg(const Eigen::VectorXd &vector) const;

  /*  Convetrs statistics from std::vector<double> data vector to formation_control::FormationStatistics ROS message.
   *
   *  Parameters:
   *    + vector: data is stored in the following order (mx, my, mxx, mxy, myy).
   *  Other methods called:
   *    + statsVectorToMsg
   *  Return value:
   *    + message filled with the given values.
   */
  formation_control::FormationStatistics statsVectorToMsg(const std::vector<double> &vector) const;

  /*  It is called every time a new target statistics has been published to a predefined topic (settable through a
   *  ROS param) and updates the private variable target_statistics_ with the new given target.
   *
   *  Parameters:
   *    + target: new target statistics.
   */
  void targetStatsCallback(const formation_control::FormationStatisticsStamped &target);
  
  /*  Computes the proper TDMA slot based on the given deadline and sleeps the thread until it has been reached.
   *  To achieve this it rounds to the current exact decisecond (hhmmss:x00) and adds the given deadline expressed in
   *  seconds, which could be based on the agent id (e.g. agent N speaks in the N+1th slot). The first slot is used
   *  for the algorithm computation by all the robots simultaneously; this ensures that all the agents use the same
   *  shared estimations.
   *
   *  Parameters:
   *    + deadline: time expressed in seconds to be added to the current time rounded to hhmmss:x00.
   */
  void waitForSlotTDMA(const double &deadline) const;
};

#endif
