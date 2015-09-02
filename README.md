# Agent Test

Basic features and TODO list:

- [ ] create a basic class for agents of the swarm
  - [ ] sensor info (fake initially)
  - [ ] virtual agent (consensus + control law)
  - [ ] real/simulated agent (LOS: pursue the virtual one)
- [ ] exchange information between distinct agents (nodes)
  - [ ] communication graph
  - [ ] ground station node (TDMA protocol)
  - [ ] show the info each node knows
- [ ] implement consensus between virtual agents
- [ ] individually estimate statistics and share results

Further improvements:

- [ ] add external controller for target statistic (joystick)
- [ ] add Gazebo simulation (car URDF model + ROS control + MoveIt)