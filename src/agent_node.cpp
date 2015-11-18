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

int main(int argc, char **argv) {
  ros::init(argc, argv, "agent");

  // activates the asynchronous multi-thread spinner
  ros::AsyncSpinner spinner(3);
  spinner.start();

  AgentCore *agent = new AgentCore();

  ros::waitForShutdown();

  delete agent;
  return 0;
}
