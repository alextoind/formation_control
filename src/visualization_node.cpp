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

#include "visualization_core.h"

int main(int argc, char **argv) {
  std::cout << LICENSE_INFO << std::flush;

  ros::init(argc, argv, "visualization");

  // activates the asynchronous multi-thread spinner
  ros::AsyncSpinner spinner(2);
  spinner.start();

  VisualizationCore *visualization = new VisualizationCore();

  ros::waitForShutdown();

  delete visualization;
  return 0;
}
