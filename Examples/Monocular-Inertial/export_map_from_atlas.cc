/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include <System.h>

#include <json.h>

using namespace std;
using nlohmann::json;

int main(int argc, char **argv) {
  if (argc != 3) {
    cerr << endl
         << "Usage: ./mono_inertial_gopro_vi path_to_vocabulary path_to_settings"
         << endl;
    return 1;
  }

  // Retrieve paths to images
  vector<double> vTimestamps;
  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, false);

  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  SLAM.SaveTrajectoryEuRoC("EuRoC_CameraTrajectory.txt");
  SLAM.SaveKeyFrameTrajectoryEuRoC("EuRoC_KeyFrameTrajectory.txt");

  SLAM.SaveTrajectoryTUM("TUM_CameraTrajectory.txt");
  SLAM.SaveKeyFrameTrajectoryTUM("TUM_KeyFrameTrajectory.txt");

  return 0;
}
