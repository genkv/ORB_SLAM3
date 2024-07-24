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

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>

#include <json.h>

using namespace std;
using nlohmann::json;
const double MS_TO_S = 1e-3; ///< Milliseconds to second conversion

bool LoadTelemetry(const string &path_to_telemetry_file,
                   vector<double> &vTimeStamps,
                   vector<double> &coriTimeStamps,
                   vector<cv::Point3f> &vAcc,
                   vector<cv::Point3f> &vGyro) {

    std::ifstream file;
    file.open(path_to_telemetry_file.c_str());
    if (!file.is_open()) {
      return false;
    }
    json j;
    file >> j;
    const auto accl = j["1"]["streams"]["ACCL"]["samples"];
    const auto gyro = j["1"]["streams"]["GYRO"]["samples"];
    const auto gps5 = j["1"]["streams"]["GPS5"]["samples"];
    const auto cori = j["1"]["streams"]["CORI"]["samples"];
    std::map<double, cv::Point3f> sorted_acc;
    std::map<double, cv::Point3f> sorted_gyr;

    for (const auto &e : accl) {
      cv::Point3f v((float)e["value"][1], (float)e["value"][2], (float)e["value"][0]);
      sorted_acc.insert(std::make_pair((double)e["cts"] * MS_TO_S, v));
    }
    for (const auto &e : gyro) {
      cv::Point3f v((float)e["value"][1], (float)e["value"][2], (float)e["value"][0]);
      sorted_gyr.insert(std::make_pair((double)e["cts"] * MS_TO_S, v));
    }
//    for (const auto &e : gps5) {
//      Eigen::Vector3d v;
//      Eigen::Vector2d vel2d_vel3d;
//      v << e["value"][0], e["value"][1], e["value"][2];
//      vel2d_vel3d << e["value"][3], e["value"][4];
//      telemetry.gps.lle.emplace_back(v);
//      telemetry.gps.timestamp_ms.emplace_back(e["cts"]);
//      telemetry.gps.precision.emplace_back(e["precision"]);
//      telemetry.gps.vel2d_vel3d.emplace_back(vel2d_vel3d);
//    }

    double imu_start_t = sorted_acc.begin()->first;
    for (auto acc : sorted_acc) {
        vTimeStamps.push_back(acc.first-imu_start_t);
        vAcc.push_back(acc.second);
    }
    for (auto gyr : sorted_gyr) {
        vGyro.push_back(gyr.second);
    }
    for (const auto &e : cori) {
        coriTimeStamps.push_back((double)e["cts"] * MS_TO_S);
    }

    file.close();
    return true;
}

void draw_gripper_mask(cv::Mat &img){
  // arguments
  double height = 0.37;
  double top_width = 0.25;
  double bottom_width = 1.4;

  // image size
  double img_h = img.rows;
  double img_w = img.cols;

  // calculate coordinates
  double top_y = 1. - height;
  double bottom_y = 1.;
  double width = img_w / img_h;
  double middle_x = width / 2.;
  double top_left_x = middle_x - top_width / 2.;
  double top_right_x = middle_x + top_width / 2.;
  double bottom_left_x = middle_x - bottom_width / 2.;
  double bottom_right_x = middle_x + bottom_width / 2.;

  top_y *= img_h;
  bottom_y *= img_h;
  top_left_x *= img_h;
  top_right_x *= img_h;
  bottom_left_x *= img_h;
  bottom_right_x *= img_h;

  // create polygon points for opencv API
  std::vector<cv::Point> points;
  points.emplace_back(bottom_left_x, bottom_y);
  points.emplace_back(top_left_x, top_y);
  points.emplace_back(top_right_x, top_y);
  points.emplace_back(bottom_right_x, bottom_y);

  std::vector<std::vector<cv::Point> > polygons;
  polygons.push_back(points);

  // draw
  cv::fillPoly(img, polygons, cv::Scalar(0));
}


int main(int argc, char **argv) {
  if (argc != 5) {
    cerr << endl
         << "Usage: ./mono_inertial_gopro_vi path_to_vocabulary path_to_settings path_to_video path_to_telemetry"
         << endl;
    return 1;
  }

  vector<double> imuTimestamps;
  vector<double> camTimestamps;
  vector<cv::Point3f> vAcc, vGyr;
  LoadTelemetry(argv[4], imuTimestamps, camTimestamps, vAcc, vGyr);

  // open settings to get image resolution
  cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
  if(!fsSettings.isOpened()) {
     cerr << "Failed to open settings file at: " << argv[2] << endl;
     exit(-1);
  }
  cv::Size img_size(fsSettings["Camera.width"],fsSettings["Camera.height"]);
  fsSettings.release();

  // Retrieve paths to images
  vector<double> vTimestamps;
  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  cv::VideoCapture cap(argv[3]);
  // Check if camera opened successfully
  if (!cap.isOpened()) {
    std::cout << "Error opening video stream or file" << endl;
    return -1;
  }

  // Main loop
  int cnt_empty_frame = 0;
  int img_id = 0;
  int nImages = cap.get(cv::CAP_PROP_FRAME_COUNT);
  double fps = cap.get(cv::CAP_PROP_FPS);
  double frame_diff_s = 1./fps;
  double prev_tframe = -100.;
  std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
  size_t last_imu_idx = 0;
  while (1) {
    cv::Mat im,im_track;
    bool success = cap.read(im);

    if (!success) {
      break;
      // cnt_empty_frame++;
      // // std::cout<<"Empty frame...\n";
      // if (cnt_empty_frame > 1000)
      //   break;
      // continue;
    }
      im_track = im.clone();
      double tframe = cap.get(cv::CAP_PROP_POS_MSEC) * MS_TO_S;

      // tframe goes to 0 sometimes after video ends;
      if (tframe < prev_tframe) {
        break;
      }
      prev_tframe = tframe;

      // double tframe = camTimestamps[img_id];
      ++img_id;

      cv::resize(im_track, im_track, img_size);
      draw_gripper_mask(im_track);

      // gather imu measurements between frames
      // Load imu measurements from previous frame
      vImuMeas.clear();
      while(imuTimestamps[last_imu_idx] <= tframe && tframe > 0)
      {
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[last_imu_idx].x,vAcc[last_imu_idx].y,vAcc[last_imu_idx].z,
                                                   vGyr[last_imu_idx].x,vGyr[last_imu_idx].y,vGyr[last_imu_idx].z,
                                                   imuTimestamps[last_imu_idx]));
          last_imu_idx++;
      }


#ifdef COMPILEDWITHC14
      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t1 =
          std::chrono::monotonic_clock::now();
#endif

      // Pass the image to the SLAM system
      SLAM.TrackMonocular(im_track, tframe, vImuMeas);

#ifdef COMPILEDWITHC14
      std::chrono::steady_clock::time_point t2 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t2 =
          std::chrono::monotonic_clock::now();
#endif

      double ttrack =
          std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
              .count();

      if (img_id % 100 == 0) {
        std::cout<<"Video FPS: "<<1./frame_diff_s<<"\n";
        std::cout<<"ORB-SLAM 3 running at: "<<1./ttrack<< " FPS\n";
      }
      vTimesTrack.push_back(ttrack);

      // Wait to load the next frame
      if (ttrack < frame_diff_s)
        usleep((frame_diff_s - ttrack) * 1e6);
  }

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (auto ni = 0; ni < vTimestamps.size(); ni++) {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
  cout << "mean tracking time: " << totaltime / nImages << endl;

  // Save camera trajectory
  // SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
  // SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
  SLAM.SaveTrajectoryTUM("TUM_CameraTrajectory.txt");
  SLAM.SaveKeyFrameTrajectoryTUM("TUM_KeyFrameTrajectory.txt");

  return 0;
}


