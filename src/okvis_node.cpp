/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Mar 23, 2012
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file okvis_node.cpp
 * @brief This file includes the ROS node implementation.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <functional>
#include <iostream>
#include <fstream>
#include <stdlib.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <ros/ros.h>
#pragma GCC diagnostic pop
#include <image_transport/image_transport.h>
#include "sensor_msgs/Imu.h"

#include <glog/logging.h>

#include <okvis/Subscriber.hpp>
#include <okvis/Publisher.hpp>
#include <okvis/RosParametersReader.hpp>
#include <okvis/ThreadedKFVio.hpp>
#include <okvis/Player.hpp>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "okvis_node");

  // set up the node
  ros::NodeHandle nh("okvis_node");

  // initialise logging
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0; // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  // publisher
  okvis::Publisher publisher(nh);

  // read configuration file
  std::string configFilename;
  if(argc==2)
  {
      configFilename = argv[1];
  }else{
      std::cout<<"You can invoke okvis_node through either a ros launch file or a command line argument."<< std::endl<<
               "In the former case, the dataset is in a rosbag, and the arguments are put in a roslaunch file,"<<std::endl<<
                  "i.e., okvis_ros/launch/okvis_node.launch. Change these parameters according to your dataset"<< std::endl<<
                  "Then in a terminal, execute "<< std::endl<< "roslaunch okvis_ros okvis_node.launch"<<std::endl<<std::endl;
      std::cout<<"In the latter case, config_filename needs to be provided to okvis_node either in the command line"<<std::endl<<
                  " or use rosparam after okvis_node is invoked, e.g.,"<<std::endl<<"rosparam set /okvis_node/config_filename "<<
                  "/path/to/config/config_fpga_p2_euroc.yaml"<< std::endl<<std::endl<<
                 "For now, the preferred approach is to provide the config_filename as a command line argument, e.g., "<<std::endl<<
                 "rosrun okvis_ros okvis_node path/to/okvis_ros/okvis/config/config_fpga_p2_euroc.yaml."<<std::endl<<std::endl;

      std::cout<<"If the data is played out from a rosbag file, it may be needed to remap the topics within the rosbag."<< std::endl
              << "To do so, for instance, in the terminal, enter"<< std::endl<<
                 "rosbag play --pause MH_01_easy.bag /cam0/image_raw:=/camera0 /imu0:=/imu"<<std::endl<<std::endl;
      std::cout<<"to visualize it in RVIZ "<< std::endl<<
                 "rosrun rviz rviz -d config/rviz.rviz"<<std::endl;

      if(!nh.getParam("config_filename",configFilename)){
          LOG(ERROR) << "Please specify filename of configuration!";
          return 1;
      }
  }
  okvis::RosParametersReader vio_parameters_reader(configFilename);
  okvis::VioParameters parameters;
  vio_parameters_reader.getParameters(parameters);

  okvis::ThreadedKFVio okvis_estimator(parameters);

#define SAVE_TO_FILE 1
/// set publishing_options.publishImuPropagatedState to false in the settings.yaml to only save optimized states
#if SAVE_TO_FILE
  std::string path = parameters.publishing.outputPath;

  // setup files to be written  
  publisher.setCsvFile(path + "/okvis_estimator_output.csv");
  /// method 1 to save estimates
//  okvis_estimator.setFullStateCallback(std::bind(&okvis::Publisher::csvSaveFullStateAsCallback, &publisher,
//                                                 std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,
//                                                 std::placeholders::_4, std::placeholders::_5));
  /// method 2 to save estimates
  okvis_estimator.setFullStateCallbackWithExtrinsics(std::bind(&okvis::Publisher::csvSaveFullStateWithExtrinsicsAsCallback, &publisher,
                                                 std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,
                                                 std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));

//  okvis_estimator.setImuCsvFile(path + "/imu0_data.csv");
//  const unsigned int numCameras = parameters.nCameraSystem.numCameras();
//  for (size_t i = 0; i < numCameras; ++i) {
//    std::stringstream num;
//    num << i;
//    okvis_estimator.setTracksCsvFile(i, path + "/cam" + num.str() + "_tracks.csv");
//  }

//  publisher.setLandmarksCsvFile(path + "/okvis_estimator_landmarks.csv");
//  okvis_estimator.setLandmarksCallback(std::bind(&okvis::Publisher::csvSaveLandmarksAsCallback,&publisher,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));

#else
  okvis_estimator.setFullStateCallback(std::bind(&okvis::Publisher::publishFullStateAsCallback,&publisher,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4));
  okvis_estimator.setLandmarksCallback(std::bind(&okvis::Publisher::publishLandmarksAsCallback,&publisher,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
#endif

  okvis_estimator.setStateCallback(std::bind(&okvis::Publisher::publishStateAsCallback,&publisher,std::placeholders::_1,std::placeholders::_2));
  publisher.setParameters(parameters); // pass the specified publishing stuff


#if 0       ///Method 1 subscriber to grab messages from rostopics
  okvis::Subscriber subscriber(nh, &okvis_estimator, vio_parameters_reader);
#else       ///Method 2 player to grab messages directly from files on a hard drive
  std::shared_ptr<okvis::Player> pPlayer;
  std::shared_ptr<std::thread> ptPlayer;
  if(parameters.input.videoFile.empty()){//image sequence input
    pPlayer.reset(new okvis::Player(&okvis_estimator, parameters, std::string()));
    ptPlayer.reset(new std::thread(&okvis::Player::Run, std::ref(*pPlayer)));
  }
  else//video input
  {
    pPlayer.reset(new okvis::Player(&okvis_estimator, parameters));
    ptPlayer.reset(new std::thread(&okvis::Player::Run, std::ref(*pPlayer)));
  }
#endif

  ros::Rate rate(20);
  while (ros::ok()) {
    ros::spinOnce();
    okvis_estimator.display();
    rate.sleep();
  }

  return 0;
}
