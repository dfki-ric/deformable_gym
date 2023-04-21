/*
* Copyright (C) 2021 Prensilia s.r.l.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <vector>

#include <iostream>
#include <ros/ros.h>
#include <mia_hand_ros_control/mia_hw_interface.h>
#include <controller_manager/controller_manager.h>


/**
 * Mia Hand hardware control loop
 * Main of the ROS node used to control the real Mia hand
 */
int main(int argc, char **argv)
{


   ros::init(argc, argv, "Mia_hw_node");


  // Get frequency as parameter
  int fs_rate;
  if (ros::param::has("~Mia_fs_"))
  {
    ros::param::get("~Mia_fs_", fs_rate);
  }
  else
  {
    fs_rate = 20; // default value
    ros::param::set("~Mia_fs_", fs_rate);

  }

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  mia_hand::MiaHWInterface mia_hw;

  bool init_success = mia_hw.init(nh,nh);

  if (!init_success)
  {
  	   ROS_ERROR_NAMED("Mia_hw_node", "Error initializing mia_hw_interface.\n");
         ros::shutdown();
  }

  controller_manager::ControllerManager cm(&mia_hw,nh);

  //ros::Rate rate(20); // 20Hz update rate

  ros::Rate rate(fs_rate); // 20Hz update rate

  ROS_INFO_STREAM("Mia_hw_interface started with frequency "<<fs_rate<< "Hz");

  bool was_connected_ = false;
  bool is_connected_ = false;
  // Start control loop
  while(ros::ok())
  {
    is_connected_ = mia_hw.checkConnection();

    if(is_connected_)
    {
      if(!was_connected_)
      {
        cm.update (ros::Time::now(), rate.expectedCycleTime(),true);
        ROS_INFO_STREAM("Mia Re-connected");
      }

      mia_hw.read (ros::Time::now(), rate.expectedCycleTime());
      cm.update   (ros::Time::now(), rate.expectedCycleTime());
      mia_hw.write(ros::Time::now(), rate.expectedCycleTime());
      rate.sleep();
    }
    else if (!is_connected_ & was_connected_)
    {
      ROS_INFO_STREAM("Mia disconnected");

    }
    was_connected_ = is_connected_;


  }

  //
  // while(ros::ok())
  // {
  //     mia_hw.read (ros::Time::now(), rate.expectedCycleTime());
  //     cm.update   (ros::Time::now(), rate.expectedCycleTime());
  //     mia_hw.write(ros::Time::now(), rate.expectedCycleTime());
  //     rate.sleep();
  // }

  spinner.stop();
  return 0;
}
