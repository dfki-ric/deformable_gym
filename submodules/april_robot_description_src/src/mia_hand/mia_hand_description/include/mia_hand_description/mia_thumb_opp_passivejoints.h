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

#ifndef MIA_THUMB_OPP_PASSIVEJOINTS_H
#define MIA_THUMB_OPP_PASSIVEJOINTS_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <string>
#include <cmath>
#include <urdf/model.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
 #include <yaml-cpp/yaml.h>
/**
 * namespace containing thumb_opp_passive_joints class.
 */
namespace mia_hand
{

/**
 * thumb_opp_passive_joints class.
 * This class can be used to evaluate the state of the thumb opposition (passive
 * joint) of the Mia Hand. using the state of the index_flex joint.
 */
class thumb_opp_passive_joint
{
public:

    /**
     * Class constructor.
     */
    thumb_opp_passive_joint();  // Constructor, parameters with default values.

    /**
     * Class destructor.
     */
    ~thumb_opp_passive_joint();


    /**
     * Initialize class.
     * @param LoadURDFInfo a bool argument. True if the class has to load joint limits info from the URDF.
     */
    void init(const bool LoadURDFInfo_ = false);

    /**
     * Evaluate the position of the thumb opposition based on index flexion state.
     * A method taking the position of the index_fle joint and returning the position of the thumb_opp joint.
     * @param j_index_flex_pos a double argument describing the position of the index_fle joint.
     * @ŗeturn double describing the position of the thumb_opp joint.
     */
     double GetThumbOppPosition(double j_index_flex_pos);

    /**
     * Load Info from loaded URDF.
     * A method that uses the class attributes to extract the upper and lower limit of the thumb joint from the URDF.
     * @see j_index_name()
     * @see j_thumb_name()
     * @see robot_description_()
     * @see ThMinPos()
     * @see ThMaxPos()
     * @return updates var ThMinPos and ThMaxPos.
     */
     bool LoadURDFInfo();

     /**
      * Update limits value of Th_opp joints
      * Update the value of variables th_min_pos_ and th_max_pos
      * param min_position: lower limit of the position of the joint
      * param max_position: upper limit of the position of the joint
      */
      void updateThOppJointLimits(double min_position, double max_position);


    /**
     * Name of the index_fle joint specified in the URDF.
     *
     */
    std::string j_index_name ;

    /**
     * Name of the thumb_opp joint specified in the URDF.
     *
     */
    std::string j_thumb_name ;

    /**
     * Name of parameter describing the robot description loaded in the ROS parameter server.
     *
     */
    std::string robot_description_ ;



private:
  /**
   * Lower Limit of the thumb_opp joint.
   *
   */
  double th_min_pos_;

  /**
   * Upper Limit of the thumb_opp joint.
   *
   */
  double th_max_pos_;

  /**
    * Index angles that pilot the thumb opposition movement
    *
    */
    double th_opp_start_close_index_angle_;
    double th_opp_stop_close_index_angle_;

  /**
    * Scale that pilot the thumb opposition movement
    *
    */
    double th_opp_scale_;
  /**
    * Scale and offset that pilot the thumb opposition movement
    *
    */
    double th_opp_offset_;

 /**
   * A ROS node variable.
   *
   */
    ros::NodeHandle n;

  /**
		* Get the name of the URDF.
		* A method taking the ros parameter urdf generic name and return the specific
		* of the loaded URDF.
		* @param param_name generic name of the param to look for.
		* @return the specific name of the parameter linking to the robot URDF
		*/
    std::string getURDF(std::string param_name) const;


};
}  // namespace

#endif  // MIA_THUMB_OPP_PASSIVEJOINTS_H
