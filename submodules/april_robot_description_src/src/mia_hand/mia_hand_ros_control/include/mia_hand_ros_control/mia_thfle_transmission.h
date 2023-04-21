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

 #ifndef TRANSMISSION_INTERFACE_MIA_THFLE_TRANSMISSION_H
 #define TRANSMISSION_INTERFACE_MIA_THFLE_TRANSMISSION_H

 #include <cassert>
 #include <string>
 #include <vector>
 #include <ros/ros.h>
 #include <transmission_interface/transmission.h>
 #include <transmission_interface/transmission_interface_exception.h>

  #include <yaml-cpp/yaml.h>

 namespace transmission_interface
 {

 /**
  * Implementation of the linear Mia thumb flexion transmission mapping the positions
  * and velocity from the joint sapce to ros actuator-space (i.e. as returned
  * by Mia hand [-255; +255] and [-90;+90]) and vice-versa.
  */
  class MiaThfleTransmission : public Transmission
  {
  public:

/**
  * Class constructor.
  */
   MiaThfleTransmission();

   // TO DELTE
   double testActuatorToJointPosition(double act_data);
   double testJointToActuatorPosition(double jnt_data);


/**
  * Do not use this methos since Mia hand has not effort control.
  */
   void actuatorToJointEffort(const transmission_interface::ActuatorData& act_data,
                                    transmission_interface::JointData&    jnt_data);

/**
  * Transform velocity variables from actuator to joint space.
  * @param act_data thumb_fle actuator actual state.
  * @param jnt_data returned thumb_fle  joint state.
  */
   void actuatorToJointVelocity(const transmission_interface::ActuatorData& act_data,
                                      transmission_interface::JointData&    jnt_data);

/**
  * Transform position variables from actuator to joint space.
  * @param act_data thumb_fle actuator actual state.
  * @param jnt_data returned thumb_fle joint state.
  */
   void actuatorToJointPosition(const transmission_interface::ActuatorData& act_data,
                                      transmission_interface::JointData&    jnt_data);

/**
  * Do not use this methos since Mia hand has not effort control.
  */
   void jointToActuatorEffort(const transmission_interface::JointData&    jnt_data,
                                    transmission_interface::ActuatorData& act_data);

/**
  * Transform velocity variables from joint to actuator space.
  * @param jnt_data mrl joint target.
  * @param act_data returned mrl actuator target.
  */
   void jointToActuatorVelocity(const transmission_interface::JointData&    jnt_data,
                                      transmission_interface::ActuatorData& act_data);

/**
  * Transform position variables from joint to actuator space.
  * @param jnt_data mrl joint target.
  * @param act_data returned mrl actuator target.
  */
   void jointToActuatorPosition(const transmission_interface::JointData&    jnt_data,
                                      transmission_interface::ActuatorData& act_data);

    // Functions describing the mrl trasmissions steps

  /**
    * Thumb flexion transmission first step function for pose: mu =  mu = h_thfle(pos).
    * @param pos pose in the ros actuator-space (i.e. as returned by Mia hand [-255; +255]).
    * @return mu: pose in the mia actuator space.
    */
    double h_thfle(const double pos);         // mu = h_thfle(pos)

  /**
    * Thumb flexion transmission first step inverse function for pose: pos = h_thfle_inv(mu).
    * @param mu pose in the mia actuator space.
    * @return pos: round pose in the ros actuator-space (i.e. as returned by Mia hand [-255; +255]).
    */
    double h_thfle_inv(const double mu);      // pos = h_thfle_inv(mu)

  /**
    * Thumb flexion transmission first step function for velocity: omega_m = dh(spe)
    * @param spe round velocity in the ros actuator-space (i.e. as returned by Mia hand [-90; +90]).
    * @return omega_m velocity in the mia actuator space.
    */
    double dh(const double spe);           // omega_m = dh(spe)


  /**
    * Thumb flexion transmission first step inverse function for velocity: spe = dh_inv(omega_m).
    * @param omega_m velocity in the mia actuator space.
    * @return spe: round velocity in the ros actuator-space (i.e. as returned by Mia hand [-90; +90]).
    */
    double dh_inv(const double omega_m);  // spe = dh_inv(omega_m)

  /**
    * Thumb flexion transmission second step function for pose: sigma = h2_thfle(mu).
    * @param mu: pose in the mia actuator space.
    * @return sigma: calibrated pose in joint space.
    */
    double h2_thfle(const double mu);         // igma = h2_thfle(mu).

  /**
    * Thumb flexion transmission second step inverse function for pose: mu = h2_thfle_inv(sigma).
    * @param sigma: calibrated pose in joint space.
    * @return mu: pose in the mia actuator space.
    */
    double h2_thfle_inv(const double sigma);      // pos = h_thfle_inv(mu)

   std::size_t numActuators() const {return 1;} //!< Number of actuators of the tranmission.
   std::size_t numJoints()    const {return 1;} //!< Number of joints of the tranmission.


   double getActuatorReduction() const {return reduction_;}
   double getJointOffset()       const {return jnt_offset_;}

  private:
   double reduction_;    //!< Linear second step reduction of the tranmission.
   double jnt_offset_;  //!< Offset of teh joint.

   double h_tf_offset_;
   double h_tf_scale_;
   double h_tf_inv_offset_;
   double h_tf_inv_scale_;
   double h2_tf_offset_;
   double h2_tf_scale_;
   double h2_tf_inv_offset_;
   double h2_tf_inv_scale_;

  };


  } // transmission_interface

 #endif // MIA_THFLE_TRANSMISSION_H
