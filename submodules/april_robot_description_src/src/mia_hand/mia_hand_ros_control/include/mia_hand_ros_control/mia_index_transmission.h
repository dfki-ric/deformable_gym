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

#ifndef MIA_INDEX_TRANSMISSION_H
#define MIA_INDEX_TRANSMISSION_H

#include <iostream>
#include <vector>
#include <cassert>
#include <string>
#include <cmath>
#include <ros/ros.h>

#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_interface_exception.h>

#include <yaml-cpp/yaml.h>


namespace transmission_interface
{

/**
  * Implementation of the no-linear Mia Index Transmission.
  */
  class MiaIndexTransmission: public Transmission
  {

    public:

    /**
      * Class constructor.
      */
      MiaIndexTransmission ();

      // TO DELETE
      double testActuatorToJointPosition(
        double ind_act_state
        );

      double testJointToActuatorPosition(
          double ind_joint_state
          );

    /**
      * Do not use this methos since Mia hand has not effort control.
      */
      void actuatorToJointEffort(
        const transmission_interface::ActuatorData& ind_act_state,
              transmission_interface::JointData&    ind_jnt_data) override;

    /**
      * Transform position variables from actuator to joint space.
      * @param ind_act_state index actuator actual state.
      * @param ind_jnt_data returned index joint state.
      */
      void actuatorToJointPosition(
        const transmission_interface::ActuatorData& ind_act_state,
              transmission_interface::JointData&    ind_jnt_data) override;

    /**
      * Transform velocity variables from actuator to joint space.
      * @param ind_act_state index actuator actual state.
      * @param ind_jnt_data returned index joint joint.
      */
      void actuatorToJointVelocity(
        const transmission_interface::ActuatorData& ind_act_state,
              transmission_interface::JointData&    ind_jnt_data) override;

    /**
      * Do not use this methos since Mia hand has not effort control.
      */
      void jointToActuatorEffort(
        const transmission_interface::JointData&    ind_jnt_data,
            transmission_interface::ActuatorData& ind_act_cmd) override;

    /**
      * Transform position variables from joint to actuator space.
      * @param ind_jnt_data index joint target.
      * @param ind_act_cmd returned index actuator target.
      */
      void jointToActuatorPosition(
        const transmission_interface::JointData&    ind_jnt_data,
              transmission_interface::ActuatorData& ind_act_cmd) override;

    /**
      * unused
      */
      void jointToActuatorVelocity(
        const transmission_interface::JointData&    ind_jnt_data,
              transmission_interface::ActuatorData& act_data) override; // unused

    /**
      * Transform velocity variables from joint to actuator space.
      * @param ind_jnt_data index joint target.
      * @param ind_act_state index actuator
      * @param ind_act_cmd returned actuator joint target.
      */
      void IndexjointToActuatorVelocity(
        const transmission_interface::JointData&    ind_jnt_data,
  	  const transmission_interface::ActuatorData& ind_act_state,
              transmission_interface::ActuatorData& ind_act_cmd) ;

  /**
    * Index transmission first step function for pose: mu = h_i(pos).
    * @param pos pose in the ros actuator-space (i.e. as returned by Mia hand [-255; +255]).
    * @return mu: pose in the mia actuator space.
    */
  	double h_i(const double pos);            // mu = h_i(pos)

  /**
    * Index transmission first step inverse function for pose: pos = h_i_inv(mu)
    * @param mu pose in the mia actuator space
    * @return pos: pose in the ros actuator-space (i.e. as returned by Mia hand [-255; +255]).
    */
  	double h_i_inv(const double mu);         // pos = h_i_inv(mu)

    /**
      * Index transmission second step function for pose: alpha = h2_i(mu).
      * @param mu motor position.
      * @return alpha: non linear transmission input angle .
      */
    	double h2_i(const double mu);            // alpha = h2_i(mu)

    /**
      * Index transmission first step inverse function for pose: pos = h_i_inv(mu)
      * @param alpha: non linear transmission input angle.
      * @return mu motor position.
      */
    	double h2_i_inv(const double alpha);         // mu = h2_i_inv(alpha)

  /**
    * Index transmission first step function for velocity: omega_m = dh_i(spe).
    * @param spe velocity in the ros actuator-space (i.e. as returned by Mia hand [-90; +90]).
    * @return omega_m: velocity in the mia actuator space.
    */
  	double dh_i(const double spe);           // omega_m = dh_i(spe)

  /**
    * Index transmission first step inverse function for velocity: spe = dh_i_inv(omega_m)
    * @param spe velocity in the ros actuator-space (i.e. as returned by Mia hand [-90; +90]).
    * @return omega_m: velocity in the mia actuator space.
    */
  	double dh_i_inv(const double omega_m);  // spe = dh_i_inv(omega_m)

  /**
    * Index transmission second step function for position: delta = f(alpha)
    * @param alpha position of the intermediate step of the transmission.
    * @return delta: position in the mia joint space.
    */
  	double f(const double alpha );         // delta = f(alpha)

  /**
    * Index transmission second step inverse function for position: alpha = f_inv(delta)
    * @param delta position in the mia joint space.
    * @return delta: alpha: position of the intermediate step of the transmission.
    */
  	double f_inv(const double delta );     // alpha = f_inv(delta)

    /**
      * Index transmission final step function for position: delta_calib = f(delta)
      * @param delta: position in the mia joint space.
      * @return delta_calib: calibrated position in the mia joint space.
      */
    	double f2(const double delta );         // delta_calib = f2(delta)

    /**
      * Index transmission second step inverse function for position: alpha = f_inv(delta)
      * @param delta_calib: calibrated position in the mia joint space.
      * @return delta: position in the mia joint space.
      */
    	double f2_inv( const double delta_calib );     // delta = f2_inv(delta_calib)

  /**
    * Index transmission second step function for velocity: alpha = f_inv(delta)
    * @param omega_a velocity f the intermediate step of the transmission.
    * @return omega_d: velocity in the mia joint space.
    */
  	double df(const double omega_a );     // // Non linear TR-1

      // General trasmissions attributes
      std::size_t numActuators() const {return 1;} //!< Number of actuators of the tranmission.
      std::size_t numJoints() const {return 1;} //!< Number of joints of the tranmission.

    private:

    /**
      * Vars of the linear part of index transmission.
      */
      double linear_reduction_;
      double h_i_offset_;
      double h_i_scale_;
      double h_i_inv_offset_;
      double h_i_inv_scale_;
      double h2_i_offset_;
      double h2_i_scale_;
      double h2_i_inv_offset_;
      double h2_i_inv_scale_;
      double f2_offset_;


    /**
      * Upper limiits of the inputs intervals of the f funtion.
      * @see f().
      */
      std::vector<double> x_intervals_f_;

    /**
      * Upper limiits of the inputs intervals of the f_inv funtion.
      * @see f_inv().
      */
      std::vector<double> x_intervals_f_inv_;

    /**
      * Upper limiits of the inputs intervals of the df funtion.
      * @see df().
      */
      std::vector<double> x_intervals_df_;

    /**
      * Scales factors of the f funtion.
      * @see f().
      */
      std::vector<double> f_scale_;

    /**
      * Offset factors of the f funtion.
      * @see f().
      */
      std::vector<double> f_offset_;

    /**
      * Scale factors of the f_inv funtion.
      * @see f_inv().
      */
      std::vector<double> f_inv_scale_;

    /**
      * Offset factors of the f_inv funtion.
      * @see f_inv().
      */
      std::vector<double> f_inv_offset_;

    /**
      * Scale factors of the df funtion.
      * @see df().
      */
      std::vector<double> df_scale_;

    /**
      * Offset factors of the df funtion.
      * @see df().
      */
      std::vector<double> df_offset_;


  };



}  // namespace

#endif  //MIA_INDEX_TRANSMISSION_H
