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

#ifndef MIA_HW_INTERFACE_H
#define MIA_HW_INTERFACE_H



/*TODO: install all Mia Hand classes under the same folder, such that, to
 * include them, the macro instruction #include "mia/... will be necessary.
 "*/

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

// messages and ServiceServer
#include "mia_hand_msgs/FingersData.h"
#include "mia_hand_msgs/FingersStrainGauges.h"
#include "mia_hand_msgs/GraspRef.h"
#include "mia_hand_msgs/ConnectSerial.h"
#include "mia_hand_msgs/ComponentStatus.h"

#include "std_msgs/Int16.h"
#include "std_msgs/String.h"

#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"

#include <vector>

// URDF
#include <urdf/model.h>

#include <sstream>

#include <transmission_interface/transmission_parser.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_interface.h>
#include "mia_hand_ros_control/mia_transmission_interface.h"
#include "mia_hand_ros_control/mia_thfle_transmission.h"
#include "mia_hand_ros_control/mia_mrl_transmission.h"
#include "mia_hand_ros_control/mia_index_transmission.h"

#include "mia_hand_driver/cpp_driver.h"

// Passive Joints of the thumb opp and index flexion
#include "mia_hand_description/mia_thumb_opp_passivejoints.h"

namespace mia_hand
{

/**
	* Struct to store the indexes of each joints in the MiaHWInterface Class
	* Index of each joint depends on the order of trasmission defined in the urdf file
	*/
	struct Joint_index_mapping
	{
		int j_thumb_opp;
		int j_thumb_flex;
		int j_index_flex;
		int j_mrl_flex;
		int j_mrl_2;
		int j_mrl_3;
	};

/**
 * Class hardware interface of the real Mia hand.
 * This class inherits from public hardware_interface::RobotHW.
 */
	class MiaHWInterface: public hardware_interface::RobotHW
	{
	public:

	/**
		* Class constructor.
		*/
		MiaHWInterface();

	/**
		* Class distructor.
		*/
		 ~MiaHWInterface();



	/**
		* Initialize the class hardware interface and the robot hardware.
		* Initialize the hardware driver, connect the COM port in which the Mia hand
		* is plugged and initialize the  hardware interface class, registering
		* the needed interfaces and the joint limits.
		* @param root_nh root node handle.
		* @param mia_hw_nh Mia hardware node handle.
		* @return True if the robot hardware is initialized successfully, False if not.
		*/
		bool init(ros::NodeHandle& root_nh, ros::NodeHandle& mia_hw_nh) override;

	/**
		* Initialize the class hardware interface and the robot hardware.
		*/
		void update(const ros::TimerEvent& e);

	/**
		* Read state data from the real robot hardware.
		* Read joint positions velocities from the real robot hardware.
		* @param time The current time.
		* @param duration The time passed since the last call to read().
		*/
		void read(const ros::Time& time, const ros::Duration& duration) override;

	/**
		* Write data command to the real robot hardware.
		* Write position or velocities command data to the robot hardware.
		* @param time The current time.
		* @param duration The time passed since the last call to read().
		*/
		void write(const ros::Time& time, const ros::Duration& duration) override;

		bool checkConnection(); //!< Check the status of the hand connection

	protected:


		/**
    * Joints control methods.
    * All possible joints controlled methods allowed by ROS. NB. EFFORT control
		* method is not allowed with Mia hand. It is included since in the URDF
		* file efforts limits are usually specified.
    */
		enum ControlMethod {EFFORT, POSITION, VELOCITY}; 		//enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

	/**
    * Register the joint limits of the real mia hardware.
    * Register the limits of the joint specified by joint_name and joint_handle. The limits are
    * retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
    * @param joint_name Name of the joint whose limits has to be registered.
    * @param joint_handle Handler of the joint.
    * @param ctrl_method Array of enum describe the control methods available for that joint.
    * @param joint_limit_nh Joint node handle.
    * @param urdf_model URDF model.
    * @param joint_type Joint type (e.g. urdf::Joint::REVOLUTE).
    * @param lower_limit Lower position limit of the joint returned by this method.
    * @param upper_limit Upper position limit of the joint returned by this method.
    * @param effort_limit Effortlimit of the joint returned by this method (unused).
    */
		void registerJointLimits(const std::string& joint_name,
								 const std::vector<hardware_interface::JointHandle>& joint_handle,
								 const std::vector<ControlMethod> ctrl_method,
								 const ros::NodeHandle& joint_limit_nh,
								 const urdf::Model *const urdf_model,
								 int *const joint_type, double *const lower_limit,
								 double *const upper_limit, double *const effort_limit);


	/** Initialization of the arrays saving the last command received.
	  * Initialize the values of array variables ( #last_joint_position_command_ and
		* #last_joint_velocity_command_ ) containing the last received command.
		* They will be used to know which command (velocity or position) to send to the hand.
		* @see #SelectCtrMethod.
		*/
		void InitBkLastCommands();

	/**
		* Select the control method to move the real mia hardware.
		* Since velocity and position interfaces are both loaded for each joint,
		* this method recognize wich type of control the received new command requires.
		* @param last_joint_control_methods_ Last type of control method used for the joint.
		* @param last_joint_velocity_command_ Last velocity command received for the joint.
		* @param joint_velocity_command_ Actual velocity command received for the joint.
		* @param last_joint_position_command_ Last position command received for the joint.
		* @param joint_position_command_ Actual position command received for the joint.
		* @return the control method to use in the current time step to move the joint.
		*/
		enum ControlMethod SelectCtrMethod(enum ControlMethod last_joint_control_methods_,
	    							    const double last_joint_velocity_command_,
	    							    const double joint_velocity_command_,
	    							    const double last_joint_position_command_,
	    							    const double joint_position_command_);


	/**
		* Evaluate the position of the Mia thumb opposition joint.
		* Since in the actual Mia hand the opposition of the thumb (passive joint)
		* is coupled to the movement of the index flexion this function uses
		* the sate of the index flexion joint of the mia hand hardware
		* to evaluate the position that the actual thumb opposition has reached at
		* the current time.
		* @param period Time since the last simulation step.
		* @return The target position of the Mia simulated thumb opposition joint.
		*/
		double GetThumbOppPosition();

	/**
		* Get the name of the URDF.
		* A method taking the ros parameter urdf generic name and return the specific
		* of the loaded URDF.
		* @param param_name generic name of the param to look for.
		* @return the specific name of the parameter linking to the robot URDF
		*/
		std::string getURDF(std::string param_name) const;


	/**
		*Get the transmission names and type from the loaded URDF
		* Get the trasnsmission names specified in the mia hand URDF to retrieve
		* joints info as name, limits and type etc.
		* @param urdf_string URDF parameter name.
	 	*/
		bool parseTransmissionsFromURDF(const std::string& urdf_string);

	/**
		* Connect Mia hand to the specified seria port .
		* @param port_num number of the serial port.
		* @return True if the connection suceeded , False if not.
		*/
		bool connect_mia(uint16_t port_num);

		bool disconnect_mia();//!< Disconnet mia and its stremaings

		CppDriver mia_;  //!< Low level driver of the Mia hand.
		int COM_number_; //!< Number of the COM in which the Mia hand is plugged.


		bool is_connected_;  //!< True if the Mia hand is connected, False otherwise.
	  bool was_connected_; //!< True if the Mia hand was connected, False otherwise.
		bool is_initialized; //!< True if tthe class has already been initialized.
		std::mutex connectionstatus_mtx_;
		std::mutex connectionmia_mtx_;

		ros::NodeHandle nh_; //!< ROS node handle.
		// ros::Timer non_realtime_loop_;
		// ros::Duration control_period_;
		// ros::Duration elapsed_time_;

		void initPublishers();     //!< Initialize publishers.

		/* Publishers
		 */
		ros::Publisher mot_cur_info_; //!< Publisher of the current absorbed by the Mia hand motors.
	  ros::Publisher fin_for_info_; //!< Publisher of the force data read by the sensors embedded in the Mia hand fingers.
		ros::Publisher connection_status_info_; //!< Publisher of the status of the connection of  the device.

		bool is_mot_cur_info_enabled;
		bool is_fin_for_info_enabled;

		/* Services
		 */

		ros::ServiceServer connect_to_port_; //!< Service to open a serial port and connect the Mia hand.
		ros::ServiceServer disconnect_;     //!< Service to disconnect the Mia hand and close the serial port.

		ros::ServiceServer switch_ana_stream_; //!< Service to manage a the streaming of the analog input data (i.e. force sensor outputs) sent by the Mia hand.
		ros::ServiceServer switch_cur_stream_; //!< Service to manage a the streaming of the current data (i.e. force sensor outputs) sent by the Mia hand.

		void initServices(); //!< Initialize services.
		/**
		 * Callback of the service #switch_ana_stream_.
		 * Manage the streaming of the analog input data (i.e. force sensor outputs) sent by the Mia hand.
		 * @param req True to enable the data streaming, False to disable it.
		 * @param resp Unused
		 */
		 bool switchAnaStreamCallback(std_srvs::SetBool::Request& req,
																std_srvs::SetBool::Response& resp);

		/**
		 * Callback of the service #switch_cur_stream_.
		 * Manage the streaming of the analog input data (i.e. force sensor outputs) sent by the Mia hand.
		 * @param req True to enable the data streaming, False to disable it.
		 * @param resp Unused
		 */
		 bool switchCurStreamCallback(std_srvs::SetBool::Request& req,
																std_srvs::SetBool::Response& resp);

		/**
     * Callback of the service #connect_to_port_.
     * Connect the Mia hand opening a serial port.
     * @param req mia_hand_msgs containing the number of the port to open.
     * @param resp mia_hand_msgs containing the success of the operation (true or false)
     * and a string message.
     */
	   bool connectToPortCallback(mia_hand_msgs::ConnectSerial::Request& req,
	                              mia_hand_msgs::ConnectSerial::Response& resp);

    /**
     * Callback of the service #disconnect_.
     * Disconnect the Mia hand and close the serial port.
     * @param req unused.
     * @param resp containing the success of the operation (true or false)
     * and a string message.
     */
	   bool disconnectCallback(std_srvs::Trigger::Request& req,
	                           std_srvs::Trigger::Response& resp);



	/**
	 	* Generic name of the URDF to look for with #getURDF(std::string param_name).
		*/
		std::string robot_description_;

	/**
		* Transmissions declared in the URDF.
		*/
	  std::vector<transmission_interface::TransmissionInfo> URDFtransmissions_;

		////////////////////////////////////////////////////////////////////////////
		// ROS Interfaces

		hardware_interface::JointStateInterface    js_interface_; //!< Interface for the real Mia joint state.
		hardware_interface::PositionJointInterface pj_interface_; //!< Interface for the real Mia joint position commands.
		hardware_interface::VelocityJointInterface vj_interface_; //!< Interface for the real Mia  joint velocity commands.
		hardware_interface::EffortJointInterface   ej_interface_; //!<  unused

		joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;     //!< Interface for the real Mia joint position saturation limit.
		joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;  //!< Interface for the real joint position soft limit.
		joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;    //!< Interface for the real Mia joint velocity saturation limit.
		joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_; //!< Interface for the real Mia joint velocity software limit.

		joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;     //!<  unused
		joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_; //!<  unused

		////////////////////////////////////////////////////////////////////////////
		//Software transmissions.

	/**
		* Transmissions class implemented for the Mia hand thumb flexion joint.
		* @see MiaThfleTransmission class
		*/
		transmission_interface::MiaThfleTransmission ThfleTrans;

	/**
		* Transmissions class implemented for the Mia hand mrl flexion joint.
		* @see MiaMrlTransmission class
		*/
	  transmission_interface::MiaMrlTransmission MrlTrans;

	/**
		* Transmissions class implemented for the Mia hand mrl flexion joint.
		* @see MiaIndexTransmission class
		*/
		transmission_interface::MiaIndexTransmission IndexTrans;

	/**
		* Transmissions vector to handle the transmissions implemented for the real Mia hand.
		*/
		std::vector<transmission_interface::Transmission*> MiaTrasmissions;

		std::vector<std::string> trasmission_names_;//!<  Actual transmissions name


	/**
		* Transmission interface for propagating current actuator position state to joint space.
		* Transmission interface used for Mia mrl and thumb flexion joints/actuators.
		*/
		transmission_interface::ActuatorToJointPositionInterface       act_to_jnt_pos_state;

	/**
		* Transmission interface for propagating current velocity actuator state to joint space.
		* Transmission interface used for Mia mrl and thumb flexion joints/actuators.
		*/
		transmission_interface::ActuatorToJointVelocityInterface       act_to_jnt_vel_state;

	/**
		* Transmission interface for propagating joint position commands to actuator space.
		* Transmission interface used for Mia mrl and thumb flexion joints/actuators.
		*/
		transmission_interface::JointToActuatorPositionInterface 			 jnt_to_act_pos;

	/**
		* Transmission interface for propagating joint velocity commands to actuator space
		* Transmission interface used for Mia mrl and thumb flexion joints/actuators.
		*/
		transmission_interface::JointToActuatorVelocityInterface 			 jnt_to_act_vel;

	/**
		* Transmission interface for propagating current actuator position state of the Mia index to joint space.
		* Transmission interface used for Mia index flexion joints/actuators.
		*/
		transmission_interface::MiaActuatorToJointPositionInterface    index_act_to_jnt_pos_state;

	/**
		* Transmission interface for propagating current actuator velocity state of the Mia index to joint space.
		* Transmission interface used for Mia index flexion joints/actuators.
		*/
		transmission_interface::MiaActuatorToJointVelocityInterface    index_act_to_jnt_vel_state;

	/**
		* Transmission interface for propagating joint position commands to actuator space.
		* Transmission interface used for Mia index flexion joints/actuators.
		*/
		transmission_interface::MiaJointToActuatorPositionInterface    index_jnt_to_act_pos;

	/**
		* Transmission interface for propagating joint velocity commands to actuator space.
		* Transmission interface used for Mia index flexion joints/actuators.
		*/
		transmission_interface::MiaJointToActuatorVelocityInterface    index_jnt_to_act_vel;


		// Actuator and joint space variables: wrappers around raw data

		transmission_interface::ActuatorData a_state_data[3]; //!< Actuator state data (one for transmission).
  	transmission_interface::ActuatorData a_cmd_data[3];  //!< Actuator command data (one for transmission).

  	transmission_interface::JointData j_state_data[3]; //!< Joint state data (one for transmission).
  	transmission_interface::JointData j_cmd_data[3];  //!< joint command data (one for transmission).

	/**
		* Class used to evaluate the target position of the Mia simulated thumb opposition joint.
		* Class describing the relation between the position of the Mia index flexion
		* joint and the dependent one of the thumb opposition. This class is implemented in the
		* mia_hand_description pkg.
		* @see GetThumbOppPosition()
		*/
		mia_hand::thumb_opp_passive_joint MyTh_opp_passiveJoint; // Passivejoint thumb opp and index_fle

		///////////////////////////////////////////////////////////////////////////
		// Others

	/**
		* Joint_index_mapping struct.
		* Struct that stores the index number of the simulated mia hand joint as contained
		* in the interfaces and arrays of this class
		* @see Joint_index_mapping()
		*/
   	Joint_index_mapping joints_ii;

		unsigned int write_counter; //!< Counter of the write method.
	  int n_dof_sim_; //!< Number of degree of freedom.
		int n_actuators_;


		std::vector<std::string> joint_names_;  //!< Name of the joints as specified in the URDF.
		std::vector<int> joint_types_;					 //!< Type of the joints specified in the URDF.

		std::vector<double> joint_position_state_;  //!< Actual position state of the joints of the Mia hardware.
		std::vector<double> joint_velocity_state_;  //!< Actual velocity state of the joints of the Mia hardware.
		std::vector<double> joint_effort_state_;    //!< Useless.

		std::vector<double> joint_position_command_; //!< Actual position joint command for the Mia hardware.
		std::vector<double> joint_velocity_command_; //!< Actual velocity joint command for the Mia hardware.
		std::vector<double> joint_effort_command_;   //!< Useless.

		std::vector<double> last_joint_position_command_;  //!< Last joint position command received for the Mia hardware.
		std::vector<double> last_joint_velocity_command_;  //!< Last joint velocity command received for the Mia hardware.

		std::vector<double> act_position_state_;  //!< Actual position state of the actuators of the Mia hardware.
		std::vector<double> act_velocity_state_; //!< Actual velocity state of the actuators of the Mia hardware.
		std::vector<double> act_effort_state_;   //!< Useless.

		std::vector<double> act_position_command_; //!< Actual position actuator command for the Mia hardware.
		std::vector<double> act_velocity_command_; //!< Actual velocity actuator command for the Mia hardware.
		std::vector<double> act_effort_command_;   //!< Useless.


		std::vector<ControlMethod> joint_control_methods_; //!< Actual control method for the joints of the Mia hardware.
		std::vector<ControlMethod> last_joint_control_methods_; //!< Last control method used for the joints of the Mia hardware.
		std::vector<std::vector<ControlMethod>> List_joint_control_methods_;  //!< List of control method available for the joints of the Mia hardware.

		std::vector<double> joint_lower_limits_; //!< Lower position limits of the joints of the Mia hardware (as specified in the URDF)
		std::vector<double> joint_upper_limits_; //!< Upper position limits of the joints of the Mia hardware (as specified in the URDF).

		std::vector<double> joint_effort_limits_; //!< Effort limit of the joints of the Mia hardware specified in the URDF (fake).

		// TO DELETE Needed to fix the problem that velocity returned by MIA is abs
		std::vector<double> old_act_position_state_; //!< Last position state of the actuator returned by the Mia hardware in the #read() funtion.
		int read_counter; //!< Counter for the #read() funtion.



	};
}  // namespace

#endif
