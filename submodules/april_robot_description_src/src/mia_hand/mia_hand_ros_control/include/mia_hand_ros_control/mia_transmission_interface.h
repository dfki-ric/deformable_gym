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

#ifndef TRANSMISSION_INTERFACE_TRANSMISSION_INTERFACE_H
#define TRANSMISSION_INTERFACE_TRANSMISSION_INTERFACE_H

#include <map>
#include <string>
#include <vector>

#include <hardware_interface/internal/resource_manager.h>
#include <transmission_interface/transmission.h>
#include <mia_hand_ros_control/mia_index_transmission.h>
#include <transmission_interface/transmission_interface_exception.h>

namespace transmission_interface
{

/**
  * Base class to handle the MiaIndexTransmission.
  * Handle for propagating a single map (position, velocity) on the MiaIndexTransmission.
  */
  class MiaTransmissionHandle
  {
    public:

    /**
      * Get the name of the transmission.
      * @return The Transmission name
      */
      std::string getName() const {return name_;}

    protected:

      std::string   name_; //!< ransmission name.
      transmission_interface::MiaIndexTransmission* transmission_; //!< MiaIndexTransmission interface.
      ActuatorData  actuator_data_;   //!< Actuator-space variables (target or actual state).
      JointData     joint_data_;     //!< Joint-space variables (target or actual state).
      ActuatorData  actuator_state_; //!< Actuator-space state variables.

    /**
      * Class constructor.
      *¸@param name Transmission name.
      * @param transmission Pointer to the MiaIndexTransmission instance.
      * @param actuator_data Actuator-space variable.
      *¸@param joint_data Joint-space variables.
      * @actuator_state Actuar-space state variable.
      */
      MiaTransmissionHandle(const std::string&  name,
        transmission_interface::MiaIndexTransmission* transmission,
        const ActuatorData& actuator_data,
        const JointData&    joint_data,
        const ActuatorData& actuator_state)
        : name_(name),
        transmission_(transmission),
        actuator_data_(actuator_data),
        joint_data_(joint_data),
        actuator_state_(actuator_state)
        {
          // Precondition: Valid transmission
          if (!transmission_)
          {
            throw TransmissionInterfaceException("Unspecified transmission.");
          }

          // Catch trivial error: All data vectors are empty (handle can't do anything without data)
          if (actuator_data.position.empty() && actuator_data.velocity.empty() && actuator_data.effort.empty() &&
          joint_data.position.empty() && joint_data.velocity.empty() && joint_data.effort.empty() &&
          actuator_state.position.empty() && actuator_state.velocity.empty() && actuator_state.effort.empty())
          {
            throw TransmissionInterfaceException("All data vectors are empty. Transmission instance can't do anything!.");
          }

          // Precondition: All non-empty data vectors must have sizes consistent with the transmission
          if (!actuator_data.position.empty() && actuator_data.position.size() != transmission_->numActuators() )
          {
            throw TransmissionInterfaceException("Actuator position data size does not match transmission.");
          }
          if (!actuator_data.velocity.empty() && actuator_data.velocity.size() != transmission_->numActuators())
          {
            throw TransmissionInterfaceException("Actuator velocity data size does not match transmission.");
          }
          if (!actuator_data.effort.empty() && actuator_data.effort.size() != transmission_->numActuators())
          {
            throw TransmissionInterfaceException("Actuator effort data size does not match transmission.");
          }

          if (!joint_data.position.empty() && joint_data.position.size() != transmission_->numJoints())
          {
            throw TransmissionInterfaceException("Joint position data size does not match transmission.");
          }
          if (!joint_data.velocity.empty() && joint_data.velocity.size() != transmission_->numJoints())
          {
            throw TransmissionInterfaceException("Joint velocity data size does not match transmission.");
          }
          if (!joint_data.effort.empty() && joint_data.effort.size() != transmission_->numJoints())
          {
            throw TransmissionInterfaceException("Joint effort data size does not match transmission.");
          }

          // Added
          if (!actuator_state.position.empty() && actuator_state.position.size() != transmission_->numActuators() )
          {
            throw TransmissionInterfaceException("Actuator position state size does not match transmission.");
          }
          if (!actuator_state.velocity.empty() && actuator_state.velocity.size() != transmission_->numActuators())
          {
            throw TransmissionInterfaceException("Actuator velocity state size does not match transmission.");
          }
          if (!actuator_state.effort.empty() && actuator_state.effort.size() != transmission_->numActuators())
          {
            throw TransmissionInterfaceException("Actuator effort state size does not match transmission.");
          }
          /////////////////////// SONO QUI

          // Precondition: Valid pointers to raw data
          if (!hasValidPointers(actuator_data.position))
          {
            throw TransmissionInterfaceException("Actuator position data contains null pointers.");
          }
          if (!hasValidPointers(actuator_data.velocity))
          {
            throw TransmissionInterfaceException("Actuator velocity data contains null pointers.");
          }
          if (!hasValidPointers(actuator_data.effort))
          {
            throw TransmissionInterfaceException("Actuator effort data contains null pointers.");
          }

          if (!hasValidPointers(joint_data.position))
          {
            throw TransmissionInterfaceException("Joint position data contains null pointers.");
          }
          if (!hasValidPointers(joint_data.velocity))
          {
            throw TransmissionInterfaceException("Joint velocity data contains null pointers.");
          }
          if (!hasValidPointers(joint_data.effort))
          {
            throw TransmissionInterfaceException("Joint effort data contains null pointers.");
          }

          // Added
          if (!hasValidPointers(actuator_state.position))
          {
            throw TransmissionInterfaceException("Actuator position state contains null pointers.");
          }
          if (!hasValidPointers(actuator_state.velocity))
          {
            throw TransmissionInterfaceException("Actuator velocity state contains null pointers.");
          }
          if (!hasValidPointers(actuator_state.effort))
          {
            throw TransmissionInterfaceException("Actuator effort state contains null pointers.");
          }
        }

      private:
        static bool hasValidPointers(const std::vector<double*>& data)
        {
          for (std::vector<double*>::const_iterator it = data.begin(); it != data.end(); ++it)
          {
            if (!(*it)) {return false;}
          }
          return true;
        }
  };

/**
  * Class handling for propagating actuator state (position, velocity and effort) to joint state for a given MiaIndexTransmission.
  * It inherits from the base class MiaTransmissionHandle.
  */
  class MiaActuatorToJointStateHandle : public MiaTransmissionHandle
  {
    public:

    /**
      * Class constructor.
      *¸@param name Transmission name.
      * @param transmission Pointer to the MiaIndexTransmission instance.
      * @param actuator_data Actuator-space variable.
      *¸@param joint_data Joint-space variables.
      */
      MiaActuatorToJointStateHandle(const std::string&  name,
        transmission_interface::MiaIndexTransmission*       transmission,
        const ActuatorData& actuator_data,
        const JointData&    joint_data)
        : MiaTransmissionHandle(name, transmission, actuator_data, joint_data, actuator_data) {}

    /**
      * Propagate actuator state to joint state for the stored MiaIndexTransmission instance.
      */
      void propagate()
      {
          transmission_->actuatorToJointPosition(actuator_data_, joint_data_);
          transmission_->actuatorToJointVelocity(actuator_data_, joint_data_);
          //transmission_->actuatorToJointEffort(  actuator_data_, joint_data_); Unused
      }
  };


  /**
    * Class handling for propagating actuator positions to joint positions for a given MiaIndexTransmission.
    * It inherits from the base class MiaTransmissionHandle.
    */
  class MiaActuatorToJointPositionHandle : public MiaTransmissionHandle
  {
    public:

    /**
      * Class constructor.
      *¸@param name Transmission name.
      * @param transmission Pointer to the MiaIndexTransmission instance.
      * @param actuator_data Actuator-space variable.
      *¸@param joint_data Joint-space variables.
      */
      MiaActuatorToJointPositionHandle(const std::string&  name,
        transmission_interface::MiaIndexTransmission*       transmission,
        const ActuatorData& actuator_data,
        const JointData&    joint_data)
        : MiaTransmissionHandle(name, transmission, actuator_data, joint_data, actuator_data) {}

    /**
      * Propagate actuator positions to joint positions for the stored MiaIndexTransmission instance.
      */
      void propagate() {transmission_->actuatorToJointPosition(actuator_data_, joint_data_);}
  };


/**
  * Class handling for propagating actuator velocity to joint velocity for a given MiaIndexTransmission.
  * It inherits from the base class MiaTransmissionHandle.
  */
  class MiaActuatorToJointVelocityHandle : public MiaTransmissionHandle
  {
    public:

    /**
      * Class constructor.
      *¸@param name Transmission name.
      * @param transmission Pointer to the MiaIndexTransmission instance.
      * @param actuator_data Actuator-space variable.
      *¸@param joint_data Joint-space variables.
      */
      MiaActuatorToJointVelocityHandle(const std::string&  name,
            transmission_interface::MiaIndexTransmission*       transmission,
            const ActuatorData& actuator_data,
            const JointData&    joint_data)
            : MiaTransmissionHandle(name, transmission, actuator_data, joint_data, actuator_data) {}

    /**
      * Propagate actuator velocity to joint velocity for the stored MiaIndexTransmission instance.
      */
      void propagate() {transmission_->actuatorToJointVelocity(actuator_data_, joint_data_);}
  };


/**
  * Class handling for propagating joint state (position, velocity) to actuator state for a given MiaIndexTransmission.
  * It inherits from the base class MiaTransmissionHandle.
  */
  class MiaJointToActuatorStateHandle : public MiaTransmissionHandle
  {
    public:
    /**
      * Class constructor.
      *¸@param name Transmission name.
      * @param transmission Pointer to the MiaIndexTransmission instance.
      * @param actuator_data Actuator-space variable.
      *¸@param joint_data Joint-space variables.
      *¸@param actuator_state Actuator-space variable describing the actual state of the actuator.
      */
      MiaJointToActuatorStateHandle(const std::string&  name,
        transmission_interface::MiaIndexTransmission*       transmission,
        const ActuatorData& actuator_data,
        const JointData&    joint_data,
        const ActuatorData& actuator_state)
        : MiaTransmissionHandle(name, transmission, actuator_data, joint_data, actuator_state) {}

    /**
      * Propagate joint state to actuator state for the stored MiaIndexTransmission instance.
      */
      void propagate()
      {
        transmission_->jointToActuatorPosition(joint_data_, actuator_data_);
        transmission_->IndexjointToActuatorVelocity(joint_data_, actuator_state_, actuator_data_ ); // --> modified to get the actual actuator position state needed for tr
        //transmission_->jointToActuatorEffort(  joint_data_, actuator_data_);
      }
              /*\}*/
  };


  /**
    * Class handling for  propagating joint positions to actuator positions for a given MiaIndexTransmission.
    * It inherits from the base class MiaTransmissionHandle.
    */
  class MiaJointToActuatorPositionHandle : public MiaTransmissionHandle
  {
    public:

    /**
      * Class constructor.
      *¸@param name Transmission name.
      * @param transmission Pointer to the MiaIndexTransmission instance.
      * @param actuator_data Actuator-space variable storing the target of the actuator.
      *¸@param joint_data Joint-space variables storing the target of the joint.
      *¸@param actuator_state Actuator-space variable storing the actual state of the actuator.
      */
      MiaJointToActuatorPositionHandle(const std::string&  name,
        transmission_interface::MiaIndexTransmission*       transmission,
        const ActuatorData& actuator_data,
        const JointData&    joint_data,
        const ActuatorData& actuator_state)
        : MiaTransmissionHandle(name, transmission, actuator_data, joint_data, actuator_state) {}

    /**
      * Propagate joint positions target to actuator positions target for stored MiaIndexTransmission instance.
      */
      void propagate() {transmission_->jointToActuatorPosition(joint_data_, actuator_data_);}
  };



/**
  * Class handling for  propagating joint velocities to actuator velocities for a given MiaIndexTransmission.
  * It inherits from the base class MiaTransmissionHandle.
  */
  class MiaJointToActuatorVelocityHandle : public MiaTransmissionHandle
  {
    public:
    /**
      * Class constructor.
      *¸@param name Transmission name.
      * @param transmission Pointer to the MiaIndexTransmission instance.
      * @param actuator_data Actuator-space variable storing the target of the actuator.
      *¸@param joint_data Joint-space variables storing the target of the joint.
      *¸@param actuator_state Actuator-space variable storing the actual state of the actuator.
      */
      MiaJointToActuatorVelocityHandle(const std::string&  name,
        transmission_interface::MiaIndexTransmission*       transmission,
        const ActuatorData& actuator_data,
        const JointData&    joint_data,
        const ActuatorData& actuator_state)
        : MiaTransmissionHandle(name, transmission, actuator_data, joint_data, actuator_state) {}

    /**
      * Propagate joint velocity target to the actuator velocity target for stored MiaIndexTransmission instance.
      */
      void propagate() {transmission_->IndexjointToActuatorVelocity(joint_data_, actuator_state_, actuator_data_ );} // --> modified to get the actual actuator position state needed for tr
  };

  template <class HandleType>
  class MiaTransmissionInterface : public hardware_interface::ResourceManager<HandleType>
  {
  public:

    HandleType getHandle(const std::string& name)
    {
      // Rethrow exception with a meaningful type
      try
      {
        return this->hardware_interface::ResourceManager<HandleType>::getHandle(name);
      }
      catch(const std::logic_error& e)
      {
        throw TransmissionInterfaceException(e.what());
      }
    }

    void propagate()
    {
      typedef typename hardware_interface::ResourceManager<HandleType>::ResourceMap::iterator ItratorType;
      for (ItratorType it = this->resource_map_.begin(); it != this->resource_map_.end(); ++it)
      {
        it->second.propagate();
      }
    }
    /*\}*/
  };

  // Convenience typedefs

  class MiaActuatorToJointStateInterface : public MiaTransmissionInterface<MiaActuatorToJointStateHandle> {};

  class MiaActuatorToJointPositionInterface : public MiaTransmissionInterface<MiaActuatorToJointPositionHandle> {};

  class MiaActuatorToJointVelocityInterface : public MiaTransmissionInterface<MiaActuatorToJointVelocityHandle> {};


  class MiaJointToActuatorStateInterface : public MiaTransmissionInterface<MiaJointToActuatorStateHandle> {};

  class MiaJointToActuatorPositionInterface : public MiaTransmissionInterface<MiaJointToActuatorPositionHandle> {};

  class MiaJointToActuatorVelocityInterface : public MiaTransmissionInterface<MiaJointToActuatorVelocityHandle> {};


} // transmission_interface

#endif //TRANSMISSION_INTERFACE_TRANSMISSION_INTERFACE_H
