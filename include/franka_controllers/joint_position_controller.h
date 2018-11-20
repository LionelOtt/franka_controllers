#ifndef __JOINT_POSITION_CONTROLLER_H__
#define __JOINT_POSITION_CONTROLLER_H__


#include <array>
#include <mutex>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensors_msgs/JointState.h>
#include <std_msgs/Float64.h>


namespace fc
{


class JointPositionController :
    public controller_interface::MultiInterfaceController<
            hardware_interface::PositionJointInterface
    >
{
    public:
        /**
         * \brief Initializes the controller.
         *
         * \param robot_hardware handle to the robot's hwardware
         *      abstraction
         * \param node_handle node handle instance
         */
        bool init(
                hardware_interface::RobotHW * robot_hardware,
                ros::NodeHandle &       node_handle
        ) override;

        /**
         * \brief Initialization of the controller upon activation.
         *
         * \param time time at which the controller was activated
         */
        void starting(ros::Time const& time) override;

        /**
         * \brief Control update loop execution.
         *
         * \param time current time
         * \param period time elapsed since last call
         */
        void update(ros::Time const& time, ros::Duration const& period) override;


    private:
        /**
         * \param Callback receiving control signals.
         *
         * \param msg control command message
         */
        void command_cb(std_msgs::Float64 const& msg);


    private:
        //! Mutex to ensure thread safe command read / write access
        std::mutex                      m_command_mutex;
        //! Last command received by the controller
        sensor_msgs::JointState         m_last_command;
        //! Current value of the joint positions
        std::array<double, 7>           m_current_joint_position;
        //! Hwardware interface handle
        hardware_interface::PositionJointInterface * m_joint_interface;
        //! Handles to the individual joints
        std::vector<hardware_interface::JointHandle> m_joint_handles;
        //! Subscriber listening to control command updates
        ros::Subscriber                 m_command_sub;
};


} /* namespace fc */


#endif /* __JOINT_POSITION_CONTROLLER_H__ */
