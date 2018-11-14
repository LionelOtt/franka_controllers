#include <franka_controllers/joint_position_controller.h>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


namespace fc
{

bool JointPositionController::init(
        hardware_interface::RobotHW *   robot_hardware,
        ros::NodeHandle &               node_handle
)
{
    m_joint_interface =
        robot_hardware->get<hardware_interface::PositionJointInterface>();

    if(m_joint_interface == nullptr)
    {
        ROS_ERROR(
            "JointPositionController: Error getting position joint\
            interface from hardware!"
        );
        return false;
    }

    std::vector<std::string> joint_names;
    if(!node_handle.getParam("joint_names", joint_names))
    {
        ROS_ERROR("JointPositionController: Could not parse joint names");
    }
    if(joint_names.size() != 7)
    {
        ROS_ERROR_STREAM(
            "JointPositionController: Wrong number of joint names, got"
            << joint_names.size() << " instead of 7 names!"
        );
        return false;
    }

    m_joint_handles.resize(joint_names.size());
    for(size_t i=0; i<joint_names.size(); ++i)
    {
        try
        {
            m_joint_handles[i] = m_joint_interface->getHandle(joint_names[i]);
        }
        catch(hardware_interface::HardwareInterfaceException const& e)
        {
            ROS_ERROR_STREAM(
                "JointPositionController: Exception getting joint\
                handles: " << e.what()
            );
            return false;
        }
    }

    m_command_sub = node_handle.subscribe(
            "command",
            1,
            &JointPositionController::command_cb,
            this
    );

    return true;
}

void JointPositionController::update(
        ros::Time const&                time,
        ros::Duration const&            period
)
{
    std::lock_guard<std::mutex> lock(m_command_mutex);

    for(size_t i=0; i<m_last_command.position.size(); ++i)
    {
        m_joint_handles[i].setCommand(m_last_command.position[i]);
    }
}

void JointPositionController::command_cb(sensor_msgs::JointState const& msg)
{
    std::lock_guard<std::mutex> lock(m_command_mutex);

    m_last_command = msg;
}


}  /* namespace franka_controllers */


PLUGINLIB_EXPORT_CLASS(
        fc::JointPositionController,
        controller_interface::ControllerBase
)
