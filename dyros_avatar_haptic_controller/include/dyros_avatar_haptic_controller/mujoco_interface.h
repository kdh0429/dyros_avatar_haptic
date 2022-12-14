#include <ros/ros.h>
#include "dyros_avatar_haptic_controller/data_container.h"
#include "mujoco_ros_msgs/SimStatus.h"
#include "mujoco_ros_msgs/JointSet.h"
#include <mutex>

#ifndef MujocoInterface_H
#define MujocoInterface_H
class MujocoInterface
{
    public:
        MujocoInterface(ros::NodeHandle &nh, DataContainer &dc, std::mutex &m_dc);
        ~MujocoInterface();
        void stateUpdate();
        void simStatusCallback(const mujoco_ros_msgs::SimStatusConstPtr &msg);
        void sendCommand(int control_mode);

    private:
        DataContainer &dc_;
        std::mutex &m_dc_;
        
        ros::Subscriber mujoco_sim_status_sub_;
        ros::Publisher mujoco_joint_set_pub_;

        bool is_first_callback = true;

        mujoco_ros_msgs::JointSet mujoco_joint_set_msg_;
};
#endif