
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <thread>

#include "dyros_avatar_haptic_controller/dyros_avatar_haptic_controller.h"
#include "dyros_avatar_haptic_controller/mujoco_interface.h"

#define TorqueControl 1
#define PositionControl 0

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyros_avatar_haptic_controller");
    ros::NodeHandle nh;
    DataContainer dc;

    int control_mode = TorqueControl;

    std::mutex m_dc;
    MujocoInterface mujoco_interface(nh, dc, std::ref(m_dc));
    DyrosAvatarHapticController dyros_avatar_haptic_controller(nh, dc, control_mode, std::ref(m_dc));

    std::thread thread[3];
    thread[0] = std::thread(&MujocoInterface::stateUpdate, &mujoco_interface);
    thread[1] = std::thread(&DyrosAvatarHapticController::compute, &dyros_avatar_haptic_controller);
    thread[2] = std::thread(&MujocoInterface::sendCommand, &mujoco_interface, control_mode);

    for (int i = 0; i < 3; i++)
    {
        thread[i].join();
    }

    return 0;
}
