#include <mutex>
#include <random>
#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include "dyros_avatar_haptic_controller/util.h"

#include "mujoco_ros_msgs/JointSet.h"

#include "dyros_avatar_haptic_controller/mujoco_interface.h"

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include <fstream>

#include <std_msgs/Float32MultiArray.h>

# define MODE_INIT 105
# define MODE_HOME 104
# define MODE_RANDOM 114
# define MODE_FORCE 102
# define MODE_STOP 115

class DyrosAvatarHapticController{
    public:
        DyrosAvatarHapticController(ros::NodeHandle &nh, DataContainer &dc, int control_mode);
        ~DyrosAvatarHapticController();
        void compute();
        void updateKinematicsDynamics();
        void computeControlInput();
        void printData();

    private:
        double hz_ = 2000;
        double cur_time_;
        double pre_time_;
        double init_time_;

        int mode_ = 0;
        double mode_init_time_ =  0.0;
        Eigen::VectorXd q_mode_init_;
        Eigen::VectorXd q_dot_mode_init_;
        Eigen::Isometry3d x_mode_init_;

        std::mutex m_dc_;
        std::mutex m_ci_;
        std::mutex m_ext_;
        std::mutex m_buffer_;
        std::mutex m_rbdl_;

        DataContainer &dc_;

        bool is_init_ = false;

        double sim_time_ = 0.0;

        bool is_write_ = false;
        std::ofstream writeFile;

        // Robot State
        Eigen::Vector7d q_;
        Eigen::Vector7d q_dot_;
        Eigen::Vector7d effort_;

        Eigen::Isometry3d x_;
        Eigen::VectorXd x_dot_;
        Eigen::MatrixXd j_temp_;
        Eigen::MatrixXd j_;

        // Control
        Eigen::VectorXd q_ddot_desired_;
        Eigen::VectorXd q_dot_desired_;
        Eigen::VectorXd q_desired_;

        Eigen::Isometry3d x_target_;
        Eigen::Isometry3d x_desired_;
        Eigen::VectorXd x_dot_desired_;
        Eigen::Isometry3d x_ddot_desired_;

        Eigen::MatrixXd kv, kp;
        Eigen::MatrixXd kv_task_, kp_task_;

        Eigen::VectorXd control_input_;
        Eigen::Vector7d control_input_filtered_;

        // Kinematics & Dynamics
        RigidBodyDynamics::Model robot_;
        Eigen::VectorXd non_linear_;
        Eigen::MatrixXd A_;
        Eigen::MatrixXd C_;
        Eigen::VectorXd g_;
        
        Eigen::MatrixXd Lambda_;

};