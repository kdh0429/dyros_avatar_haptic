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

# define MODE_FORCE 102
# define MODE_HOME 104
# define MODE_STOP 115

class DyrosAvatarHapticController{
    public:
        DyrosAvatarHapticController(ros::NodeHandle &nh, DataContainer &dc, int control_mode, std::mutex &m_dc);
        ~DyrosAvatarHapticController();
        void compute();
        void updateKinematicsDynamics();
        void computeControlInput();
        void printData();
        Eigen::Matrix6d getC(Eigen::Vector6d q, Eigen::Vector6d q_dot, RigidBodyDynamics::Model robot_model);
        Eigen::Vector6d computeMOB();

    private:
        const int dof_ = 6;
        double hz_ = 2000;
        double cur_time_;
        double pre_time_;
        double init_time_;

        int mode_ = 0;
        double mode_init_time_ =  0.0;

        std::mutex &m_dc_;

        DataContainer &dc_;

        bool is_init_ = false;

        double sim_time_ = 0.0;

        bool is_write_ = false;
        std::ofstream writeFile;

        // Robot State
        struct RobotState
        {
            std::string id_;
            Eigen::Vector6d q_mode_init_;
            Eigen::Vector6d q_dot_mode_init_;
            Eigen::Isometry3d x_mode_init_;

            Eigen::Vector6d q_;
            Eigen::Vector6d q_init_;
            Eigen::Vector6d q_dot_;
            Eigen::Vector6d effort_;


            // Control
            Eigen::Vector6d q_ddot_desired_;
            Eigen::Vector6d q_dot_desired_;
            Eigen::Vector6d q_desired_;

            Eigen::Isometry3d x_target_;
            Eigen::Isometry3d x_desired_;
            Eigen::VectorXd x_dot_desired_;
            Eigen::Isometry3d x_ddot_desired_;

            Eigen::Matrix6d kv, kp;
            Eigen::Matrix6d kv_task_, kp_task_;

            Eigen::Vector6d F_I_;
            Eigen::Vector6d F_d_;
            double kp_force_ = 0.1;
            double ki_force_ = 0.2;

            Eigen::Vector6d control_input_;
            Eigen::Vector6d control_input_init_;

            // Kinematics & Dynamics
            RigidBodyDynamics::Model robot_model;
            RigidBodyDynamics::Model &robot_model_ = robot_model;
            Eigen::VectorXd non_linear_;
            Eigen::MatrixXd A_;
            Eigen::MatrixXd C_;
            Eigen::MatrixXd C_T_;
            Eigen::VectorXd g_;
            Eigen::Isometry3d x_;
            Eigen::Vector6d x_dot_;
            Eigen::MatrixXd j_temp_;
            Eigen::MatrixXd j_;
            Eigen::MatrixXd j_dyn_cons_inv_T_;
            
            Eigen::MatrixXd Lambda_;

            // MOB
            Eigen::Vector6d integral_term_mob_;
            Eigen::Vector6d residual_mob_;
            Eigen::Matrix6d K_mob_;
        };
        RobotState right_arm_;
        RobotState left_arm_;

        std::vector<RobotState*> robots_;
};