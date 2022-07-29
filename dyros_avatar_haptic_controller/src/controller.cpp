#include "dyros_avatar_haptic_controller/dyros_avatar_haptic_controller.h"

DyrosAvatarHapticController::DyrosAvatarHapticController(ros::NodeHandle &nh, DataContainer &dc, int control_mode) : dc_(dc)
{
    if (control_mode == 0)
        dc_.sim_mode_ = "position";
    else if (control_mode == 1)
        dc_.sim_mode_ = "torque";

    // RBDL
    std::string urdf_name = ros::package::getPath("dyros_avatar_haptic_description") + "/robots/dyros_avatar_haptic.urdf";
    std::cout<<"Model name: " << urdf_name <<std::endl;
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_name.c_str(), &robot_, false, false);

    // Keyboard
    init_keyboard();
}

DyrosAvatarHapticController::~DyrosAvatarHapticController()
{

}

void DyrosAvatarHapticController::compute()
{
    ros::Rate r(hz_);
    while(ros::ok())
    {
        // if (!dc_.is_first_callback)
        // {
        //     if (!is_init_)
        //     {
        //         // Robot State

        //         is_init_ = true;
        //     }

        //     if (is_init_)
        //     {
        //         cur_time_= ros::Time::now().toSec() - init_time_;

        //         m_dc_.lock();
        //         sim_time_ = dc_.sim_time_;
        //         q_ = dc_.q_;
        //         q_dot_ = dc_.q_dot_;
        //         effort_ = dc_.effort_;
        //         m_dc_.unlock();

        //         updateKinematicsDynamics();


        //         computeControlInput();  

        //         // printData();            
 

        //         pre_time_ = cur_time_;
        //     }

        //     if (_kbhit()) {
        //         int ch = _getch();
        //         _putch(ch);
        //         mode_ = ch;

        //         mode_init_time_ = ros::Time::now().toSec() - init_time_;
        //         q_mode_init_ = q_;
        //         q_dot_mode_init_ = q_dot_;
        //         x_mode_init_ = x_;

        //         std::cout << "Mode Changed to: ";   // i: 105, r: 114, m: 109, s: 115, f:102, h: 104
        //         switch(mode_)
        //         {
        //             case(104):
        //                 std::cout << "Home Pose" << std::endl;
        //                 break;
        //             case(105):
        //                 std::cout<< "Init Pose" << std::endl;
        //                 break;
        //             case(114):
        //                 std::cout<< "Random Trajectory" << std::endl;
        //                 break;
        //             case(102):
        //                 std::cout << "Force Control" << std::endl;
        //                 break;
        //             case(115):
        //                 std::cout << "Stop" << std::endl;
        //                 break;
        //         }
        //     }
        ros::spinOnce();
        // r.sleep();
        // }
    }
    close_keyboard();
}

void DyrosAvatarHapticController::updateKinematicsDynamics()
{
    // static const int BODY_ID = robot_.GetBodyId("panda_link7");

    // x_.translation().setZero();
    // x_.linear().setZero();
    // x_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot_, q_, BODY_ID, Eigen::Vector3d(0.0, 0.0, 0.0), true);
    // x_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(robot_, q_, BODY_ID, true).transpose();
    
    // j_temp_.setZero();
    // RigidBodyDynamics::CalcPointJacobian6D(robot_, q_, BODY_ID, Eigen::Vector3d(0.0, 0.0, 0.0), j_temp_, true);
    // j_.setZero();
    // for (int i = 0; i<2; i++)
	// {
	// 	j_.block<3, 7>(i * 3, 0) = j_temp_.block<3, 7>(3 - i * 3, 0);
	// }    

    // x_dot_ = j_ * q_dot_;

    // non_linear_.setZero();
    // RigidBodyDynamics::NonlinearEffects(robot_, q_, q_dot_, non_linear_);

    // g_.setZero();
    // Eigen::Vector7d q_dot_zero;
    // q_dot_zero.setZero();
    // RigidBodyDynamics::NonlinearEffects(robot_, q_, q_dot_zero, g_);

    // A_.setZero();
    // RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot_, q_, A_, true);

    // Lambda_ = (j_ * A_.inverse() * j_.transpose()).inverse();
}

void DyrosAvatarHapticController::computeControlInput()
{
    // double traj_duration = 5.0;

    // Eigen::VectorXd f_star;
    // f_star.resize(6);

    // // Position control y, z
    // x_target_.translation() << 0.3, 0.0, 0.8;
    // x_target_.linear() << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0;

    // for (int i = 0; i < 3; i++)
    // {
    //     x_desired_.translation()(i) = cubic(cur_time_, mode_init_time_, mode_init_time_+traj_duration, x_mode_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
    //     x_dot_desired_(i) = cubicDot(cur_time_, mode_init_time_, mode_init_time_+traj_duration, x_mode_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
    // }
    
    // x_desired_.linear() = rotationCubic(cur_time_, mode_init_time_, mode_init_time_+traj_duration, x_mode_init_.linear(), x_target_.linear()); 
    // x_dot_desired_.segment(3,3) = rotationCubicDot(cur_time_, mode_init_time_, mode_init_time_+traj_duration, x_mode_init_.linear(), x_target_.linear()); 

    // Eigen::VectorXd x_error;
    // x_error.resize(6);
    // x_error.setZero();
    // Eigen::VectorXd x_dot_error;
    // x_dot_error.resize(6);
    // x_dot_error.setZero();

    // x_error.segment(0,3) = x_desired_.translation() - x_.translation();
    // x_error.segment(3,3) = -getPhi(x_.linear(), x_desired_.linear());
    // x_dot_error.segment(0,3)= x_dot_desired_.segment(0,3) - x_dot_.segment(0,3);
    // x_dot_error.segment(3,3)= x_dot_desired_.segment(3,3) - x_dot_.segment(3,3);

    // f_star = kp_task_*x_error +kv_task_*x_dot_error;

    // // Force control x
    // f_d_x_ = cubic(cur_time_, mode_init_time_, mode_init_time_+traj_duration, estimated_ext_force_init_(0), 40.0, 0.0, 0.0);
    // double k_p_force = 0.05;
    // double k_v_force = 0.001;
    
    // f_star(0) = k_p_force*(f_d_x_ - estimated_ext_force_(0)) + k_v_force*(estimated_ext_force_(0) - estimated_ext_force_pre_(0))/hz_;
    
    // estimated_ext_force_pre_ = estimated_ext_force_;

    // // Eigen::VectorXd F_d;
    // // F_d.resize(6);
    // // F_d.setZero();
    // // F_d(0) = f_d_x_;

    // // control_input_ = j_.transpose()*(Lambda_*f_star + F_d) + non_linear_;

    // f_star(0) = 0.0;
    // f_I_ = f_I_ + 1.0 * (f_d_x_ - estimated_ext_force_(0)) / hz_;
    

    // Eigen::VectorXd F_d;
    // F_d.resize(6);
    // F_d.setZero();
    // F_d(0) = f_d_x_ + f_I_;
    
    // control_input_ = j_.transpose()*(Lambda_*f_star + F_d) + non_linear_;
    
    dc_.control_input_ = control_input_; 
}

void DyrosAvatarHapticController::printData()
{ 
    if (int(cur_time_*10) != int(pre_time_*10))
    {

    }
}