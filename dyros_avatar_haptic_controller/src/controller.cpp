#include "dyros_avatar_haptic_controller/dyros_avatar_haptic_controller.h"

DyrosAvatarHapticController::DyrosAvatarHapticController(ros::NodeHandle &nh, DataContainer &dc, int control_mode, std::mutex &m_dc) : dc_(dc), m_dc_(m_dc)
{
    if (control_mode == 0)
        dc_.sim_mode_ = "position";
    else if (control_mode == 1)
        dc_.sim_mode_ = "torque";

    // RBDL
    std::string urdf_name = ros::package::getPath("dyros_avatar_haptic_description") + "/urdf/dyros_avatar_haptic.urdf";
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
        if (!dc_.is_first_callback)
        {
            if (!is_init_)
            {
                // Robot State

                is_init_ = true;
                m_dc_.lock();
                sim_time_ = dc_.sim_time_;
                q_init_ << 0.1, 0.8, -1.6, 0.8, 0.0, -1.57;
                m_dc_.unlock();

                kp.setZero();
                kv.setZero();
                kp.diagonal() << 400, 400, 400, 400, 400, 400;
                kv.diagonal() << 40, 40, 40, 40, 40, 40;

                j_temp_.resize(6, dc_.num_dof_);
                j_temp_.setZero();
                j_.resize(6, dc_.num_dof_);
                j_.setZero();

                non_linear_.resize(dc_.num_dof_);
                non_linear_.setZero();
                A_.resize(dc_.num_dof_, dc_.num_dof_);
                A_.setZero();
                C_.resize(dc_.num_dof_, dc_.num_dof_);
                C_.setZero();
                C_T_.resize(dc_.num_dof_, dc_.num_dof_);
                C_T_.setZero();
                Lambda_.resize(6,6);
                Lambda_.setZero();

                // MOB
                integral_term_mob_.setZero();
                residual_mob_.setZero();
                K_mob_.setZero();
                K_mob_.diagonal() << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0;
                
                init_time_ = ros::Time::now().toSec();
            }

            if (is_init_)
            {
                cur_time_= ros::Time::now().toSec() - init_time_;

                m_dc_.lock();
                sim_time_ = dc_.sim_time_;
                q_ = dc_.q_;
                q_dot_ = dc_.q_dot_;
                effort_ = dc_.effort_;
                m_dc_.unlock();

                updateKinematicsDynamics();

                computeControlInput();  

                // printData();            
 

                pre_time_ = cur_time_;
            }

            if (_kbhit()) {
                int ch = _getch();
                _putch(ch);
                mode_ = ch;

                mode_init_time_ = ros::Time::now().toSec() - init_time_;
                q_mode_init_ = q_;
                q_dot_mode_init_ = q_dot_;
                x_mode_init_ = x_;
                control_input_init_ = control_input_;

                F_I_.setZero();

                integral_term_mob_.setZero();
                residual_mob_.setZero();

                std::cout << "Mode Changed to: ";   //   f:102, h: 104, s: 115
                switch(mode_)
                {
                    case(102):
                        std::cout << "Force Control" << std::endl;
                        break;
                    case(104):
                        std::cout << "Home Pose" << std::endl;
                        break;
                    case(115):
                        std::cout << "Stop" << std::endl;
                        break;
                }
            }
        ros::spinOnce();
        r.sleep();
        }
    }
    close_keyboard();
}

void DyrosAvatarHapticController::updateKinematicsDynamics()
{
    static const int BODY_ID = robot_.GetBodyId("R6v3_1");

    x_.translation().setZero();
    x_.linear().setZero();
    x_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot_, q_, BODY_ID, Eigen::Vector3d(0.0, 0.0, 0.0), true);
    x_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(robot_, q_, BODY_ID, true).transpose();
    
    j_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(robot_, q_, BODY_ID, Eigen::Vector3d(0.0, 0.0, 0.0), j_temp_, true);
    j_.setZero();
    for (int i = 0; i<2; i++)
	{
		j_.block<3, 6>(i * 3, 0) = j_temp_.block<3, 6>(3 - i * 3, 0);
	}    

    x_dot_ = j_ * q_dot_;

    non_linear_.setZero();
    RigidBodyDynamics::NonlinearEffects(robot_, q_, q_dot_, non_linear_);

    A_.setZero();
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot_, q_, A_, true);

    C_ = getC(q_, q_dot_);
    C_T_ = C_.transpose();

    Lambda_ = (j_ * A_.inverse() * j_.transpose()).inverse();
}

void DyrosAvatarHapticController::computeControlInput()
{

    if (mode_ == MODE_HOME)
    {
        double traj_duration = 5.0;
        for (int i = 0; i < dc_.num_dof_; i++)
        {
            q_desired_(i) = cubic(cur_time_, mode_init_time_, mode_init_time_+traj_duration, q_mode_init_(i), q_init_(i), 0.0, 0.0);
            q_dot_desired_(i) = cubicDot(cur_time_, mode_init_time_, mode_init_time_+traj_duration, q_mode_init_(i), q_init_(i), 0.0, 0.0);
        }
        control_input_ = kp * (q_desired_ - q_) +  kv * (q_dot_desired_ - q_dot_) + non_linear_;
    }
    else if (mode_ == MODE_FORCE)
    {
        computeMOB();
        F_d_ << 10.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        Eigen::Vector6d F_;
        F_I_ = F_I_ + 0.2*(F_d_ - residual_mob_) / hz_;
        F_ = 0.1*(F_d_ - residual_mob_) + F_I_;

        control_input_ = j_.transpose()*F_ + non_linear_;
    }
    else if (mode_ == MODE_STOP)
    {
        double traj_duration = 3.0;
        for (int i = 0; i < dc_.num_dof_; i++)
        {
            control_input_(i) = cubic(cur_time_, mode_init_time_, mode_init_time_+traj_duration, control_input_init_(i), 0.0, 0.0, 0.0);
        }
    }
    else
    {
        control_input_ =  non_linear_;
    }
    
    dc_.control_input_ = control_input_; 
}

Eigen::Matrix6d DyrosAvatarHapticController::getC(Eigen::Vector6d q, Eigen::Vector6d q_dot){
    double h = 2e-12;

    Eigen::VectorXd q_new;
    q_new.resize(dc_.num_dof_);
    
    Eigen::Matrix6d C, C1, C2;
    C.setZero();
    C1.setZero();
    C2.setZero();

    Eigen::MatrixXd A(dc_.num_dof_, dc_.num_dof_), A_new(dc_.num_dof_, dc_.num_dof_);
    Eigen::MatrixXd m[dc_.num_dof_];
    double b[dc_.num_dof_][dc_.num_dof_][dc_.num_dof_];

    for (int i = 0; i < dc_.num_dof_; i++)
    {
        q_new = q;
        q_new(i) += h;

        A.setZero();
        A_new.setZero();

        RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot_, q, A, true);
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot_, q_new, A_new, true);

        m[i].resize(dc_.num_dof_, dc_.num_dof_);
        m[i] = (A_new - A) / h;
    }

    for (int i = 0; i < dc_.num_dof_; i++)
        for (int j = 0; j < dc_.num_dof_; j++)
            for (int k = 0; k < dc_.num_dof_; k++)
                b[i][j][k] = 0.5 * (m[k](i, j) + m[j](i, k) - m[i](j, k));


    for (int i = 0; i < dc_.num_dof_; i++)
        for (int j = 0; j < dc_.num_dof_; j++)
            C1(i, j) = b[i][j][j] * q_dot(j);

    for (int k = 0; k < dc_.num_dof_; k++)
        for (int j = 0; j < dc_.num_dof_; j++)
            for (int i = 1 + j; i < dc_.num_dof_; i++)
            C2(k, j) += 2.0 * b[k][j][i] * q_dot(i);
    C = C1 + C2;

    return C;
}

Eigen::Vector6d DyrosAvatarHapticController::computeMOB()
{ 
    integral_term_mob_ = integral_term_mob_ + (control_input_ + C_T_*q_dot_ - non_linear_ - residual_mob_)/hz_;
    residual_mob_ = K_mob_ * (integral_term_mob_ - A_*q_dot_);
}

void DyrosAvatarHapticController::printData()
{ 
    if (int(cur_time_*10) != int(pre_time_*10))
    {

    }
}