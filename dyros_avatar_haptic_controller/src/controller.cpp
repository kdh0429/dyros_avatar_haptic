#include "dyros_avatar_haptic_controller/dyros_avatar_haptic_controller.h"

DyrosAvatarHapticController::DyrosAvatarHapticController(ros::NodeHandle &nh, DataContainer &dc, int control_mode, std::mutex &m_dc) : dc_(dc), m_dc_(m_dc)
{
    if (control_mode == 0)
        dc_.sim_mode_ = "position";
    else if (control_mode == 1)
        dc_.sim_mode_ = "torque";

    right_arm_.id_ = "right_arm";
    left_arm_.id_ = "left_arm";
    robots_.push_back(&right_arm_);
    robots_.push_back(&left_arm_);

    // RBDL
    std::string urdf_name = ros::package::getPath("dyros_avatar_haptic_description") + "/urdf/dyros_avatar_haptic_right.urdf";
    std::cout<<"Right Robot Model name: " << urdf_name <<std::endl;
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_name.c_str(), &right_arm_.robot_model_, false, false);
    urdf_name = ros::package::getPath("dyros_avatar_haptic_description") + "/urdf/dyros_avatar_haptic_left.urdf";
    std::cout<<"Left Robot Model name: " << urdf_name <<std::endl;
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_name.c_str(), &left_arm_.robot_model_, false, false);

    // FT Callback
    ft_sub_ = nh.subscribe("/tocabi/hand_ftsensors", 1, &DyrosAvatarHapticController::FTCallback, this, ros::TransportHints().tcpNoDelay(true));
    right_arm_.hand_pub_ = nh.advertise<tocabi_msgs::matrix_3_4>("/TRACKER5HAPTIC", 100);
    left_arm_.hand_pub_ = nh.advertise<tocabi_msgs::matrix_3_4>("/TRACKER3HAPTIC", 100);

    // Keyboard
    init_keyboard();

}

DyrosAvatarHapticController::~DyrosAvatarHapticController()
{

}


void DyrosAvatarHapticController::FTCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for (int i = 0; i <6; i++)
    {
        right_arm_.F_d_(i) = msg->data[i];
        left_arm_.F_d_(i) = msg->data[i+6];
    }
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
                sim_time_ = dc_.sim_time_;

                for (auto &robot: robots_)
                {
                    if (robot->id_.compare("right_arm") == 0){
                        robot->q_init_ << 0.1, 0.8, -1.6, 0.8, 0.0, -1.57;
                    }
                    else{
                        robot->q_init_ << 0.1, 0.8, 1.6, -0.8, 0.0, 1.57;
                    }

                    robot->kp.setZero();
                    robot->kv.setZero();
                    robot->kp.diagonal() << 400, 400, 400, 400, 400, 400;
                    robot->kv.diagonal() << 10, 10, 10, 10, 10, 10;

                    robot->j_temp_.resize(6, dof_);
                    robot->j_temp_.setZero();
                    robot->j_.resize(6, dof_);
                    robot->j_.setZero();
                    robot->j_dyn_cons_inv_T_.resize(dof_, 6);
                    robot->j_dyn_cons_inv_T_.setZero();

                    robot->non_linear_.resize(dof_);
                    robot->non_linear_.setZero();
                    robot->A_.resize(dof_, dof_);
                    robot->A_.setZero();
                    robot->C_.resize(dof_, dof_);
                    robot->C_.setZero();
                    robot->C_T_.resize(dof_, dof_);
                    robot->C_T_.setZero();
                    robot->Lambda_.resize(6,6);
                    robot->Lambda_.setZero();

                    // MOB
                    robot->integral_term_mob_.setZero();
                    robot->residual_mob_.setZero();
                    robot->K_mob_.setZero();
                    robot->K_mob_.diagonal() << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0;
                }
                init_time_ = ros::Time::now().toSec();
            }

            if (is_init_)
            {
                cur_time_= ros::Time::now().toSec() - init_time_;

                m_dc_.lock();
                sim_time_ = dc_.sim_time_;
                right_arm_.q_ = dc_.q_.head(6);
                left_arm_.q_ = dc_.q_.tail(6);
                right_arm_.q_dot_ = dc_.q_dot_.head(6);
                left_arm_.q_dot_ = dc_.q_dot_.tail(6);
                right_arm_.effort_ = dc_.effort_.head(6);
                left_arm_.effort_ = dc_.effort_.tail(6);
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

                for (auto &robot: robots_)
                {
                    robot->q_mode_init_ = robot->q_;
                    robot->q_dot_mode_init_ = robot->q_dot_;
                    robot->x_mode_init_ = robot->x_;
                    robot->control_input_init_ = robot->control_input_;
                        
                    robot->F_I_.setZero();

                    robot->integral_term_mob_.setZero();
                    robot->residual_mob_.setZero();
                }

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
    // close_keyboard();
}

void DyrosAvatarHapticController::updateKinematicsDynamics()
{
    for (auto &robot: robots_)
    {
        static const int BODY_ID = robot->robot_model_.GetBodyId("EE");

        robot->x_.translation().setZero();
        robot->x_.linear().setZero();
        robot->x_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot->robot_model_, robot->q_, BODY_ID, Eigen::Vector3d(0.0, 0.0, 0.0), true);
        robot->x_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(robot->robot_model_, robot->q_, BODY_ID, true).transpose();
        
        robot->j_temp_.setZero();
        RigidBodyDynamics::CalcPointJacobian6D(robot->robot_model_, robot->q_, BODY_ID, Eigen::Vector3d(0.0, 0.0, 0.0), robot->j_temp_, true);
        robot->j_.setZero();
        for (int i = 0; i<2; i++)
        {
            robot->j_.block<3, 6>(i * 3, 0) = robot->j_temp_.block<3, 6>(3 - i * 3, 0);
        }    

        robot->x_dot_ = robot->j_ * robot->q_dot_;
        robot->non_linear_.setZero();
        RigidBodyDynamics::NonlinearEffects(robot->robot_model_, robot->q_, robot->q_dot_, robot->non_linear_);

        robot->A_.setZero();
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot->robot_model_, robot->q_, robot->A_, true);

        robot->C_ = getC(robot->q_, robot->q_dot_, robot->robot_model_);
        robot->C_T_ = robot->C_.transpose();

        robot->Lambda_ = (robot->j_ * robot->A_.inverse() * robot->j_.transpose()).inverse();Eigen::Matrix<double, 6,6> I;
        I.setIdentity();
        robot->j_dyn_cons_inv_T_ = (robot->j_*robot->A_.inverse()*robot->j_.transpose() + 0.001*I).inverse() * robot->j_ * robot->A_.inverse();

        // ROS MSG
        robot->hand_msg_.firstRow.resize(4);
        robot->hand_msg_.firstRow[0] = robot->x_.linear()(0,0);
        robot->hand_msg_.firstRow[1] = robot->x_.linear()(0,1);
        robot->hand_msg_.firstRow[2] = robot->x_.linear()(0,2);
        robot->hand_msg_.firstRow[3] = robot->x_.translation()(0);

        robot->hand_msg_.secondRow.resize(4);
        robot->hand_msg_.secondRow[0] = robot->x_.linear()(1,0);
        robot->hand_msg_.secondRow[1] = robot->x_.linear()(1,1);
        robot->hand_msg_.secondRow[2] = robot->x_.linear()(1,2);
        robot->hand_msg_.secondRow[3] = robot->x_.translation()(1);

        robot->hand_msg_.thirdRow.resize(4);
        robot->hand_msg_.thirdRow[0] = robot->x_.linear()(2,0);
        robot->hand_msg_.thirdRow[1] = robot->x_.linear()(2,1);
        robot->hand_msg_.thirdRow[2] = robot->x_.linear()(2,2);
        robot->hand_msg_.thirdRow[3] = robot->x_.translation()(2);

        robot->hand_pub_.publish(robot->hand_msg_);
    }
}

void DyrosAvatarHapticController::computeControlInput()
{
    if (mode_ == MODE_HOME)
    {
        double traj_duration = 5.0;
        for (auto &robot: robots_)
        {
            for (int i = 0; i < dof_; i++)
            {
                robot->q_desired_(i) = cubic(cur_time_, mode_init_time_, mode_init_time_+traj_duration, robot->q_mode_init_(i), robot->q_init_(i), 0.0, 0.0);
                robot->q_dot_desired_(i) = cubicDot(cur_time_, mode_init_time_, mode_init_time_+traj_duration, robot->q_mode_init_(i), robot->q_init_(i), 0.0, 0.0);
            }
            robot->control_input_ = robot->kp * (robot->q_desired_ - robot->q_) +  robot->kv * (robot->q_dot_desired_ - robot->q_dot_) + robot->non_linear_;
        }
    }
    else if (mode_ == MODE_FORCE)
    {
        computeMOB();
        for (auto &robot: robots_)
        {
            double traj_duration = 2.0;

            Eigen::Vector6d F_;
            robot->F_I_ = robot->F_I_ + robot->ki_force_*(robot->F_d_ - robot->j_dyn_cons_inv_T_*robot->residual_mob_) / hz_;
            // Integral Anti-Windup(saturation)
            for (int i = 0; i <6; i++)
            {
                minmax_cut(robot->F_I_(i), -10.0, 10.0);
            }
            // F_ = robot->kp_force_*(robot->F_d_ - robot->j_dyn_cons_inv_T_*robot->residual_mob_) + robot->F_I_;
            F_ = robot->F_d_;

            robot->control_input_ = robot->j_.transpose()*F_ + robot->non_linear_;
        }
    }
    else if (mode_ == MODE_STOP)
    {
        double traj_duration = 3.0;
        for (auto &robot: robots_)
        {
            for (int i = 0; i < dof_; i++)
            {
                robot->control_input_(i) = cubic(cur_time_, mode_init_time_, mode_init_time_+traj_duration, robot->control_input_init_(i), 0.0, 0.0, 0.0);
            }
        }
    }
    else
    {
        for (auto &robot: robots_)
        {
            robot->control_input_ =  robot->non_linear_;
        }
    }

    m_dc_.lock();
    dc_.control_input_.head(6) = right_arm_.control_input_; 
    dc_.control_input_.tail(6) = left_arm_.control_input_; 
    m_dc_.unlock();
}

Eigen::Matrix6d DyrosAvatarHapticController::getC(Eigen::Vector6d q, Eigen::Vector6d q_dot, RigidBodyDynamics::Model robot_model)
{
    double h = 2e-12;

    Eigen::VectorXd q_new;
    q_new.resize(dof_);
    
    Eigen::Matrix6d C, C1, C2;
    C.setZero();
    C1.setZero();
    C2.setZero();

    Eigen::MatrixXd A(dof_, dof_), A_new(dof_, dof_);
    Eigen::MatrixXd m[dof_];
    double b[dof_][dof_][dof_];

    for (int i = 0; i < dof_; i++)
    {
        q_new = q;
        q_new(i) += h;

        A.setZero();
        A_new.setZero();

        RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot_model, q, A, true);
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot_model, q_new, A_new, true);

        m[i].resize(dof_, dof_);
        m[i] = (A_new - A) / h;
    }

    for (int i = 0; i < dof_; i++)
        for (int j = 0; j < dof_; j++)
            for (int k = 0; k < dof_; k++)
                b[i][j][k] = 0.5 * (m[k](i, j) + m[j](i, k) - m[i](j, k));


    for (int i = 0; i < dof_; i++)
        for (int j = 0; j < dof_; j++)
            C1(i, j) = b[i][j][j] * q_dot(j);

    for (int k = 0; k < dof_; k++)
        for (int j = 0; j < dof_; j++)
            for (int i = 1 + j; i < dof_; i++)
            C2(k, j) += 2.0 * b[k][j][i] * q_dot(i);
    C = C1 + C2;

    return C;
}

Eigen::Vector6d DyrosAvatarHapticController::computeMOB()
{ 
    for (auto &robot: robots_)
    {
        robot->integral_term_mob_ = robot->integral_term_mob_ + (robot->control_input_ + robot->C_T_*robot->q_dot_ - robot->non_linear_ - robot->residual_mob_)/hz_;
        robot->residual_mob_ = robot->K_mob_ * (robot->integral_term_mob_ - robot->A_*robot->q_dot_);
    }
}

void DyrosAvatarHapticController::printData()
{ 
    if (int(cur_time_*10) != int(pre_time_*10))
    {

    }
}