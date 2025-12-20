#include "SingleRigidBody.hpp"

SingleRigidBody::SingleRigidBody()
{
    M = 15.0;
    gravity << 0, 0, -9.81;
    I_body << 0.02448, 0.00012166, 0.0014849, 0.00012166, 0.098077, -3.12E-05, 0.0014849, -3.12E-05, 0.107;

    p_com_world.setZero();
    v_com_world.setZero();
    rpy_world.setZero();
    rpy_dot_world.setZero();
    quat_world = Eigen::Quaterniond::Identity();
    R_world_to_body.setZero();
    R_body_to_world.setZero();

    A_mat.setZero();
    Force.setZero();
    b_vec.setZero();
    Force.resize(12);
    b_vec.resize(6);
    
    // b vector 제어 Gain값
    Kp_Pos.setIdentity();
    Kp_Pos.diagonal() << 100.0, 50.0, 100.0;

    Kd_Pos.setIdentity();
    Kd_Pos.diagonal() << 1.0, 0.50, 1.0;

    Kp_Ori.setIdentity();
    Kp_Ori.diagonal() << 80.0, 100.0, 100.0;

    Kd_Ori.setIdentity();
    Kd_Ori.diagonal() << 0.80, 1.0, 1.0;

    // QP 초기조건
    Hessian.resize(NUM_LEG * NUM_AXIS, NUM_LEG * NUM_AXIS);
    Gradient.resize(NUM_LEG * NUM_AXIS);
    LinearMatrix.resize(NUM_LEG * 5, 12);
    UpperBound.resize(NUM_LEG * 5);
    LowerBound.resize(NUM_LEG * 5);

    alpha = 0.5;
    Q = Eigen::MatrixXd::Identity(6,6);

    Compute_LinearMatrix();
    Compute_Constraint();
}

SingleRigidBody::~SingleRigidBody() {}

void SingleRigidBody::SetRobotState(const Eigen::VectorXd &p, const Eigen::VectorXd &p_dot, const Eigen::VectorXd &quat, const Eigen::VectorXd &rpy, const Eigen::VectorXd &rpy_dot)
{
    p_com_world = p;
    v_com_world = p_dot;
    rpy_world = rpy;
    rpy_dot_world = rpy_dot;

    quat_world = Eigen::Quaterniond(quat(3), quat(0), quat(1), quat(2));
    quat_world.normalize();

    R_body_to_world = quat_world.toRotationMatrix(); // body -> world
    R_world_to_body = R_body_to_world.transpose();   // world -> body

    gravity_body = R_world_to_body * gravity;

    std::cout << "p_com_world = " << p_com_world << std::endl;
    std::cout << "v_com_world = " << v_com_world << std::endl;
    std::cout << "rpy_world = " << rpy_world << std::endl;
    std::cout << "rpy_dot_world = " << rpy_dot_world << std::endl;
    std::cout << "gravity_body = " << gravity_body << std::endl;
    std::cout << "R_world_to_body = " << R_world_to_body << std::endl;
    std::cout << "R_body_to_world = " << R_body_to_world << std::endl;
}

void SingleRigidBody::SetFootPosition(const std::array<Eigen::Vector3d, 4> &feet_pos_body)
{
    // parameter로 들어올 변수 -> EE_Pose_FL, EE_Pose_FR, EE_Pose_RL, EE_Pose_RR --> FK 결과 (body)
    // p_com --> IMU 센서로 들여옴.
    for (int i = 0; i < 4; i++)
    {
        p_foot_world[i] = R_body_to_world * feet_pos_body[i];
    }
    std::cout << "FL_foot_world = " << p_foot_world[0].transpose() << std::endl;
    std::cout << "FR_foot_world = " << p_foot_world[1].transpose() << std::endl;
    std::cout << "RL_foot_world = " << p_foot_world[2].transpose() << std::endl;
    std::cout << "RR_foot_world = " << p_foot_world[3].transpose() << std::endl;
}

Eigen::Matrix3d SingleRigidBody::MakeCross2Skew(const Eigen::Vector3d &p)
{
    Eigen::Matrix3d P_skew;
    P_skew << 0, -p(2), p(1),
        p(2), 0, -p(0),
        -p(1), p(0), 0;
    return P_skew;
}

Eigen::Vector3d SingleRigidBody::MakeMatrix2Skew(const Eigen::Matrix3d &R1, const Eigen::Matrix3d &R2)
{
    // R2(ref) = Ree * R1(act)
    Eigen::Matrix3d R_error = R2.transpose() * R1;
    Eigen::Matrix3d S = (R_error - R_error.transpose()) / 2.0;
    Eigen::Vector3d Error_Vec;
    Error_Vec << S(2, 1) - S(1, 2), S(0, 2) - S(2, 0), S(1, 0) - S(0, 1);

    return 0.5 * Error_Vec;

    std::cout << "Error_vec_body = " << 0.5 * Error_Vec.transpose() << std::endl;

    // 성민이 형
    // Eigen::Vector3d E_r;
    // E_r << R_err(2, 1) - R_err(1, 2), R_err(0, 2) - R_err(2, 0), R_err(1, 0) - R_err(0, 1);
    // return 0.5 * E_r;
}

Eigen::Matrix3d SingleRigidBody::RPYRotationMatrix(double roll, double pitch, double yaw)
{
    Eigen::Matrix3d R_x, R_y, R_z;
    R_x << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);

    R_y << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);

    R_z << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;

    return R_z * R_y * R_x;
}

void SingleRigidBody::Update_A_Matrix()
{
    A_mat.setZero();
    Eigen::Matrix3d I_3x3 = Eigen::Matrix3d::Identity();
    std::array<Eigen::Matrix3d, 4> P_skew_foot;

    for (int i = 0; i < 4; i++)
    {
        P_skew_foot[i] = MakeCross2Skew(p_foot_world[i]);

        A_mat.block<3, 3>(0, 3 * i) = I_3x3;
        A_mat.block<3, 3>(3, 3 * i) = P_skew_foot[i];
    }

    std::cout << "A = " << A_mat << std::endl;
}

void SingleRigidBody::Compute_b_Vector(const Eigen::Vector3d &p_des_world, const Eigen::Matrix3d &R_des_body, Eigen::Vector3d rpy_des_error_world) // Wrench 는 힘과 토크를 하나로 묶어 부르는 용어임.
{
    Eigen::Vector3d p_des_err_world = p_des_world - p_com_world;
    Eigen::Vector3d p_des_err_body = R_world_to_body * p_des_err_world;
    Eigen::Vector3d rpy_des_error_body = R_world_to_body * rpy_des_error_world;

    Eigen::Vector3d v_com_body = R_world_to_body * v_com_world;
    Eigen::Vector3d rpy_dot_body = R_world_to_body * rpy_dot_world;
    Eigen::Vector3d skew_error = MakeMatrix2Skew(R_world_to_body, R_des_body);

    b_vec.segment<3>(0) = -M * gravity_body + (Kp_Pos * p_des_err_body - Kd_Pos * v_com_body);
    b_vec.segment<3>(3) = I_body * (Kp_Ori * skew_error - Kd_Ori * rpy_dot_body);
        
    std::cout << "p_des_err_world = "<< p_des_err_world << std::endl;
    std::cout << "p_des_err_body = "<< p_des_err_body << std::endl;
    std::cout << "rpy_des_error_body = " << rpy_des_error_body << std::endl; 
    std::cout << "skew_error = " << skew_error << std::endl;

    // std::cout << "b = " << b_vec << std::endl;
}

void SingleRigidBody::Set_CostFunction()
{
    Eigen::MatrixXd H_dense = A_mat.transpose() * Q * A_mat;
    Hessian = H_dense.sparseView();
    Gradient = (-1) * A_mat.transpose() * Q * b_vec;

    std::cout << "Hessian = " << std::endl << Hessian << std::endl;
    std::cout << "Gradient = " << std::endl << Gradient << std::endl;
    // std::cout << "LinearMatrix = " << std::endl << LinearMatrix << std::endl;
    // std::cout << "LowerBound = " << std::endl << LowerBound << std::endl;
    // std::cout << "UpperBound = " << std::endl << UpperBound << std::endl;
}

void SingleRigidBody::Solve_QP()
{
    Set_CostFunction();

    if (!solver.isInitialized())
    {
        solver.settings()->setVerbosity(true);
        solver.settings()->setWarmStart(true);
        solver.data()->setNumberOfVariables(NUM_LEG * NUM_AXIS);
        solver.data()->setNumberOfConstraints(num_of_constraints * NUM_LEG);

        solver.data()->setHessianMatrix(Hessian);
        solver.data()->setGradient(Gradient);
        solver.data()->setLinearConstraintsMatrix(LinearMatrix);
        solver.data()->setLowerBound(LowerBound);
        solver.data()->setUpperBound(UpperBound);
        solver.initSolver();
    }
    else
    {
        solver.updateHessianMatrix(Hessian);
        solver.updateGradient(Gradient);
        solver.updateBounds(LowerBound, UpperBound);
    }

    solver.solveProblem();

    QP_Solution = solver.getSolution();

    for (int i = 0; i < NUM_LEG * NUM_AXIS; i++)
    {
        Force(i) = QP_Solution(i);
    }

    Eigen::VectorXd residual = A_mat * Force - b_vec;
    std::cout << "residual: " << residual.transpose() << std::endl;
    Eigen::VectorXd AF = A_mat * Force;
    std::cout << "A*F = " << AF.transpose() << std::endl;
    std::cout << "b   = " << b_vec.transpose() << std::endl;
}

void SingleRigidBody::Compute_LinearMatrix(double friction_mu)
{
    mu = friction_mu / sqrt(2);

    // linearmatrix 모두 0으로 만들고,
    LinearMatrix.setZero();

    for (int leg = 0; leg < NUM_LEG; leg++)
    {
        LinearMatrix.insert(0 + 5 * leg, 0 + 3 * leg) = 1;
        LinearMatrix.insert(1 + 5 * leg, 0 + 3 * leg) = 1;
        LinearMatrix.insert(2 + 5 * leg, 1 + 3 * leg) = 1;
        LinearMatrix.insert(3 + 5 * leg, 1 + 3 * leg) = 1;
        LinearMatrix.insert(4 + 5 * leg, 2 + 3 * leg) = 1;

        LinearMatrix.insert(0 + 5 * leg, 2 + 3 * leg) = -mu;
        LinearMatrix.insert(1 + 5 * leg, 2 + 3 * leg) = mu;
        LinearMatrix.insert(2 + 5 * leg, 2 + 3 * leg) = -mu;
        LinearMatrix.insert(3 + 5 * leg, 2 + 3 * leg) = mu;
    }
}

void SingleRigidBody::Compute_Constraint(double f_z_max)
{
    const double inf = OsqpEigen::INFTY;
    double f_z_min = 1e-6;

    for (int leg = 0; leg < NUM_LEG; leg++)
    {
        LowerBound.segment<5>(5 * leg) << -inf, 0, -inf, 0, f_z_min;
        UpperBound.segment<5>(5 * leg) << 0, inf, 0, inf, f_z_max;
    }
}

/*
먼저
어차피 객체로 접근할 수 있으니까,
객체로 ,
*/