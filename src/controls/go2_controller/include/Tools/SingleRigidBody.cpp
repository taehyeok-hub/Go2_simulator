#include "SingleRigidBody.hpp"

SingleRigidBody::SingleRigidBody()
{
    M = 6.921;
    gravity << 0, 0, -9.81;
    I_body << 0.02448, 0.00012166, 0.0014849, 0.00012166, 0.098077, -3.12E-05, 0.0014849, -3.12E-05, 0.107;

    p_com_world.setZero();
    R_world_to_body.setZero();
    R_body_to_world.setZero();
    
    A_mat.setZero();
    F_world.setZero();
    b_vec.setZero();
    
    F_world.resize(12);
    b_vec.resize(6);

    // QP 초기조건
    Hessian.resize(NUM_LEG * NUM_AXIS, NUM_LEG * NUM_AXIS);
    Gradient.resize(NUM_LEG * NUM_AXIS);
    LinearMatrix.resize(NUM_LEG * 5, 12);
    UpperBound.resize(NUM_LEG * 5);
    LowerBound.resize(NUM_LEG * 5);

    alpha = 1e-6;
    Q.setIdentity(); // Q 행렬 가중치는 튜닝이 필요할 수 있네.
    Compute_LinearMatrix();
    Compute_Constraint();
}

SingleRigidBody::~SingleRigidBody() {}

void SingleRigidBody::SetRobotState(const Eigen::VectorXd &p, const Eigen::VectorXd &p_dot, const Eigen::VectorXd &quat, const Eigen::VectorXd &rpy, const Eigen::VectorXd &rpy_dot)
{
    p_com_world = p;
    v_com_world = p_dot;
    rpy_world = rpy;
    omega_world = rpy_dot;

    quat_world = Eigen::Quaterniond(quat(3), quat(0), quat(1), quat(2));
    quat_world.normalized();

    R_body_to_world = quat_world.toRotationMatrix(); // body -> world
    R_world_to_body = R_body_to_world.transpose();   // world -> body
}

void SingleRigidBody::SetFootPosition(const std::array<Eigen::Vector3d, 4> &feet_pos, Eigen::VectorXd &p_com)
{
    // parameter로 들어올 변수 -> EE_Pose_FL, EE_Pose_FR, EE_Pose_RL, EE_Pose_RR --> FK 결과
    // p_com --> IMU 센서로 들여옴.
    p_foot_world = feet_pos;
    p_com_world = p_com;

    Update_A_Matrix();
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
    Eigen::Matrix3d R_error = R2 * R1.transpose();

    Eigen::AngleAxisd angle_axis_err(R_error);
    Eigen::Vector3d error_vec = angle_axis_err.axis() * angle_axis_err.angle();

    return error_vec;
}

Eigen::Matrix3d SingleRigidBody::RPYRotationMatrix(double roll, double pitch, double yaw)
{
    Eigen::Matrix3d R_x, R_y, R_z;
    R_x <<  1,         0,          0,
            0, cos(roll), -sin(roll),
            0, sin(roll),  cos(roll);

    R_y <<  cos(pitch), 0 , sin(pitch),
            0         , 1 ,          0,
           -sin(pitch), 0 , cos(pitch);

    R_z <<  cos(yaw), -sin(yaw), 0,
            sin(yaw),  cos(yaw), 0,
            0       ,         0, 1;

    return R_z * R_y * R_x;
}

void SingleRigidBody::Update_A_Matrix()
{
    A_mat.setZero();
    Eigen::Matrix3d I_3x3 = Eigen::Matrix3d::Identity();
    std::array<Eigen::Matrix3d, 4> P_skew_foot;

    for (int i = 0; i < 4; i++)
    {
        // r_leg_w = p_leg_w[i] - p_com_w  -----> r_leg_b = R * r_leg_w
        Eigen::Vector3d r_foot_world = p_foot_world[i] - p_com_world;
        P_skew_foot[i] = MakeCross2Skew(r_foot_world);

        A_mat.block<3, 3>(0, 3 * i) = I_3x3;
        A_mat.block<3, 3>(3, 3 * i) = R_world_to_body * P_skew_foot[i];
    }
}

void SingleRigidBody::Compute_b_Vector(const Eigen::Vector3d &p_des_world, const Eigen::Matrix3d &R_des_body,
                                       double Kp_pos, double Kd_pos, double Kp_ori, double Kd_ori) // Wrench 는 힘과 토크를 하나로 묶어 부르는 용어임.
{
    Kp_Pos.diagonal() << Kp_pos, Kp_pos, Kp_pos;
    Kd_Pos.diagonal() << Kd_pos, Kd_pos, Kd_pos;
    Kp_Ori.diagonal() << Kp_ori, Kp_ori, Kp_ori;
    Kd_Ori.diagonal() << Kd_ori, Kd_ori, Kd_ori;

    Eigen::Vector3d omega_body = R_world_to_body * omega_world;

    b_vec.segment<3>(0) = (-1) * M * gravity + (Kp_Pos * (p_des_world - p_com_world) - Kd_Pos * v_com_world);
    b_vec.segment<3>(3) = I_body * (Kp_Ori * MakeMatrix2Skew(R_world_to_body, R_des_body) - Kd_Ori * omega_body);
}

void SingleRigidBody::Solve_QP()
{
    Eigen::MatrixXd H_dense = A_mat.transpose() * A_mat;
    Hessian = H_dense.sparseView();
    Gradient = (-1) * A_mat.transpose() * b_vec;

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
        F_world(i) = QP_Solution(i);
    }
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
    double f_z_min = 0;

    for (int leg = 0; leg < NUM_LEG; leg++)
    {
        LowerBound.segment<5>(5 * leg) << -inf, 0, -inf, 0, f_z_min;
        UpperBound.segment<5>(5 * leg) << 0, inf, 0, inf, f_z_max;
    }
}

void SingleRigidBody::Solve_Force()
{
    Compute_Constraint();
    Solve_QP();
}

/*
먼저
어차피 객체로 접근할 수 있으니까,
객체로 ,
*/