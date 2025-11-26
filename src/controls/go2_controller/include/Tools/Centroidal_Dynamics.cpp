#include "Centroidal_Dynamics.hpp"

CentroidalDynamics::CentroidalDynamics()
{
    A.setZero(2 * NUM_AXIS, NUM_LEG * NUM_AXIS);
    B.setZero(2 * NUM_AXIS);

    Hessian.resize(NUM_LEG * NUM_AXIS, NUM_LEG * NUM_AXIS);
    Gradient.resize(NUM_LEG * NUM_AXIS);
    LinearMatrix.resize(3*NUM_LEG, 12);
    LowerBound.resize(3*NUM_LEG);
    UpperBound.resize(3*NUM_LEG);
    QPSolution.resize(NUM_LEG * NUM_AXIS);
    force.resize(NUM_LEG * NUM_AXIS);

    Hessian.setZero();
    Gradient.setZero();
    LinearMatrix.setZero();
    LowerBound.setZero();
    UpperBound.setZero();
    QPSolution.setZero();
    force.setZero();

    Kp_Pos.setIdentity();
    Kp_Pos.diagonal() << 30000.0, 30000.0, 30000.0;

    Kd_Pos.setIdentity();
    Kd_Pos.diagonal() << 300.0, 300.0, 500.0;

    Kp_Ori.setIdentity();
    Kp_Ori.diagonal() << 10000.0, 10000.0, 10000.0;

    Kd_Ori.setIdentity();
    Kd_Ori.diagonal() << 100.0, 100.0, 100.0;

    Body_I << 11.689, -0.00455, 1.525, -0.00455, 9.8551, -0.09272, 1.525, -0.09272, 17.845;
    gravity << 0, 0, -9.81;

    Q = Eigen::MatrixXd::Identity(6, 6);

    // for (int i = 0; i < NUM_LEG; ++i)
    // {
    //     LinearMatrix.insert(0 + 5 * i, 0 + 3 * i) = 1;
    //     LinearMatrix.insert(1 + 5 * i, 0 + 3 * i) = 1;
    //     LinearMatrix.insert(2 + 5 * i, 1 + 3 * i) = 1;
    //     LinearMatrix.insert(3 + 5 * i, 1 + 3 * i) = 1;
    //     LinearMatrix.insert(4 + 5 * i, 2 + 3 * i) = 1;

    //     LinearMatrix.insert(0 + 5 * i, 2 + 3 * i) = mu;
    //     LinearMatrix.insert(1 + 5 * i, 2 + 3 * i) = -mu;
    //     LinearMatrix.insert(2 + 5 * i, 2 + 3 * i) = mu;
    //     LinearMatrix.insert(3 + 5 * i, 2 + 3 * i) = -mu;
    // }

    for (int i = 0; i < NUM_LEG; ++i)
    {
        LinearMatrix.insert(0 + 3 * i, 0 + 3 * i) = 1;
        LinearMatrix.insert(1 + 3 * i, 1 + 3 * i) = 1;
        LinearMatrix.insert(2 + 3 * i, 2 + 3 * i) = 1;
    }
}

CentroidalDynamics::~CentroidalDynamics() {}

void CentroidalDynamics::Set_BodyState(Eigen::VectorXd Body_Pos_, Eigen::VectorXd Body_Vel_, Eigen::VectorXd Body_RPY_, Eigen::VectorXd Body_RPY_Dot_, Eigen::VectorXd Body_Quat_)
{
    Body_Pos = Body_Pos_; //world
    Body_Vel = Body_Vel_; //world
    Body_RPY = Body_RPY_; //world
    Body_RPY_Dot = Body_RPY_Dot_; //world

    Body_Quat = Eigen::Quaterniond(Body_Quat_(0), Body_Quat_(1), Body_Quat_(2), Body_Quat_(3));
    Body_Quat.normalize();
    Rz = Body_Quat.toRotationMatrix();

    Local_gravity = Rz.transpose() * gravity;

    std::cout << Local_gravity.transpose() << std::endl;

    Body_Err(X) = Body_Pos(X);
    Body_Err(Y) = Body_Pos(Y);
    Body_Err(Z) = Body_Pos(Z);
}

void CentroidalDynamics::Set_FootPosition(Eigen::VectorXd Pino_F1_, Eigen::VectorXd Pino_F2_, Eigen::VectorXd Pino_H1_, Eigen::VectorXd Pino_H2_)
{
    Pino_F1 = Pino_F1_;
    Pino_F2 = Pino_F2_;
    Pino_H1 = Pino_H1_;
    Pino_H2 = Pino_H2_;
}

void CentroidalDynamics::Set_LegJacobian(Eigen::MatrixXd F1_J_, Eigen::MatrixXd F2_J_, Eigen::MatrixXd H1_J_, Eigen::MatrixXd H2_J_)
{
    F1_J = F1_J_;
    F2_J = F2_J_;
    H1_J = H1_J_;
    H2_J = H2_J_;
}

void CentroidalDynamics::Set_GaitPhase(int Phase[])
{
    GaitPhase[F1] = Phase[F1];
    GaitPhase[F2] = Phase[F2];
    GaitPhase[H1] = Phase[H1];
    GaitPhase[H2] = Phase[H2];
}

void CentroidalDynamics::Set_Reference(Eigen::VectorXd Body_Ref_)
{
    Body_Ref = Body_Ref_;

    Eigen::Vector3d P_Ref = Body_Ref.head<3>();
    Eigen::Matrix3d R_Ref = RPYToRotationMatrix(Body_Ref(3), Body_Ref(4), Body_Ref(5));
    Eigen::Matrix3d R_Act = RPYToRotationMatrix(Body_RPY(0), Body_RPY(1), Body_RPY(2));
    Eigen::Matrix3d R_Err = R_Act.transpose() * R_Ref;

    Eigen::Quaterniond Q_Ref(Body_Ref(3), Body_Ref(4), Body_Ref(5), Body_Ref(6));
    Q_Ref.normalize();

    // Eigen::Vector3d E_r = ComputeQuatError(Q_Ref, Body_Quat);

    Eigen::Vector3d E_r = ComputeRPYError(R_Err);

    Lin_Ref = (1.0 / mass) * (Kp_Pos * (P_Ref - Body_Pos) - Kd_Pos * Body_Vel);
    Ang_Ref = Kp_Ori * E_r - Kd_Ori * Body_RPY_Dot;
    // Ang_Ref = Kp_Ori * E_r - Kd_Ori * Body_AngVel;

    // std::cout << Lin_Ref(0) << "  |  " << Lin_Ref(1) <<  "  |  " << Lin_Ref(2) << std::endl;
    // std::cout << Ang_Ref(0) << "  |  " << Ang_Ref(1) <<  "  |  " << Ang_Ref(2) << std::endl;
}

void CentroidalDynamics::Set_A_Matrix()
{
    Eigen::Matrix3d F1_cross = SkewSymmetricMatrix(Pino_F1);
    Eigen::Matrix3d F2_cross = SkewSymmetricMatrix(Pino_F2);
    Eigen::Matrix3d H1_cross = SkewSymmetricMatrix(Pino_H1);
    Eigen::Matrix3d H2_cross = SkewSymmetricMatrix(Pino_H2);

    A.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    A.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    A.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
    A.block<3, 3>(0, 9) = Eigen::Matrix3d::Identity();

    A.block<3, 3>(3, 0) = F1_cross;
    A.block<3, 3>(3, 3) = F2_cross;
    A.block<3, 3>(3, 6) = H1_cross;
    A.block<3, 3>(3, 9) = H2_cross;
}

void CentroidalDynamics::Set_B_Matrix()
{
    Eigen::Vector3d linear_force = mass * (Local_gravity + Lin_Ref);

    Eigen::Vector3d angular_torque = Body_I * Ang_Ref;

    B.block<3, 1>(0, 0) = linear_force;
    B.block<3, 1>(3, 0) = angular_torque;
}

void CentroidalDynamics::Compute_CostFunction()
{
    Eigen::MatrixXd dense_Hessian = 2 * 0.15 * A.transpose() * Q * A;
    Hessian = dense_Hessian.sparseView();

    Eigen::VectorXd dense_Gradient = -2 * 0.15 * A.transpose() * Q * B;
    Gradient = dense_Gradient;
}

void CentroidalDynamics::Compute_Constraint()
{
    int gt = 0;

    for (int leg = 0; leg < NUM_LEG; ++leg)
    {
        if (GaitPhase[leg] == 1)
        {
            gt = 0;
        }
        else if (GaitPhase[leg] == 0)
        {
            gt = 1;
        }

        // LowerBound.segment<5>(leg * 5) << 0, -OsqpEigen::INFTY, 0, -OsqpEigen::INFTY, 5 * gt;
        // UpperBound.segment<5>(leg * 5) << OsqpEigen::INFTY, 0, OsqpEigen::INFTY, 0, 2000 * gt;

        LowerBound.segment<3>(3 * leg) << -800, -800, -800;
        UpperBound.segment<3>(3 * leg) << 800, 800, 800;
    }
}

void CentroidalDynamics::Solve_QP()
{
    if (!solver.isInitialized())
    {
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        solver.data()->setNumberOfVariables(NUM_LEG * NUM_AXIS);
        solver.data()->setNumberOfConstraints(3*NUM_LEG);

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
    QPSolution = solver.getSolution();

    for (size_t i = 0; i < NUM_LEG * NUM_AXIS; i++)
    {
        force(i) = QPSolution(i);
    }
}