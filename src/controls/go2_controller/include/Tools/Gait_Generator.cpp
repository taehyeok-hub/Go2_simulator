#include "Gait_Generator.hpp"

Gait_Generator::Gait_Generator()
{
    // Gait.State1 << 1.0, 1.0, 1.0, 1.0;
    // Gait.State2 << 1.0, 0.0, 0.0, 1.0;
    // Gait.State3 << 0.0, 1.0, 1.0, 0.0;

    Trot_Pattern.setZero(4, 4);
    Trot_Pattern << 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0;

    for (size_t i = 0; i < NUM_LEG; ++i)
    {
        Gait_Timing[i] = Eigen::VectorXd::Zero(2 * T_TROT);
    }
    gaitmode = STANCE_TROT; // STAND // STANCE_TROT // TROT
    Stand_Time = T_STANCE + T_SWING;
}

Gait_Generator::~Gait_Generator() {}

void Gait_Generator::Trot_Gait_Generator(double switch_time)
{
    // Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    // // 보행 주기 -> 이초마다 교차
    // int period = static_cast<int>(switch_time * 500);

    // // switch (Gait_Switch) {
    // // case 0:
    // //     Target_State = State1;
    // //     break;
    // // case 1:
    // //     Target_State = State2;
    // //     break;
    // // case 2:
    // //     Target_State = State3;
    // //     break;
    // // case 3:
    // //     Target_State = State4;
    // //     break;
    // // }

    // // if (Switch_Time < period)
    // // {
    // //     Switch_Time++;
    // // }
    // // else if (Switch_Time == period)
    // // {
    // //     Switch_Time = 0;
    // //     Gait_Switch = (Gait_Switch + 1) % 4;
    // // }

    // // 2. 공통 카운터 로직 (어떤 상태든 시간은 똑같이 흐름)
    // Switch_Time++;
    // if (Switch_Time >= period)
    // {
    //     // 0 -> 1 -> 2 -> 3 ->0 순환
    //     Gait_Switch = (Gait_Switch + 1) % 4;
    //     Switch_Time = 0;
    // }

    // Target_State = Trot_Pattern.row(Gait_Switch);
}

void Gait_Generator::Gait_Update()
{
    switch (gaitmode)
    {
    case STAND:
        if (Stand_Time > 0) // Stand_Time 은 GaitGenerator의 생성자에서 결정
        {
            for (int leg = 0; leg < NUM_LEG; leg++)
            {
                Gait_Timing[leg].setConstant(STANCE); // Gait_Timing[leg] = STANCE 랑은 다름.
            }
            Stand_Time--;
        }

        if (Stand_Time == 0) // 120 tick // T_STANCE + T_SWING // 무조건 네 다리가 stance여야하는 시간
        {
            gaitmode = TROT;
            Init_Trot = true;
        }
        break;

    case STANCE_TROT:
        if (Init_Trot)
        {
            for (size_t i = 0; i < T_STANCE; i++)
            {
                Gait_Timing[FL](i) = STANCE;
                Gait_Timing[FR](i) = STANCE;
                Gait_Timing[RL](i) = STANCE;
                Gait_Timing[RR](i) = STANCE;
            }

            for (size_t i = T_STANCE; i < (T_STANCE + T_SWING); i++)
            {
                Gait_Timing[FL](i) = STANCE;
                Gait_Timing[FR](i) = STANCE;
                Gait_Timing[RL](i) = STANCE;
                Gait_Timing[RR](i) = STANCE;
            }

            for (size_t i = (T_STANCE + T_SWING); i < (2 * T_STANCE + T_SWING); i++)
            {
                Gait_Timing[FL](i) = STANCE;
                Gait_Timing[FR](i) = STANCE;
                Gait_Timing[RL](i) = STANCE;
                Gait_Timing[RR](i) = STANCE;
            }

            for (size_t i = (2 * T_STANCE + T_SWING); i < 2 * (T_STANCE + T_SWING); i++)
            {
                Gait_Timing[FL](i) = STANCE;
                Gait_Timing[FR](i) = STANCE;
                Gait_Timing[RL](i) = STANCE;
                Gait_Timing[RR](i) = STANCE;
            }

            Init_Trot = false;
        }

        for (size_t leg = 0; leg < NUM_LEG; leg++)
        {
            temp[leg] = Gait_Timing[leg](0); // 첫째 것 미리 넣어두고,
        }

        for (size_t i = 1; i < 2 * T_TROT; i++)
        {
            Gait_Timing[FL](i - 1) = Gait_Timing[FL](i);
            Gait_Timing[FR](i - 1) = Gait_Timing[FR](i);
            Gait_Timing[RL](i - 1) = Gait_Timing[RL](i);
            Gait_Timing[RR](i - 1) = Gait_Timing[RR](i);
        }

        for (size_t leg = 0; leg < NUM_LEG; leg++)
        {
            Gait_Timing[leg](2 * T_TROT - 1) = temp[leg];
        }
        break;

    case TROT:
        if (Init_Trot)
        {
            for (int leg = 0; leg < NUM_LEG; leg++)
            {
                Gait_Timing[leg].resize(T_TROT); // T_TROT로 바꿈.
            }

            for (size_t i = 0; i < T_STANCE; i++)
            {
                Gait_Timing[FL](i) = SWING;
                Gait_Timing[FR](i) = STANCE;
                Gait_Timing[RL](i) = STANCE;
                Gait_Timing[RR](i) = SWING;
            }

            for (size_t i = T_STANCE; i < T_TROT; i++)
            {
                Gait_Timing[FL](i) = STANCE;
                Gait_Timing[FR](i) = SWING;
                Gait_Timing[RL](i) = SWING;
                Gait_Timing[RR](i) = STANCE;
            }
            Init_Trot = false;
        }

        for (size_t leg = 0; leg < NUM_LEG; leg++)
        {
            temp[leg] = Gait_Timing[leg](0); // temp는 진짜 값을 저장하는 임시변수
        }

        for (size_t i = 1; i < T_TROT; i++)
        {
            Gait_Timing[FL](i - 1) = Gait_Timing[FL](i);
            Gait_Timing[FR](i - 1) = Gait_Timing[FR](i);
            Gait_Timing[RL](i - 1) = Gait_Timing[RL](i);
            Gait_Timing[RR](i - 1) = Gait_Timing[RR](i);
        }

        for (size_t leg = 0; leg < NUM_LEG; leg++)
        {
            Gait_Timing[leg](T_TROT - 1) = temp[leg];
        }
        break;

    default:
        break;
    }
}