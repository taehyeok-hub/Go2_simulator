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
    gaitmode = TROT;
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
    case TROT:
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
                Gait_Timing[FR](i) = SWING;
                Gait_Timing[RL](i) = SWING;
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
                Gait_Timing[FL](i) = SWING;
                Gait_Timing[FR](i) = STANCE;
                Gait_Timing[RL](i) = STANCE;
                Gait_Timing[RR](i) = SWING;
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
    }
}