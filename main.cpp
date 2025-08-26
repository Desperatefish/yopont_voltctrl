#include <QCoreApplication>
#include <QTimer>
#include "disareavoltctrl.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    qSetMessagePattern("[%{time yyyy-MM-dd hh:mm:ss.zzz}] "
                       "%{type} (%{function}) - %{message}");

    // —— 1) 闭环：保存“上一周期”的功率与电压，作为下一周期输入 ——
    static float aP = 1.0f,  bP = 1.0f,  cP = 1.0f;   // 上周期有功(kW)
    static float aV = 240.0f, bV = 220.0f, cV = 230.0f;  // 上周期电压(V)
    static int16_t pcsStatus=0;

    // —— 2) 简易“电网响应模型”参数（按需调整）——
    const float kVPerKW_A = 1.00f;   // A相灵敏度: 每+1kW 电压增加 1V
    const float kVPerKW_B = 0.70f;
    const float kVPerKW_C = 0.85f;
    const float beta      = 0.50f;   // 一阶惯性(0..1)，越大响应越快
    const float VenvA     = 240.0f;  // 近似环境/设想“无功率”时的母线电压
    const float VenvB     = 220.0f;
    const float VenvC     = 235.0f;
    const float Vmin      = 180.0f, Vmax = 260.0f; // 仿真电压夹紧范围

    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&](){

        DistAreaVoltCtrlParams p{};

        // 使能 & 运行态
        p.voltageRegulationEnable = 1;
        p.pcsRtvRunStatus = pcsStatus;

        // 通讯前进（不能重复，否则策略会“跳过控制”）
        static quint16 pcsComm = 0, bmsHb = 1000;
        p.pcsCommStatus = ++pcsComm;
        p.bmsCommStatusHeartbeat = ++bmsHb;

        // 各相额定功率（kVA）
        p.pcsAPhaseRatedPower = 35.0f;
        p.pcsBPhaseRatedPower = 35.0f;
        p.pcsCPhaseRatedPower = 35.0f;

        // 实时无功功率（kVar）
        p.pcsAPhaseRtvReactivePower = 1.0f;
        p.pcsBPhaseRtvReactivePower = 1.0f;
        p.pcsCPhaseRtvReactivePower = 1.0f;

        // —— 3) 用“上一周期模拟出来的电压 aV/bV/cV”作为本周期测得电压 ——
        p.pcsAPhaseVoltage = aV;
        p.pcsBPhaseVoltage = bV;
        p.pcsCPhaseVoltage = cV;

        // —— 4) 确保“当前小时”策略不是 0（停机）——
        // 简单起见，全小时都设为“高电压优先=1”
        for (int h = 0; h < 24; ++h)
            reinterpret_cast<uint16_t*>(&p.hour0StrategySetting)[h] = 1;

        // BMS 充放电允许
        p.bmsMaxChargeCurrent    = 100;
        p.bmsMaxDischargeCurrent = 100;

        // 目标电压
        p.aPhaseLowVoltTargetVoltage  = 210.0f;
        p.aPhaseHighVoltTargetVoltage = 230.0f;
        p.bPhaseLowVoltTargetVoltage  = 210.0f;
        p.bPhaseHighVoltTargetVoltage = 225.0f;
        p.cPhaseLowVoltTargetVoltage  = 210.0f;
        p.cPhaseHighVoltTargetVoltage = 230.0f;

        // 死区/步长
        p.voltageTargetDeadband = 2.0f;
        p.regulationStep        = 3.0f;

        // —— 5) 本周期“实时有功”=上周期设定（形成闭环）——
        p.pcsAPhaseRtvActivePower = aP;
        p.pcsBPhaseRtvActivePower = bP;
        p.pcsCPhaseRtvActivePower = cP;

        // 运行策略
        DistAreaVoltCtrlOutput out = DistAreaVoltCtrl_Run(&p);

        // —— 6) 获取下发功率，并作为“下一周期测量功率” ——
        if (out.pcsAPhaseRemoteActivePower.valid) {aP = out.pcsAPhaseRemoteActivePower.value;pcsStatus=2;}
        if (out.pcsBPhaseRemoteActivePower.valid) {bP = out.pcsBPhaseRemoteActivePower.value;pcsStatus=2;}
        if (out.pcsCPhaseRemoteActivePower.valid) {cP = out.pcsCPhaseRemoteActivePower.value;pcsStatus=2;}
        if (out.pcsRemoteStart.valid) pcsStatus=7;
        if (out.pcsRemoteStop.valid)  pcsStatus=0;


        // —— 7) 用简易线路模型更新“下一周期电压” ——
        //    V_target = Venv - k * P_set    （正功率放电→电压增加）
        const float aV_tgt = VenvA + kVPerKW_A * aP;
        const float bV_tgt = VenvB + kVPerKW_B * bP;
        const float cV_tgt = VenvC + kVPerKW_C * cP;

        aV += beta * (aV_tgt - aV);
        bV += beta * (bV_tgt - bV);
        cV += beta * (cV_tgt - cV);

        // 夹紧范围
        aV = std::max(Vmin, std::min(aV, Vmax));
        bV = std::max(Vmin, std::min(bV, Vmax));
        cV = std::max(Vmin, std::min(cV, Vmax));

        qInfo() << "A_set:" << aP << "A_V:" << aV
                << " | B_set:" << bP << "B_V:" << bV
                << " | C_set:" << cP << "C_V:" << cV;
    });

    timer.start(4000);
    return a.exec();
}
