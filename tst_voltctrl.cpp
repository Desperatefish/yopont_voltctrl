#include <QtTest/QtTest>
#include "disareavoltctrl.h"

// ============ 小工具：按“当前小时”写 24h 策略(0/1/2) ============
static inline void setHourStrategy(DistAreaVoltCtrlParams &p, uint16_t hour, uint16_t mode) {
    // hour0StrategySetting 后面 24 个 uint16_t 连续存放
    reinterpret_cast<uint16_t*>(&p.hour0StrategySetting)[hour] = mode;
}

// ============ 小工具：状态码断言 ============
static inline void expectStatus(const DistAreaVoltCtrlOutput& out, AppStatusCode code) {
    QVERIFY2(out.appStatus.valid, "appStatus.valid should be true");
    QCOMPARE(out.appStatus.value, static_cast<uint16_t>(code));
}

// ============ 小工具：浮点近似断言 ============
static inline void expectNear(float actual, float expected, float tol = 1e-3f) {
    QVERIFY2(qAbs(actual - expected) <= tol,
             qPrintable(QString("actual=%1 expected=%2 (tol=%3)").arg(actual).arg(expected).arg(tol)));
}

// ============ 全局通讯序列（确保“采样前进”） ============
static uint16_t gPCS = 1;
static uint16_t gBMS = 1001;

// ============ 基础参数模板 ============
static DistAreaVoltCtrlParams baseParams()
{
    DistAreaVoltCtrlParams p{};
    // 使能/运行/无故障
    p.voltageRegulationEnable = 1;
    p.pcsRtvRunStatus = 2; // 运行
    p.pcsFault = 0;
    p.bmsFault = 0;

    // 默认通讯前进
    p.pcsCommStatus = gPCS++;
    p.bmsCommStatusHeartbeat = gBMS++;

    // 目标电压
    p.aPhaseLowVoltTargetVoltage  = 215.0f;
    p.aPhaseHighVoltTargetVoltage = 225.0f;
    p.bPhaseLowVoltTargetVoltage  = 210.0f;
    p.bPhaseHighVoltTargetVoltage = 225.0f;
    p.cPhaseLowVoltTargetVoltage  = 215.0f;
    p.cPhaseHighVoltTargetVoltage = 225.0f;

    // 死区/步长
    p.voltageTargetDeadband = 2.0f;
    p.regulationStep        = 3.0f;

    // 额定 S（用于 S^2 - Q^2）
    p.pcsAPhaseRatedPower = 35.0f;
    p.pcsBPhaseRatedPower = 35.0f;
    p.pcsCPhaseRatedPower = 35.0f;

    // 无功
    p.pcsAPhaseRtvReactivePower = 0.0f;
    p.pcsBPhaseRtvReactivePower = 0.0f;
    p.pcsCPhaseRtvReactivePower = 0.0f;

    // BMS 允许充放
    p.bmsMaxChargeCurrent    = 100.0f;
    p.bmsMaxDischargeCurrent = 100.0f;

    // 电压/初始有功
    p.pcsAPhaseVoltage        = 240.0f;
    p.pcsBPhaseVoltage        = 230.0f;
    p.pcsCPhaseVoltage        = 235.0f;
    p.pcsAPhaseRtvActivePower = 10.0f;
    p.pcsBPhaseRtvActivePower = 0.0f;
    p.pcsCPhaseRtvActivePower = 0.0f;

    // 默认将 24h 策略都设为“高压优先(1)”
    for (int h = 0; h < 24; ++h)
        reinterpret_cast<uint16_t*>(&p.hour0StrategySetting)[h] = 1;

    return p;
}

// ============ 预热：让首次通讯进入“已初始化” ============
// 注意：只有在不测“FirstRun”时使用
static void warmUpOnce(DistAreaVoltCtrlParams &p)
{
    const int hour = QTime::currentTime().hour();
    setHourStrategy(p, hour, 1);
    (void)DistAreaVoltCtrl_Run(&p); // 第一次会返回 FirstRun
    // 推进通讯，清零 wrongNum
    p.pcsCommStatus = gPCS++;
    p.bmsCommStatusHeartbeat = gBMS++;
}

// ============ 每次调用前推进通讯 ============
static DistAreaVoltCtrlOutput runOnce(DistAreaVoltCtrlParams &p)
{
    p.pcsCommStatus = gPCS++;
    p.bmsCommStatusHeartbeat = gBMS++;
    return DistAreaVoltCtrl_Run(&p);
}

class TestVoltCtrl : public QObject
{
    Q_OBJECT
private slots:
    // ---------- A1 首次通讯 → FirstRun ----------
    void A1_FirstRun_Status()
    {
        DistAreaVoltCtrlParams p = baseParams();
        const int hour = QTime::currentTime().hour();
        setHourStrategy(p, hour, 1);
        auto out1 = DistAreaVoltCtrl_Run(&p);
        expectStatus(out1, AppStatusCode::FirstRun);

        // 第二次应进入正常流程（不是FirstRun就行）
        auto out2 = runOnce(p);
        QVERIFY(out2.appStatus.valid);
        QVERIFY2(out2.appStatus.value != static_cast<uint16_t>(AppStatusCode::None) &&
                     out2.appStatus.value != static_cast<uint16_t>(AppStatusCode::FirstRun),
                 "Second run should not be None/FirstRun");
    }

    // ---------- A2 通讯未前进 → CommFault；三连 SAME 触发 pcs/bmsCommFault ----------
    void A2_Comm_NoAdvance_And_RaiseAlarm()
    {
        DistAreaVoltCtrlParams p = baseParams();
        warmUpOnce(p);

        // 先一次前进，确保 wrongNum=0
        (void)runOnce(p);

        // 三次 SAME：不推进 comm，触发计数与告警
        p.pcsCommStatus = p.pcsCommStatus;
        p.bmsCommStatusHeartbeat = p.bmsCommStatusHeartbeat;
        auto o1 = DistAreaVoltCtrl_Run(&p);
        expectStatus(o1, AppStatusCode::CommFault);
        auto o2 = DistAreaVoltCtrl_Run(&p);
        expectStatus(o2, AppStatusCode::CommFault);
        auto o3 = DistAreaVoltCtrl_Run(&p);
        expectStatus(o3, AppStatusCode::CommFault);

        // 第三次后应上报 commFault（实现里 wrongNum>=3 才上报）
        QVERIFY(o3.pcsCommFault.valid);
        QCOMPARE(o3.pcsCommFault.value, (uint16_t)1);
        QVERIFY(o3.bmsCommFault.valid);
        QCOMPARE(o3.bmsCommFault.value, (uint16_t)1);
    }

    // ---------- B1 设备故障（PCS/BMS）→ EquipmentFault ----------
    void B1_PcsFault_Exit()
    {
        DistAreaVoltCtrlParams p = baseParams(); warmUpOnce(p);
        p.pcsFault = 1;
        auto out = DistAreaVoltCtrl_Run(&p);
        expectStatus(out, AppStatusCode::EquipmentFault);
        QVERIFY(!out.pcsAPhaseRemoteActivePower.valid);
    }
    void B2_BmsFault_Exit()
    {
        DistAreaVoltCtrlParams p = baseParams(); warmUpOnce(p);
        p.bmsFault = 1;
        auto out = DistAreaVoltCtrl_Run(&p);
        expectStatus(out, AppStatusCode::EquipmentFault);
        QVERIFY(!out.pcsAPhaseRemoteActivePower.valid);
    }

    // ---------- C1 未使能 → Disable ----------
    void C1_Disable_Exit()
    {
        DistAreaVoltCtrlParams p = baseParams();
        warmUpOnce(p);
        p.voltageRegulationEnable = 0;
        auto out = DistAreaVoltCtrl_Run(&p);
        expectStatus(out, AppStatusCode::Disable);
    }

    // ---------- D1 停机策略（运行态）→ 总有功/无功置 0 + RemoteSetpoint ----------
    void D1_StrategyStop_Run_ZeroTotals()
    {
        DistAreaVoltCtrlParams p = baseParams(); warmUpOnce(p);
        const int hour = QTime::currentTime().hour();
        setHourStrategy(p, hour, 0);   // 停机策略
        p.pcsRtvRunStatus = 2;         // 运行态

        auto out = DistAreaVoltCtrl_Run(&p);
        QVERIFY(out.pcsRemoteActivePower.valid);
        expectNear(out.pcsRemoteActivePower.value, 0.0f);
        QVERIFY(out.pcsRemoteReactivePower.valid);
        expectNear(out.pcsRemoteReactivePower.value, 0.0f);
        expectStatus(out, AppStatusCode::RemoteSetpoint);
    }

    // ---------- D2 停机策略（待机态）→ 下停机命令 + RemoteCommand ----------
    void D2_StrategyStop_Standby_StopCmd()
    {
        DistAreaVoltCtrlParams p = baseParams(); warmUpOnce(p);
        const int hour = QTime::currentTime().hour();
        setHourStrategy(p, hour, 0);   // 停机策略
        p.pcsRtvRunStatus = 7;         // 待机

        auto out = DistAreaVoltCtrl_Run(&p);
        QVERIFY(out.pcsRemoteStop.valid);
        QCOMPARE(out.pcsRemoteStop.value, (uint16_t)1);
        expectStatus(out, AppStatusCode::RemoteCommand);
    }

    // ---------- D3 非停机、PCS=停机 → 下开机命令 + RemoteCommand ----------
    void D3_NonStop_PcsIsStopped_StartCmd()
    {
        DistAreaVoltCtrlParams p = baseParams(); warmUpOnce(p);
        const int hour = QTime::currentTime().hour();
        setHourStrategy(p, hour, 1); // 高压优先
        p.pcsRtvRunStatus = 0;       // PCS 停机

        auto out = DistAreaVoltCtrl_Run(&p);
        QVERIFY(out.pcsRemoteStart.valid);
        QCOMPARE(out.pcsRemoteStart.value, (uint16_t)1);
        expectStatus(out, AppStatusCode::RemoteCommand);
    }

    // ---------- E1 高压优先：高于(高目标+deadband) → RemoteSetpoint & 减一步 ----------
    void E1_HighMode_Decrease()
    {
        DistAreaVoltCtrlParams p = baseParams(); warmUpOnce(p);
        const int hour = QTime::currentTime().hour(); setHourStrategy(p, hour, 1);
        p.pcsAPhaseVoltage        = 240.0f; // > 235+2
        p.pcsAPhaseRtvActivePower = 10.0f;
        p.regulationStep          = 3.0f;

        auto out = DistAreaVoltCtrl_Run(&p);
        expectStatus(out, AppStatusCode::RemoteSetpoint);
        QVERIFY(out.pcsAPhaseRemoteActivePower.valid);
        // 注意末尾有 0.1 量化/0.05 归零，这里允许少量偏差
        expectNear(out.pcsAPhaseRemoteActivePower.value, 7.0f, 0.11f);
    }

    // ---------- E2 低压优先：低于(低目标-deadband) → RemoteSetpoint & 加一步 ----------
    void E2_LowMode_Increase()
    {
        DistAreaVoltCtrlParams p = baseParams(); warmUpOnce(p);
        const int hour = QTime::currentTime().hour(); setHourStrategy(p, hour, 2);
        p.pcsAPhaseVoltage        = 210.0f; // < 215-2
        p.pcsAPhaseRtvActivePower = 1.0f;
        p.regulationStep          = 4.0f;

        auto out = DistAreaVoltCtrl_Run(&p);
        expectStatus(out, AppStatusCode::RemoteSetpoint);
        QVERIFY(out.pcsAPhaseRemoteActivePower.valid);
        expectNear(out.pcsAPhaseRemoteActivePower.value, 5.0f, 0.11f);
    }

    // ---------- E3 死区内 → Standby（不下发） ----------
    void E3_Deadband_NoAction()
    {
        DistAreaVoltCtrlParams p = baseParams();
        warmUpOnce(p);
        const int hour = QTime::currentTime().hour(); setHourStrategy(p, hour, 1);
        p.pcsAPhaseVoltage = 226.5f;
        p.pcsBPhaseVoltage = 221.5f;
        p.pcsCPhaseVoltage = 223.5f;
        // 目标电压
        p.aPhaseHighVoltTargetVoltage = 225.0f;
        p.bPhaseHighVoltTargetVoltage = 220.0f;
        p.cPhaseHighVoltTargetVoltage = 225.0f;

        auto out = DistAreaVoltCtrl_Run(&p);
        expectStatus(out, AppStatusCode::Standby);
    }

    // ---------- F1 错误传参：regulationStep<=0 → WrongParams ----------
    void F1_WrongRegulationStep()
    {
        DistAreaVoltCtrlParams p = baseParams();
        // 不预热：让 validateParams 先行
        const int hour = QTime::currentTime().hour();
        setHourStrategy(p, hour, 1);
        p.regulationStep = 0.0f; // 非法

        auto out = DistAreaVoltCtrl_Run(&p);
        expectStatus(out, AppStatusCode::WrongParams);
        QVERIFY(!out.pcsAPhaseRemoteActivePower.valid);
        QVERIFY(!out.pcsRemoteStart.valid && !out.pcsRemoteStop.valid);
    }

    // ---------- F2 错误传参：pcsAPhaseVoltage<=150 → WrongParams ----------
    void F2_WrongPcsAPhaseVoltage()
    {
        DistAreaVoltCtrlParams p = baseParams();
        p.pcsAPhaseVoltage=140;
        auto out = DistAreaVoltCtrl_Run(&p);
        expectStatus(out, AppStatusCode::WrongParams);
    }

    // ---------- G1 BMS 限制：禁放(+方向)置零 → RemoteSetpoint ----------
    void G1_BMS_DisableDischarge_ZeroOnPositive()
    {
        DistAreaVoltCtrlParams p = baseParams(); warmUpOnce(p);
        const int hour = QTime::currentTime().hour(); setHourStrategy(p, hour, 2); // 往正方向
        p.bmsMaxDischargeCurrent = 0;      // 禁放
        p.pcsAPhaseVoltage        = 210.0f;
        p.pcsAPhaseRtvActivePower = 1.0f;
        p.regulationStep          = 4.0f;

        auto out = DistAreaVoltCtrl_Run(&p);
        expectStatus(out, AppStatusCode::RemoteSetpoint);
        QVERIFY(out.pcsAPhaseRemoteActivePower.valid);
        expectNear(out.pcsAPhaseRemoteActivePower.value, 0.0f, 0.05f);
    }

    // ---------- G2 BMS 限制：禁充(-方向)置零 → RemoteSetpoint ----------
    void G2_BMS_DisableCharge_ZeroOnNegative()
    {
        DistAreaVoltCtrlParams p = baseParams(); warmUpOnce(p);
        const int hour = QTime::currentTime().hour(); setHourStrategy(p, hour, 1); // 往负方向
        p.bmsMaxChargeCurrent = 0;          // 禁充
        p.pcsAPhaseVoltage        = 242.0f;
        p.pcsAPhaseRtvActivePower = -1.0f;
        p.regulationStep          = 4.0f;

        auto out = DistAreaVoltCtrl_Run(&p);
        expectStatus(out, AppStatusCode::RemoteSetpoint);
        QVERIFY(out.pcsAPhaseRemoteActivePower.valid);
        expectNear(out.pcsAPhaseRemoteActivePower.value, 0.0f, 0.05f);
    }


     void G3_Exceeds_AvailablePower()
     {
        DistAreaVoltCtrlParams p = baseParams();
        p.pcsAPhaseRtvActivePower = -35.0f;
        p.pcsAPhaseVoltage        = 230.0f;
        p.aPhaseHighVoltTargetVoltage = 225.0f;
        p.pcsAPhaseRtvReactivePower = 0.0f;
        p.pcsAPhaseRatedPower = 35.0f;
        warmUpOnce(p);
        auto out = DistAreaVoltCtrl_Run(&p);
        expectStatus(out, AppStatusCode::RemoteSetpoint);
        expectNear(out.pcsAPhaseRemoteActivePower.value, -35.0f, 0.05f);
     }
};


QTEST_APPLESS_MAIN(TestVoltCtrl)
#include "tst_voltctrl.moc"
