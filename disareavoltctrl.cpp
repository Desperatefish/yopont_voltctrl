#include "disareavoltctrl.h"
#include "Logger.h"

// 全局静态变量：分别跟踪 Grid、PCS、BMS 的通讯状态
//static CommTracker gridCommTracker = {false, 0};  //本次项目未使用电表
static CommTracker pcsCommTracker  = {false, 0, 0};
static CommTracker bmsCommTracker  = {false, 0, 0};

static inline bool finite(float v) { return !qIsNaN(v) && qIsFinite(v); }
static inline bool finitePos(float v){ return finite(v) && v>0.f; }
static inline bool inRange(float v, float lo, float hi) { return v>=lo && v<=hi; }
// 认为合理电压范围
static constexpr float V_MIN = 150.0f;
static constexpr float V_MAX = 290.0f;

// 获取当前小时对应的策略
static StrategyMode currentHourStrategy(const DistAreaVoltCtrlParams* p, int hour)
{
    switch (hour)
    {
    case 0:  return (StrategyMode)p->hour0StrategySetting;
    case 1:  return (StrategyMode)p->hour1StrategySetting;
    case 2:  return (StrategyMode)p->hour2StrategySetting;
    case 3:  return (StrategyMode)p->hour3StrategySetting;
    case 4:  return (StrategyMode)p->hour4StrategySetting;
    case 5:  return (StrategyMode)p->hour5StrategySetting;
    case 6:  return (StrategyMode)p->hour6StrategySetting;
    case 7:  return (StrategyMode)p->hour7StrategySetting;
    case 8:  return (StrategyMode)p->hour8StrategySetting;
    case 9:  return (StrategyMode)p->hour9StrategySetting;
    case 10: return (StrategyMode)p->hour10StrategySetting;
    case 11: return (StrategyMode)p->hour11StrategySetting;
    case 12: return (StrategyMode)p->hour12StrategySetting;
    case 13: return (StrategyMode)p->hour13StrategySetting;
    case 14: return (StrategyMode)p->hour14StrategySetting;
    case 15: return (StrategyMode)p->hour15StrategySetting;
    case 16: return (StrategyMode)p->hour16StrategySetting;
    case 17: return (StrategyMode)p->hour17StrategySetting;
    case 18: return (StrategyMode)p->hour18StrategySetting;
    case 19: return (StrategyMode)p->hour19StrategySetting;
    case 20: return (StrategyMode)p->hour20StrategySetting;
    case 21: return (StrategyMode)p->hour21StrategySetting;
    case 22: return (StrategyMode)p->hour22StrategySetting;
    case 23: return (StrategyMode)p->hour23StrategySetting;
    default: return StrategyMode::StrategyStop;
    }
}

// 参数校验
static ValidateResult validateParams(const DistAreaVoltCtrlParams* p) {
    ValidateResult r;
    if (!p) { r.ok=false; r.issues<<"params==nullptr"; return r; }

    // 小工具：统一记录错误
    auto addIssue = [&](const QString& msg){ r.ok=false; r.issues<<msg; };

    // 批量“有限数”检查（非 NaN/Inf）
    auto checkFiniteBatch = [&](std::initializer_list<std::pair<const char*, float>> xs){
        for (const auto& kv : xs) {
            const char* name = kv.first;
            float v = kv.second;
            if (!finite(v)) addIssue(QString("%1 is NaN/Inf").arg(name));
        }
    };

    checkFiniteBatch({
        {"pcsA_V", p->pcsAPhaseVoltage}, {"pcsB_V", p->pcsBPhaseVoltage}, {"pcsC_V", p->pcsCPhaseVoltage},
        {"Qa", p->pcsAPhaseRtvReactivePower}, {"Qb", p->pcsBPhaseRtvReactivePower}, {"Qc", p->pcsCPhaseRtvReactivePower},
        {"P3", p->pcsThreePhaseRtvActivePower}, {"Q3", p->pcsThreePhaseRtvReactivePower}, {"S3", p->pcsThreePhaseRtvApparentPower},
        {"regStep", p->regulationStep}, {"deadband", p->voltageTargetDeadband},
        {"I_chg_max", p->bmsMaxChargeCurrent}, {"I_dschg_max", p->bmsMaxDischargeCurrent},
        {"A_low_target", p->aPhaseLowVoltTargetVoltage}, {"A_high_target", p->aPhaseHighVoltTargetVoltage},
        {"B_low_target", p->bPhaseLowVoltTargetVoltage}, {"B_high_target", p->bPhaseHighVoltTargetVoltage},
        {"C_low_target", p->cPhaseLowVoltTargetVoltage}, {"C_high_target", p->cPhaseHighVoltTargetVoltage}
    });

    if (p->regulationStep <= 0)       addIssue("regulationStep<=0");
    if (p->voltageTargetDeadband < 0) addIssue("deadband<0");
    if (p->bmsMaxChargeCurrent  < 0 || p->bmsMaxDischargeCurrent < 0)
        addIssue("BMS currents <0");
    if (p->bmsSoc<0||p->bmsSoc>100)
        addIssue("BMS SOC wrong");
    if (p->voltageRegulationEnable != 0 && p->voltageRegulationEnable != 1)
        addIssue("voltageRegulationEnable not 0/1");

    // 电压范围判断
    if(!inRange(p->pcsAPhaseVoltage, V_MIN, V_MAX)) addIssue("A相实时电压越界");
    if(!inRange(p->aPhaseLowVoltTargetVoltage, V_MIN, V_MAX)) addIssue("A相低电压目标越界");
    if(!inRange(p->aPhaseHighVoltTargetVoltage, V_MIN, V_MAX)) addIssue("A相高电压目标越界");
    if(!inRange(p->pcsBPhaseVoltage, V_MIN, V_MAX)) addIssue("B相实时电压越界");
    if(!inRange(p->bPhaseLowVoltTargetVoltage, V_MIN, V_MAX)) addIssue("B相低电压目标越界");
    if(!inRange(p->bPhaseHighVoltTargetVoltage, V_MIN, V_MAX)) addIssue("B相高电压目标越界");
    if(!inRange(p->pcsCPhaseVoltage, V_MIN, V_MAX)) addIssue("C相实时电压越界");
    if(!inRange(p->cPhaseLowVoltTargetVoltage, V_MIN, V_MAX)) addIssue("C相低电压目标越界");
    if(!inRange(p->cPhaseHighVoltTargetVoltage, V_MIN, V_MAX)) addIssue("C相高电压目标越界");

    // 策略枚举合法（当前小时）
    const int h = QTime::currentTime().hour();
    const StrategyMode sm = currentHourStrategy(p, h);
    if (sm!=StrategyMode::StrategyStop && sm!=StrategyMode::StrategyHigh && sm!=StrategyMode::StrategyLow)
        addIssue("invalid StrategyMode");

    // SOC 上下限
    if (finite(p->bmsSocUpperLimit) && finite(p->bmsSocLowerLimit)) {
        if (p->bmsSocLowerLimit < 0 || p->bmsSocUpperLimit > 100 || p->bmsSocLowerLimit > p->bmsSocUpperLimit)
            addIssue("SOC limits invalid");
    }

    return r;
}

// 电压目标选择
static float selectTargetVoltage(StrategyMode currentStrategy, float lowTargetVoltage, float highTargetVoltage)
{
    if (currentStrategy == StrategyMode::StrategyHigh) {
        return highTargetVoltage;
    } else if (currentStrategy == StrategyMode::StrategyLow) {
        return lowTargetVoltage;
    }
    Logger::instance().log(Logger::Warn, "时段设置错误");
    return 0;
}

// 正放负充：步进调节有功功率
static void stepAdjustActivePower(float currentVolt, float targetVoltage, float Deadband, float regulationStep, float currentActivePower, FloatWithValid* outputP,char phase)
{
    const float diff = currentVolt - targetVoltage;
    if(qFabs(diff) > Deadband) {
        if(diff > 0) {
            outputP->value = currentActivePower - regulationStep;
            outputP->valid = true;
            Logger::instance().log(Logger::Info, QString("%1相高电压,步进减少有功功率 %2").arg(QChar(phase)).arg(regulationStep));
        } else {
            outputP->value = currentActivePower + regulationStep;
            outputP->valid = true;
            Logger::instance().log(Logger::Info, QString("%1相低电压，步进增加有功功率 %2").arg(QChar(phase)).arg(regulationStep));
        }
    }
}

// 校验有功功率是否超限
static void validActivePower(float Q, float S, FloatWithValid* outputP, float maxChargeI, float maxDischargeI, char phase)
{
    if (!outputP->valid) {
        Logger::instance().log(Logger::Info, QString("%1相功率不下发，不处理").arg(QChar(phase)));
        return;
    }

    float rad = S * S - Q * Q;
    float pMax = rad > 0 ? qSqrt(rad) : 0.0f;

    if(outputP->value > pMax) {
        outputP->value = pMax;
        Logger::instance().log(Logger::Info, QString("超过额定功率，%1相有功功率修正为：%2").arg(QChar(phase)).arg(outputP->value));
    }
    if (outputP->value > 0 && maxDischargeI == 0) {
        outputP->value = 0;
        Logger::instance().log(Logger::Info, QString("BMS禁放，%1相有功功率修正为：%2").arg(QChar(phase)).arg(outputP->value));
    }
    if(outputP->value < -pMax) {
        outputP->value = -pMax;
        Logger::instance().log(Logger::Info, QString("超过额定功率，%1相有功功率修正为：%2").arg(QChar(phase)).arg(outputP->value));
    }
    if (outputP->value < 0 && maxChargeI  == 0) {
        outputP->value = 0;
        Logger::instance().log(Logger::Info, QString("BMS禁充，%1相有功功率修正为：%2").arg(QChar(phase)).arg(outputP->value));
    }
    // 可选：量化
    const float quantum = 0.1f;
    const float snap0   = 0.05f;
    outputP->value = qRound(outputP->value/quantum)*quantum;
    if (qFabs(outputP->value) < snap0) outputP->value = 0.f;
    Logger::instance().log(Logger::Info, QString("%1相有功功率最终设定值：%2").arg(QChar(phase)).arg(outputP->value));
}

// 主函数：电压治理策略运行
DistAreaVoltCtrlOutput DistAreaVoltCtrl_Run(const DistAreaVoltCtrlParams *params)
{
    //    // 设置日志输出格式
    //    qSetMessagePattern("[%{time yyyy-MM-dd hh:mm:ss.zzz}] "
    //                       "%{type} "
    //                       "(%{file}:%{line}, %{function}) - %{message}");

    // 初始化，所有字段默认为0/false
    DistAreaVoltCtrlOutput output{};

    // ---------- 参数合法性校验 ----------
    const auto v = validateParams(params);
    if (!v.ok) {
        Logger::instance().log(Logger::Warn, QString("参数校验失败: %1").arg(v.issues.join("; ")));
        output.appStatus.value = static_cast<uint16_t>(AppStatusCode::WrongParams);
        output.appStatus.valid=1;
        return output;  // 强约束失败：直接停控
    }

    // ---------------- 设备通讯监测 ----------------
    if(!pcsCommTracker.hasLast || !bmsCommTracker.hasLast)
    {
        pcsCommTracker.hasLast = true;
        bmsCommTracker.hasLast = true;
        pcsCommTracker.lastStatus = params->pcsCommStatus;
        bmsCommTracker.lastStatus = params->bmsCommStatusHeartbeat;
        Logger::instance().log(Logger::Warn, "设备首次通讯，不进行控制");
        output.appStatus.value = static_cast<uint16_t>(AppStatusCode::FirstRun);
        output.appStatus.valid=1;
        return output;
    }
    else
    {
        if(pcsCommTracker.lastStatus == params->pcsCommStatus||bmsCommTracker.lastStatus == params->bmsCommStatusHeartbeat)
        {
            output.appStatus.value = static_cast<uint16_t>(AppStatusCode::CommFault);
            output.appStatus.valid=1;
            if(pcsCommTracker.lastStatus == params->pcsCommStatus)
            {
                pcsCommTracker.wrongNum++;
                //  防溢出
                if(pcsCommTracker.wrongNum>1000)
                    pcsCommTracker.wrongNum=1000;
                Logger::instance().log(Logger::Warn, "PCS本周期采样未前进，跳过控制");
                if(pcsCommTracker.wrongNum >= 3)
                {
                    output.pcsCommFault.value = 1;
                    output.pcsCommFault.valid = 1;
                    Logger::instance().log(Logger::Warn, "PCS多次通信未更新，上报云平台");
                }
            }
            if(bmsCommTracker.lastStatus == params->bmsCommStatusHeartbeat)
            {
                bmsCommTracker.wrongNum++;
                //  防溢出
                if(bmsCommTracker.wrongNum>1000)
                    bmsCommTracker.wrongNum=1000;
                Logger::instance().log(Logger::Warn, "BMS本周期采样未前进，跳过控制");
                if(bmsCommTracker.wrongNum >= 3)
                {
                    output.bmsCommFault.value = 1;
                    output.bmsCommFault.valid = 1;
                    Logger::instance().log(Logger::Warn, "BMS多次通信未更新，上报云平台");
                }
            }
            return output;
        }
        pcsCommTracker.lastStatus = params->pcsCommStatus;
        bmsCommTracker.lastStatus = params->bmsCommStatusHeartbeat;
        pcsCommTracker.wrongNum = 0;
        bmsCommTracker.wrongNum = 0;
        output.pcsCommFault.value = 0;
        output.pcsCommFault.valid = 1;
        output.bmsCommFault.value = 0;
        output.bmsCommFault.valid = 1;
        Logger::instance().log(Logger::Info, "设备通讯正常");
    }

    // ---------------- 设备故障监测 ----------------
    if(params->pcsFault)
    {
        Logger::instance().log(Logger::Warn, "PCS故障，停止控制");
        output.appStatus.value = static_cast<uint16_t>(AppStatusCode::EquipmentFault);
        output.appStatus.valid=1;
        return output;
    }

    if(params->bmsFault)
    {
        Logger::instance().log(Logger::Warn, "BMS故障，停止控制");
        output.appStatus.value = static_cast<uint16_t>(AppStatusCode::EquipmentFault);
        output.appStatus.valid=1;
        return output;
    }

    // ---------------- 治理使能 ------------------
    if(params->voltageRegulationEnable == 0)
    {
        Logger::instance().log(Logger::Warn, "治理使能关闭，直接退出治理");
        output.appStatus.value = static_cast<uint16_t>(AppStatusCode::Disable);
        output.appStatus.valid=1;
        return output;
    }

    // ---------------- 停止时段 ------------------
    QTime now = QTime::currentTime();
    int currentHour = now.hour();   // 当前小时 [0..23]
    const StrategyMode currentStrategySetting = currentHourStrategy(params, currentHour);

    if (currentStrategySetting == StrategyMode::StrategyStop)
    {
        // 在停机区间
        if(params->pcsRtvRunStatus == 2) //处于运行状态，下零功率
        {
            output.pcsRemoteActivePower.value = 0;
            output.pcsRemoteActivePower.valid = true;
            output.pcsRemoteReactivePower.value = 0;
            output.pcsRemoteReactivePower.valid = true;
            output.appStatus.value = static_cast<uint16_t>(AppStatusCode::RemoteSetpoint);
            output.appStatus.valid=true;
            Logger::instance().log(Logger::Info, "停机区间，运行状态，下零功率");
        }

        if(params->pcsRtvRunStatus == 7) //处于待机状态，下停机命令
        {
            output.pcsRemoteStop.value = 1;
            output.pcsRemoteStop.valid = true;
            output.appStatus.value = static_cast<uint16_t>(AppStatusCode::RemoteCommand);
            output.appStatus.valid=true;
            Logger::instance().log(Logger::Info, "停机区间，待机状态，下停机命令");
        }

        return output;
    }
    else
    {
        // 非停机区间
        if(params->pcsRtvRunStatus == 0 && !params->pcsFault && !params->bmsFault) //处于停机状态,下开机命令
        {
            output.pcsRemoteStart.value = 1;
            output.pcsRemoteStart.valid = true;
            output.appStatus.value = static_cast<uint16_t>(AppStatusCode::RemoteCommand);
            output.appStatus.valid=1;
            Logger::instance().log(Logger::Info, "非停机区间，停机状态，下开机命令");
            return output;
        }
    }

    // ---------------- 目标电压计算 ----------------
    const float aTargetVoltage = selectTargetVoltage(currentStrategySetting, params->aPhaseLowVoltTargetVoltage, params->aPhaseHighVoltTargetVoltage);
    const float bTargetVoltage = selectTargetVoltage(currentStrategySetting, params->bPhaseLowVoltTargetVoltage, params->bPhaseHighVoltTargetVoltage);
    const float cTargetVoltage = selectTargetVoltage(currentStrategySetting, params->cPhaseLowVoltTargetVoltage, params->cPhaseHighVoltTargetVoltage);

    if(finitePos(aTargetVoltage))
        stepAdjustActivePower(params->pcsAPhaseVoltage, aTargetVoltage, params->voltageTargetDeadband, params->regulationStep, params->pcsAPhaseRtvActivePower, &output.pcsAPhaseRemoteActivePower,'A');

    if(finitePos(bTargetVoltage))
        stepAdjustActivePower(params->pcsBPhaseVoltage, bTargetVoltage, params->voltageTargetDeadband, params->regulationStep, params->pcsBPhaseRtvActivePower, &output.pcsBPhaseRemoteActivePower,'B');

    if(finitePos(cTargetVoltage))
        stepAdjustActivePower(params->pcsCPhaseVoltage, cTargetVoltage, params->voltageTargetDeadband, params->regulationStep, params->pcsCPhaseRtvActivePower, &output.pcsCPhaseRemoteActivePower,'C');

    // ---------------- 功率限幅校验 ----------------
    validActivePower(params->pcsAPhaseRtvReactivePower, params->pcsAPhaseRatedPower, &output.pcsAPhaseRemoteActivePower, params->bmsMaxChargeCurrent, params->bmsMaxDischargeCurrent, 'A');
    validActivePower(params->pcsBPhaseRtvReactivePower, params->pcsBPhaseRatedPower, &output.pcsBPhaseRemoteActivePower, params->bmsMaxChargeCurrent, params->bmsMaxDischargeCurrent, 'B');
    validActivePower(params->pcsCPhaseRtvReactivePower, params->pcsCPhaseRatedPower, &output.pcsCPhaseRemoteActivePower, params->bmsMaxChargeCurrent, params->bmsMaxDischargeCurrent, 'C');

    if(output.pcsAPhaseRemoteActivePower.valid||output.pcsBPhaseRemoteActivePower.valid||output.pcsCPhaseRemoteActivePower.valid||output.pcsRemoteActivePower.valid)
    {
        output.appStatus.value = static_cast<uint16_t>(AppStatusCode::RemoteSetpoint);
        output.appStatus.valid=true;
    }
    else {
        output.appStatus.value = static_cast<uint16_t>(AppStatusCode::Standby);
        output.appStatus.valid=true;
    }
    return output;
}
