#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <QDebug>
#include <QTime>
#include <QtMath>
#include <QStringList>
#include <Logger.h>

// ================= 台区治理策略参数结构体 =================
typedef struct {
    // ---------- 采集量 - 并网点 ----------
    float    gridAPhaseRtvActivePower;        // A相实时有功功率 (kW)
    float    gridBPhaseRtvActivePower;        // B相实时有功功率 (kW)
    float    gridCPhaseRtvActivePower;        // C相实时有功功率 (kW)
    float    gridThreePhaseRtvActivePower;    // 三相实时有功功率 (kW)
    float    gridAPhaseRtvApparentPower;      // A相实时视在功率 (kVA)
    float    gridBPhaseRtvApparentPower;      // B相实时视在功率 (kVA)
    float    gridCPhaseRtvApparentPower;      // C相实时视在功率 (kVA)
    float    gridThreePhaseRtvApparentPower;  // 三相实时视在功率 (kVA)
    float    gridAPhaseVoltage;               // A相电压 (V)
    float    gridBPhaseVoltage;               // B相电压 (V)
    float    gridCPhaseVoltage;               // C相电压 (V)
    uint16_t gridCT;                          // CT比
    uint16_t gridPT;                          // PT比
    uint16_t gridFault;                       // 故障
    uint16_t gridCommStatus;                  // 通讯状态（秒）

    // ---------- 采集量 - PCS ----------
    uint16_t pcsRtvRunStatus;                 // PCS实时运行状态 (0=停机,2=运行,7=待机)
    float    pcsAPhaseVoltage;                // A相电压 (V)
    float    pcsBPhaseVoltage;                // B相电压 (V)
    float    pcsCPhaseVoltage;                // C相电压 (V)
    float    pcsAPhaseRtvActivePower;         // A相实时有功功率 (kW)
    float    pcsBPhaseRtvActivePower;         // B相实时有功功率 (kW)
    float    pcsCPhaseRtvActivePower;         // C相实时有功功率 (kW)
    float    pcsThreePhaseRtvActivePower;     // 三相实时有功功率 (kW)
    float    pcsAPhaseRtvReactivePower;       // A相实时无功功率 (kVar)
    float    pcsBPhaseRtvReactivePower;       // B相实时无功功率 (kVar)
    float    pcsCPhaseRtvReactivePower;       // C相实时无功功率 (kVar)
    float    pcsThreePhaseRtvReactivePower;   // 三相实时无功功率 (kVar)
    float    pcsAPhaseRtvApparentPower;       // A相实时视在功率 (kVA)
    float    pcsBPhaseRtvApparentPower;       // B相实时视在功率 (kVA)
    float    pcsCPhaseRtvApparentPower;       // C相实时视在功率 (kVA)
    float    pcsThreePhaseRtvApparentPower;   // 三相实时视在功率 (kVA)
    uint16_t pcsFault;                        // 故障
    uint16_t pcsCommStatus;                   // 通讯状态（秒）

    // ---------- 采集量 - BMS ----------
    float    bmsMaxChargeCurrent;             // 最大可充电电流 (A)
    float    bmsMaxDischargeCurrent;          // 最大可放电电流 (A)
    float    bmsSoc;                          // SOC (%)
    uint16_t bmsFault;                        // 故障
    uint16_t bmsCommStatusHeartbeat;          // 通讯状态（心跳）

    // ---------- 配置量 - 并网点 ----------
    uint16_t gridRatedApparentPower;          // 额定视在功率 (kVA)

    // ---------- 配置量 - PCS ----------
    uint16_t pcsRatedPower;                   // 额定功率 (kW)
    uint16_t pcsAPhaseRatedPower;             // A相额定功率 (kVA)
    uint16_t pcsBPhaseRatedPower;             // B相额定功率 (kVA)
    uint16_t pcsCPhaseRatedPower;             // C相额定功率 (kVA)

    // ---------- 配置量 - BMS ----------
    float    bmsSocUpperLimit;                // SOC上限 (%)
    float    bmsSocLowerLimit;                // SOC下限 (%)

    // ---------- 策略配置 ----------
    uint16_t voltageRegulationEnable;         // 电压治理使能 (0/1)
    float    voltageTargetDeadband;           // 电压目标值死区 (V)
    uint16_t regulationPeriod;                // 调节周期(s)
    float    regulationStep;                  // 调节步长 (kW)

    // 24 小时策略设置
    uint16_t hour0StrategySetting;            // 0-1时策略(0=停机,1=高电压优先,2=低电压优先)
    uint16_t hour1StrategySetting;            // 1-2时策略
    uint16_t hour2StrategySetting;            // 2-3时策略
    uint16_t hour3StrategySetting;            // 3-4时策略
    uint16_t hour4StrategySetting;            // 4-5时策略
    uint16_t hour5StrategySetting;            // 5-6时策略
    uint16_t hour6StrategySetting;            // 6-7时策略
    uint16_t hour7StrategySetting;            // 7-8时策略
    uint16_t hour8StrategySetting;            // 8-9时策略
    uint16_t hour9StrategySetting;            // 9-10时策略
    uint16_t hour10StrategySetting;           // 10-11时策略
    uint16_t hour11StrategySetting;           // 11-12时策略
    uint16_t hour12StrategySetting;           // 12-13时策略
    uint16_t hour13StrategySetting;           // 13-14时策略
    uint16_t hour14StrategySetting;           // 14-15时策略
    uint16_t hour15StrategySetting;           // 15-16时策略
    uint16_t hour16StrategySetting;           // 16-17时策略
    uint16_t hour17StrategySetting;           // 17-18时策略
    uint16_t hour18StrategySetting;           // 18-19时策略
    uint16_t hour19StrategySetting;           // 19-20时策略
    uint16_t hour20StrategySetting;           // 20-21时策略
    uint16_t hour21StrategySetting;           // 21-22时策略
    uint16_t hour22StrategySetting;           // 22-23时策略
    uint16_t hour23StrategySetting;           // 23-24时策略

    // ---------- 电压目标 ----------
    float    aPhaseLowVoltTargetVoltage;      // A相低电压目标 (V)
    float    aPhaseHighVoltTargetVoltage;     // A相高电压目标 (V)
    float    bPhaseLowVoltTargetVoltage;      // B相低电压目标 (V)
    float    bPhaseHighVoltTargetVoltage;     // B相高电压目标 (V)
    float    cPhaseLowVoltTargetVoltage;      // C相低电压目标 (V)
    float    cPhaseHighVoltTargetVoltage;     // C相高电压目标 (V)

} DistAreaVoltCtrlParams;

struct ValidateResult {
    bool ok{true};
    QStringList issues;           // 详细问题，便于上报/排障
};

enum class StrategyMode : uint16_t {
    StrategyStop = 0,   // 停机
    StrategyHigh = 1,   // 高电压优先
    StrategyLow  = 2    // 低电压优先
};

// ================= 通讯追踪结构体 =================
typedef struct {
    bool     hasLast;       // 是否已有上次值
    uint16_t lastStatus;    // 上次通讯状态（秒/心跳）
    uint16_t wrongNum;   // 错误次数
} CommTracker;


// ================= 有效值封装结构体 =================
typedef struct {
    uint16_t value;   // 遥控值
    bool     valid;   // 是否有效 (0/1)
} UInt16WithValid;

typedef struct {
    float    value;   // 遥调值
    bool     valid;   // 是否有效 (0/1)
} FloatWithValid;


// ================= 策略输出结构体 =================
typedef struct {
    UInt16WithValid  pcsRemoteStart;              // 遥控启动
    UInt16WithValid  pcsRemoteStop;               // 遥控停止
    UInt16WithValid  peakValleyControlEnable;     // 削峰填谷使能


    FloatWithValid   pcsRemoteActivePower;        // 遥调有功 (总)
    FloatWithValid   pcsAPhaseRemoteActivePower;  // A相遥调有功
    FloatWithValid   pcsBPhaseRemoteActivePower;  // B相遥调有功
    FloatWithValid   pcsCPhaseRemoteActivePower;  // C相遥调有功

    FloatWithValid   pcsRemoteReactivePower;      // 遥调无功 (总)
    FloatWithValid   pcsAPhaseRemoteReactivePower;// A相遥调无功
    FloatWithValid   pcsBPhaseRemoteReactivePower;// B相遥调无功
    FloatWithValid   pcsCPhaseRemoteReactivePower;// C相遥调无功

    UInt16WithValid  gridCommFault;               // 并网点通信异常报警
    UInt16WithValid  pcsCommFault;                // PCS通信异常报警
    UInt16WithValid  bmsCommFault;                // BMS通信异常报警
    UInt16WithValid  appStatus;                   // 程序状态

} DistAreaVoltCtrlOutput;

enum class AppStatusCode : uint16_t {
    None            = 0,  // 默认值，程序未运行
    Standby         = 1,  // 使能，但无遥控/遥调（正常待机）
    RemoteCommand   = 2,  // 下发了遥控命令（启停）
    RemoteSetpoint  = 3,  // 下发了遥调（功率设定）
    WrongParams     = 4,  // 错误传参
    FirstRun        = 5,  // 首次运行
    Disable          = 6,  // 未使能
    CommFault       = 7,  // 通讯异常
    EquipmentFault  = 8   // 设备故障
};


// ================= 策略函数声明 =================
DistAreaVoltCtrlOutput DistAreaVoltCtrl_Run(const DistAreaVoltCtrlParams *params);
