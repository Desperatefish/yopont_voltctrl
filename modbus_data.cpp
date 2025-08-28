#include "modbus_data.h"
#include "Logger.h"
#include "disareavoltctrl.h"
#include <QModbusClient>
#include <QModbusDataUnit>
#include <QPointer>
#include <QDateTime>
#include <cmath>

//采集量与配置量A系数
float gConfFactor_A[100] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,1, 1, 1, 1, 1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1, 1, 0.1, 0.1, 0.1, 1, 1,
                            1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1, 0.1, 1, 0.1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,0.1,0.1,0.1,0.1,0.1,0.1};
//采集量与配置量B系数
float gConfFactor_B[100] = {0};

//输出量A系数
float gOutFactor_A[20] = {1, 1, 1, 10, 10, 10, 10, 10, 10, 10, 10, 1, 1, 1, 1};

//输出量B系数
float gOutFactor_B[20] = {0};

DistAreaVoltCtrlParams params{};

DistAreaVoltCtrlOutput ctrloutput{};

// 结构体字段偏移和类型定义
constexpr size_t fieldOffsets[] =
{
    offsetof(DistAreaVoltCtrlParams, gridAPhaseRtvActivePower),
    offsetof(DistAreaVoltCtrlParams, gridBPhaseRtvActivePower),
    offsetof(DistAreaVoltCtrlParams, gridCPhaseRtvActivePower),
    offsetof(DistAreaVoltCtrlParams, gridThreePhaseRtvActivePower),
    offsetof(DistAreaVoltCtrlParams, gridAPhaseRtvApparentPower),
    offsetof(DistAreaVoltCtrlParams, gridBPhaseRtvApparentPower),
    offsetof(DistAreaVoltCtrlParams, gridCPhaseRtvApparentPower),
    offsetof(DistAreaVoltCtrlParams, gridThreePhaseRtvApparentPower),
    offsetof(DistAreaVoltCtrlParams, gridAPhaseVoltage),
    offsetof(DistAreaVoltCtrlParams, gridBPhaseVoltage),
    offsetof(DistAreaVoltCtrlParams, gridCPhaseVoltage),
    offsetof(DistAreaVoltCtrlParams, gridCT),
    offsetof(DistAreaVoltCtrlParams, gridPT),
    offsetof(DistAreaVoltCtrlParams, gridFault),
    offsetof(DistAreaVoltCtrlParams, gridCommStatus),
    offsetof(DistAreaVoltCtrlParams, pcsRtvRunStatus),
    offsetof(DistAreaVoltCtrlParams, pcsAPhaseVoltage),
    offsetof(DistAreaVoltCtrlParams, pcsBPhaseVoltage),
    offsetof(DistAreaVoltCtrlParams, pcsCPhaseVoltage),
    offsetof(DistAreaVoltCtrlParams, pcsAPhaseRtvActivePower),
    offsetof(DistAreaVoltCtrlParams, pcsBPhaseRtvActivePower),
    offsetof(DistAreaVoltCtrlParams, pcsCPhaseRtvActivePower),
    offsetof(DistAreaVoltCtrlParams, pcsThreePhaseRtvActivePower),
    offsetof(DistAreaVoltCtrlParams, pcsAPhaseRtvReactivePower),
    offsetof(DistAreaVoltCtrlParams, pcsBPhaseRtvReactivePower),
    offsetof(DistAreaVoltCtrlParams, pcsCPhaseRtvReactivePower),
    offsetof(DistAreaVoltCtrlParams, pcsThreePhaseRtvReactivePower),
    offsetof(DistAreaVoltCtrlParams, pcsAPhaseRtvApparentPower),
    offsetof(DistAreaVoltCtrlParams, pcsBPhaseRtvApparentPower),
    offsetof(DistAreaVoltCtrlParams, pcsCPhaseRtvApparentPower),
    offsetof(DistAreaVoltCtrlParams, pcsThreePhaseRtvApparentPower),
    offsetof(DistAreaVoltCtrlParams, pcsFault),
    offsetof(DistAreaVoltCtrlParams, pcsCommStatus),
    offsetof(DistAreaVoltCtrlParams, bmsMaxChargeCurrent),
    offsetof(DistAreaVoltCtrlParams, bmsMaxDischargeCurrent),
    offsetof(DistAreaVoltCtrlParams, bmsSoc),
    offsetof(DistAreaVoltCtrlParams, bmsFault),
    offsetof(DistAreaVoltCtrlParams, bmsCommStatusHeartbeat),
    offsetof(DistAreaVoltCtrlParams, gridRatedApparentPower),
    offsetof(DistAreaVoltCtrlParams, pcsRatedPower),
    offsetof(DistAreaVoltCtrlParams, pcsAPhaseRatedPower),
    offsetof(DistAreaVoltCtrlParams, pcsBPhaseRatedPower),
    offsetof(DistAreaVoltCtrlParams, pcsCPhaseRatedPower),
    offsetof(DistAreaVoltCtrlParams, bmsSocUpperLimit),
    offsetof(DistAreaVoltCtrlParams, bmsSocLowerLimit),
    offsetof(DistAreaVoltCtrlParams, voltageRegulationEnable),
    offsetof(DistAreaVoltCtrlParams, voltageTargetDeadband),
    offsetof(DistAreaVoltCtrlParams, regulationPeriod),
    offsetof(DistAreaVoltCtrlParams, regulationStep),
    offsetof(DistAreaVoltCtrlParams, hour0StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour1StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour2StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour3StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour4StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour5StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour6StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour7StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour8StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour9StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour10StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour11StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour12StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour13StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour14StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour15StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour16StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour17StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour18StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour19StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour20StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour21StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour22StrategySetting),
    offsetof(DistAreaVoltCtrlParams, hour23StrategySetting),
    offsetof(DistAreaVoltCtrlParams, aPhaseLowVoltTargetVoltage),
    offsetof(DistAreaVoltCtrlParams, aPhaseHighVoltTargetVoltage),
    offsetof(DistAreaVoltCtrlParams, bPhaseLowVoltTargetVoltage),
    offsetof(DistAreaVoltCtrlParams, bPhaseHighVoltTargetVoltage),
    offsetof(DistAreaVoltCtrlParams, cPhaseLowVoltTargetVoltage),
    offsetof(DistAreaVoltCtrlParams, cPhaseHighVoltTargetVoltage)
};
//数据类型是否为float
constexpr bool isFieldFloat[] =
{
    true, true, true, true, true, true, true, true, true, true,
    true, false, false, false, false, false, true, true, true, true,
    true, true, true, true, true, true, true, true, true, true,
    true, false, false, true, true, true, false, false, false,
    false, false, false, false, true, true, false, true, false, true,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, true, true, true, true, true,true
};

struct Field
{
    int regAddr;       // modbus 寄存器地址
    int idx;           // 对应系数数组下标
    void* valuePtr;    // 指向 value
    bool* validPtr;    // 指向 valid
    bool isFloat;      // 是否 float 类型
};

void ReadHandler::updateReadTimerInterval(int newMs)
{
    readTimer.setInterval(newMs);
    Logger::instance().log(Logger::Info,
        QString("寄存器 readTimer 周期已更新为 %1 ").arg(newMs));
}

//向寄存器中写入相关数据
void writeDistAreaVoltCtrlOutput(QModbusClient* modbus, const DistAreaVoltCtrlOutput& output)
{
    Field fields[] =
    {
        {101, 0,  (void*)&output.pcsRemoteStart.value,              (bool*)&output.pcsRemoteStart.valid, false},
        {102, 1,  (void*)&output.pcsRemoteStop.value,               (bool*)&output.pcsRemoteStop.valid, false},
        {103, 2,  (void*)&output.peakValleyControlEnable.value,     (bool*)&output.peakValleyControlEnable.valid, false},
        {104, 3,  (void*)&output.pcsRemoteActivePower.value,        (bool*)&output.pcsRemoteActivePower.valid, true},
        {105, 4,  (void*)&output.pcsAPhaseRemoteActivePower.value,  (bool*)&output.pcsAPhaseRemoteActivePower.valid, true},
        {106, 5,  (void*)&output.pcsBPhaseRemoteActivePower.value,  (bool*)&output.pcsBPhaseRemoteActivePower.valid, true},
        {107, 6,  (void*)&output.pcsCPhaseRemoteActivePower.value,  (bool*)&output.pcsCPhaseRemoteActivePower.valid, true},
        {108, 7,  (void*)&output.pcsRemoteReactivePower.value,      (bool*)&output.pcsRemoteReactivePower.valid, true},
        {109, 8,  (void*)&output.pcsAPhaseRemoteReactivePower.value,(bool*)&output.pcsAPhaseRemoteReactivePower.valid, true},
        {110, 9,  (void*)&output.pcsBPhaseRemoteReactivePower.value,(bool*)&output.pcsBPhaseRemoteReactivePower.valid, true},
        {111,10,  (void*)&output.pcsCPhaseRemoteReactivePower.value,(bool*)&output.pcsCPhaseRemoteReactivePower.valid, true},
        {112,11,  (void*)&output.gridCommFault.value,               (bool*)&output.gridCommFault.valid, false},
        {113,12,  (void*)&output.pcsCommFault.value,                (bool*)&output.pcsCommFault.valid, false},
        {114,13,  (void*)&output.bmsCommFault.value,                (bool*)&output.bmsCommFault.valid, false},
        {115,14,  (void*)&output.appStatus.value,                   (bool*)&output.appStatus.valid, false},
    };

    for (const Field& field : fields)
    {
        if (!*(field.validPtr)) continue;  // valid == false 不写

        int16_t modbusValue;
        double calcVal;
        if (field.isFloat)
        {
            float fVal = *(float*)field.valuePtr;
            calcVal = static_cast<int16_t>(fVal * gOutFactor_A[field.idx] + gOutFactor_B[field.idx]);
        }
        else
        {
            uint16_t uVal = *(uint16_t*)field.valuePtr;
            calcVal = static_cast<int16_t>(uVal * gOutFactor_A[field.idx] + gOutFactor_B[field.idx]);
        }

        modbusValue = static_cast<int16_t>(std::round(calcVal)); //使用四舍五入避免精度丢失

        QModbusDataUnit writeUnit(QModbusDataUnit::HoldingRegisters, field.regAddr, 1);
        writeUnit.setValue(0, static_cast<quint16>(modbusValue));

        if (auto *reply = modbus->sendWriteRequest(writeUnit, 1))  // 1 = slave ID
        {
            QObject::connect(reply, &QModbusReply::finished, [reply, field, modbusValue]() {
                if (reply->error() != QModbusDevice::NoError)
                {
                    Logger::instance().log(Logger::Warn,
                        QString("寄存器 %1 写入失败: %2").arg(field.regAddr).arg(reply->errorString()));
                }
                else
                {
                    Logger::instance().log(Logger::Info,
                        QString("寄存器 %1 写入成功, 值=%2").arg(field.regAddr).arg(modbusValue));
                }
                reply->deleteLater();
            });
        }
        else
        {
            Logger::instance().log(Logger::Warn,
                QString("寄存器 %1 发送写请求失败").arg(field.regAddr));
        }
    }
}



ReadHandler::ReadHandler(QModbusClient *device, int serverAddr, QObject *parent)
    : QObject(parent), modbusDevice(device), serverAddress(serverAddr)
{
    // 连接状态变化
    connect(modbusDevice, &QModbusClient::stateChanged,
            this, &ReadHandler::onStateChanged);

    // 错误处理
    connect(modbusDevice, &QModbusClient::errorOccurred,
            this, &ReadHandler::onErrorOccurred);

    // 定时器（初始关闭）
    readTimer.setInterval(3000);
    readTimer.setSingleShot(false);
    connect(&readTimer, &QTimer::timeout, this, &ReadHandler::onTimeout);

    // 重连定时器（单次定时，默认关闭）
    reconnectTimer.setInterval(5000);
    reconnectTimer.setSingleShot(true);
    connect(&reconnectTimer, &QTimer::timeout, this, &ReadHandler::tryReconnect);

    Logger::instance().log(Logger::Info, "ReadHandler 初始化完成");
}

ReadHandler::~ReadHandler()
{
    if (readTimer.isActive())
    {
        readTimer.stop();
    }
    if (reconnectTimer.isActive())
    {
        reconnectTimer.stop();
    }
    if (modbusDevice && modbusDevice->state() == QModbusDevice::ConnectedState)
    {
        modbusDevice->disconnectDevice();
    }
    Logger::instance().log(Logger::Info, "ReadHandler 已清理资源");
}

//连接状态变化
void ReadHandler::onStateChanged(int state)
{
    switch (state)
    {
    case QModbusDevice::ConnectedState:
        Logger::instance().log(Logger::Info, "Modbus 已连接，启动定时器");
        readTimer.start();
        if (reconnectTimer.isActive()) reconnectTimer.stop();
        break;

    case QModbusDevice::UnconnectedState:
        Logger::instance().log(Logger::Warn, "Modbus 已断开，停止定时器，准备重连");
        readTimer.stop();
        if (!reconnectTimer.isActive())
        {
            reconnectTimer.start();  // 5 秒后尝试重连
        }
        break;

    default:
        break;
    }
}

//错误处理
void ReadHandler::onErrorOccurred(QModbusDevice::Error error)
{
    if (error == QModbusDevice::NoError)
        return;
    QString msg = QString("Modbus 错误: %1").arg(modbusDevice->errorString());
    Logger::instance().log(Logger::Error, msg);
}

//重连机制
void ReadHandler::tryReconnect()
{
    if (!modbusDevice) return;
    if (modbusDevice->state() == QModbusDevice::UnconnectedState)
    {
        Logger::instance().log(Logger::Info, "尝试重新连接 Modbus 服务器...");
        modbusDevice->connectDevice();
    }
}

//定时数据读取
void ReadHandler::onTimeout()
{
    if (!modbusDevice || modbusDevice->state() != QModbusDevice::ConnectedState)
    {
        Logger::instance().log(Logger::Warn, "设备未连接，跳过本次读取");
        return;
    }

    constexpr int startAddress = 1;
    constexpr int registerCount = 91;
    QModbusDataUnit readUnit(QModbusDataUnit::HoldingRegisters, startAddress, registerCount);

    Logger::instance().log(Logger::Info,
                           QString("发送读取请求: 寄存器 %1 到 %2")
                           .arg(startAddress).arg(startAddress + registerCount));

    if (auto *reply = modbusDevice->sendReadRequest(readUnit, serverAddress))
    {
        if (!reply->isFinished())
        {
            QPointer<QModbusReply> safeReply = reply;
            connect(reply, &QModbusReply::finished, this, [this, safeReply]() {
                if (!safeReply) return;
                parseReply(safeReply);
                safeReply->deleteLater();
                ctrloutput = DistAreaVoltCtrl_Run(&params);
                writeDistAreaVoltCtrlOutput(modbusDevice,ctrloutput);
            });


            /*
            ctrloutput.pcsRemoteStart.valid = 1;
            ctrloutput.pcsRemoteStart.value = 2;
            ctrloutput.pcsCPhaseRemoteReactivePower.valid = 1;
            ctrloutput.pcsCPhaseRemoteReactivePower.value = -12.5;
            ctrloutput.appStatus.valid = 1;
            ctrloutput.appStatus.value = 2;
            */

        }
        else
        {
            reply->deleteLater();
            Logger::instance().log(Logger::Error, "读取请求立即失败");
        }
    }
    else
    {
        Logger::instance().log(Logger::Error, "发送读取请求失败");
    }
}


void ReadHandler::parseReply(QModbusReply *reply)
{
    if (reply->error() != QModbusDevice::NoError)
    {
        Logger::instance().log(Logger::Error,
                               QString("请求错误: %1 (code=%2)")
                               .arg(reply->errorString())
                               .arg(reply->error()));
        return;
    }

    const QModbusDataUnit result = reply->result();
    Logger::instance().log(Logger::Info,
                           QString("收到数据，寄存器数量: %1").arg(result.valueCount()));

    // 寄存器日志
    QStringList regLogs;

    char* structBase = reinterpret_cast<char*>(&params);
    int idx = 0;

    for (int addr = 1; addr <= 91; ++addr)
    {
        // 跳过寄存器 39~50
        if (addr >= 39 && addr <= 50)
            continue;

        int index = addr - result.startAddress();
        if (index < 0 || index >= static_cast<int>(result.valueCount())) continue;

        quint16 raw = result.value(index);

        // 地址38使用uint16，其余使用int16
        double val = (addr == 38) ? static_cast<uint16_t>(raw) : static_cast<int16_t>(raw);

        float finalVal = val * gConfFactor_A[idx] + gConfFactor_B[idx];

        // 寄存器日志
        regLogs << QString("寄存器[%1] 原始值=%2, 计算后=%3")
                        .arg(addr).arg(raw).arg(finalVal);


        // 写入结构体
        if (isFieldFloat[idx])
        {
            if (addr == 91)
            {
                Logger::instance().log(Logger::Info,
                    QString("寄存器91写入前 regulationPeriod=%1").arg(params.cPhaseHighVoltTargetVoltage));
            }
            *reinterpret_cast<float*>(structBase + fieldOffsets[idx]) = finalVal;
        }
        else
        {
             *reinterpret_cast<int16_t*>(structBase + fieldOffsets[idx]) = static_cast<int16_t>(finalVal);
            /*
            if(addr == 38)
            {
                *reinterpret_cast<uint16_t*>(structBase + fieldOffsets[idx]) = static_cast<uint16_t>(finalVal);
            }
            else
            {
                *reinterpret_cast<int16_t*>(structBase + fieldOffsets[idx]) = static_cast<int16_t>(finalVal);
                Logger::instance().log(Logger::Info,
                    QString("idx=%1, addr=%2, offset=%3, finalVal=%4")
                        .arg(idx).arg(addr).arg(fieldOffsets[idx]).arg(finalVal));
            }
            */
        }


        if (addr == 91)
        {
                Logger::instance().log(Logger::Info,
                    QString("idx=%1, addr=%2, offset=%3, finalVal=%4")
                        .arg(idx).arg(addr).arg(fieldOffsets[idx]).arg(finalVal));
            Logger::instance().log(Logger::Info,
                QString("寄存器91写入后 regulationPeriod=%1").arg(params.cPhaseHighVoltTargetVoltage));
            //*reinterpret_cast<int16_t*>(structBase + fieldOffsets[idx]) = static_cast<int16_t>(finalVal);
            //Logger::instance().log(Logger::Info,
                //QString("寄存器60写入后 regulationPeriod=%1").arg(params.regulationPeriod));
        }

        idx++;
    }
    params.regulationPeriod = params.regulationPeriod * 1000;
    Logger::instance().log(Logger::Info,
                            QString("配置的调节周期:%1").arg(params.regulationPeriod));

    if(params.regulationPeriod >= 3000 && params.regulationPeriod <= 60000)
    {

       int timeInterval = static_cast<int>(static_cast<int16_t>(params.regulationPeriod));
       Logger::instance().log(Logger::Info,
                               QString("配置的调节周期:%1").arg(timeInterval));
       updateReadTimerInterval(timeInterval);
    }

    if (!regLogs.isEmpty())
        Logger::instance().log(Logger::Info,
                               QString("寄存器解析结果:\n%1").arg(regLogs.join("\n")));

    // 输出整个结构体内容
    QString structLog;
    structLog += QString("bmsCommStatusHeartbeat=%1, gridRatedApparentPower=%2, cPhaseHighVoltTargetVoltage=%3")
                    .arg(params.bmsCommStatusHeartbeat)
                    .arg(params.gridRatedApparentPower)
                    .arg(params.cPhaseHighVoltTargetVoltage);
    Logger::instance().log(Logger::Info, QString("结构体最终解析值: %1").arg(structLog));
}

/*
//数据解析
void ReadHandler::parseReply(QModbusReply *reply)
{
    if (reply->error() == QModbusDevice::NoError)
    {
        const QModbusDataUnit result = reply->result();
        Logger::instance().log(Logger::Info,
                               QString("收到数据，寄存器数量: %1").arg(result.valueCount()));

        int idx = 0; // 结构体字段偏移
        QStringList logLines;   // 存储日志信息
        for (int addr = 1; addr <= 91; ++addr)
        {
            // 跳过寄存器 39~50
            if (addr >= 39 && addr <= 50)
                continue;

            int index = addr - result.startAddress();
            if (index < 0 || index >= static_cast<int>(result.valueCount())) continue;

            quint16 raw = result.value(index);

            // 地址38使用uint16，其余使用int16
            double val = (addr == 38) ? static_cast<uint16_t>(raw)
                                      : static_cast<int16_t>(raw);

            // 应用系数
            float finalVal = val * gConfFactor_A[idx] + gConfFactor_B[idx];

            // 打印寄存器信息
            logLines << QString("寄存器[%1] 原始值=%2, 计算后=%3")
                                    .arg(addr).arg(raw).arg(finalVal);

            // switch-case 填充结构体
            switch (idx)
            {
                case 0:  params.gridAPhaseRtvActivePower = finalVal; break;
                case 1:  params.gridBPhaseRtvActivePower = finalVal; break;
                case 2:  params.gridCPhaseRtvActivePower = finalVal; break;
                case 3:  params.gridThreePhaseRtvActivePower = finalVal; break;
                case 4:  params.gridAPhaseRtvApparentPower = finalVal; break;
                case 5:  params.gridBPhaseRtvApparentPower = finalVal; break;
                case 6:  params.gridCPhaseRtvApparentPower = finalVal; break;
                case 7:  params.gridThreePhaseRtvApparentPower = finalVal; break;
                case 8:  params.gridAPhaseVoltage = finalVal; break;
                case 9:  params.gridBPhaseVoltage = finalVal; break;
                case 10: params.gridCPhaseVoltage = finalVal; break;
                case 11: params.gridCT = static_cast<uint16_t>(finalVal); break;
                case 12: params.gridPT = static_cast<uint16_t>(finalVal); break;
                case 13: params.gridFault = static_cast<uint16_t>(finalVal); break;
                case 14: params.gridCommStatus = static_cast<uint16_t>(finalVal); break;
                case 15: params.pcsRtvRunStatus = static_cast<uint16_t>(finalVal); break;
                case 16: params.pcsAPhaseVoltage = finalVal; break;
                case 17: params.pcsBPhaseVoltage = finalVal; break;
                case 18: params.pcsCPhaseVoltage = finalVal; break;
                case 19: params.pcsAPhaseRtvActivePower = finalVal; break;
                case 20: params.pcsBPhaseRtvActivePower = finalVal; break;
                case 21: params.pcsCPhaseRtvActivePower = finalVal; break;
                case 22: params.pcsThreePhaseRtvActivePower = finalVal; break;
                case 23: params.pcsAPhaseRtvReactivePower = finalVal; break;
                case 24: params.pcsBPhaseRtvReactivePower = finalVal; break;
                case 25: params.pcsCPhaseRtvReactivePower = finalVal; break;
                case 26: params.pcsThreePhaseRtvReactivePower = finalVal; break;
                case 27: params.pcsAPhaseRtvApparentPower = finalVal; break;
                case 28: params.pcsBPhaseRtvApparentPower = finalVal; break;
                case 29: params.pcsCPhaseRtvApparentPower = finalVal; break;
                case 30: params.pcsThreePhaseRtvApparentPower = finalVal; break;
                case 31: params.pcsFault = static_cast<uint16_t>(finalVal); break;
                case 32: params.pcsCommStatus = static_cast<uint16_t>(finalVal); break;
                case 33: params.bmsMaxChargeCurrent = finalVal; break;
                case 34: params.bmsMaxDischargeCurrent = finalVal; break;
                case 35: params.bmsSoc = static_cast<uint16_t>(finalVal); break;
                case 36: params.bmsFault = static_cast<uint16_t>(finalVal); break;
                case 37: params.bmsCommStatusHeartbeat = static_cast<uint16_t>(finalVal); break;
                // 跳过39~50寄存器，这里idx已经按结构体字段顺序连续
                case 38: params.gridRatedApparentPower = static_cast<uint16_t>(finalVal); break;
                case 39: params.pcsRatedPower = static_cast<uint16_t>(finalVal); break;
                case 40: params.pcsAPhaseRatedPower = static_cast<uint16_t>(finalVal); break;
                case 41: params.pcsBPhaseRatedPower = static_cast<uint16_t>(finalVal); break;
                case 42: params.pcsCPhaseRatedPower = static_cast<uint16_t>(finalVal); break;
                case 43: params.bmsSocUpperLimit = finalVal; break;
                case 44: params.bmsSocLowerLimit = finalVal; break;
                case 45: params.voltageRegulationEnable = static_cast<uint16_t>(finalVal); break;
                case 46: params.voltageTargetDeadband = finalVal; break;
                case 47: params.regulationPeriod = static_cast<uint16_t>(finalVal); break;
                case 48: params.regulationStep = finalVal; break;
                case 49: params.hour0StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 50: params.hour1StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 51: params.hour2StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 52: params.hour3StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 53: params.hour4StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 54: params.hour5StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 55: params.hour6StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 56: params.hour7StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 57: params.hour8StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 58: params.hour9StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 59: params.hour10StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 60: params.hour11StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 61: params.hour12StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 62: params.hour13StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 63: params.hour14StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 64: params.hour15StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 65: params.hour16StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 66: params.hour17StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 67: params.hour18StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 68: params.hour19StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 69: params.hour20StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 70: params.hour21StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 71: params.hour22StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 72: params.hour23StrategySetting = static_cast<uint16_t>(finalVal); break;
                case 73: params.aPhaseLowVoltTargetVoltage = finalVal; break;
                case 74: params.aPhaseHighVoltTargetVoltage = finalVal; break;
                case 75: params.bPhaseLowVoltTargetVoltage = finalVal; break;
                case 76: params.bPhaseHighVoltTargetVoltage = finalVal; break;
                case 77: params.cPhaseLowVoltTargetVoltage = finalVal; break;
                case 78: params.cPhaseHighVoltTargetVoltage = finalVal; break;
                default: break;
            }
            idx++;
        }
        if (!logLines.isEmpty())
        {
            Logger::instance().log(Logger::Info,
                QString("寄存器解析结果:\n%1").arg(logLines.join("\n")));
        }
        qDebug() << QString("params.bmsCommStatusHeartbeat[%1],params.gridRatedApparentPower[%2],params.cPhaseHighVoltTargetVoltage [%3]").arg(params.bmsCommStatusHeartbeat) .arg(params.gridRatedApparentPower) .arg(params.cPhaseHighVoltTargetVoltage);
    }
    else
    {
        Logger::instance().log(Logger::Error,
                               QString("请求错误: %1 (code=%2)").arg(reply->errorString()) .arg(reply->error()));
    }
}
*/
