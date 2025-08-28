#ifndef MODBUS_DATA_H
#define MODBUS_DATA_H

//#pragma once
#include <QObject>
#include <QTimer>
#include <QModbusReply>
#include <QModbusDataUnit>

class QModbusClient;

class ReadHandler : public QObject
{
    Q_OBJECT
public:
    explicit ReadHandler(QModbusClient *device, int serverAddr, QObject *parent = nullptr);
    ~ReadHandler();

private slots:
    void onTimeout();
    void onStateChanged(int state);               // 连接状态改变
    void onErrorOccurred(QModbusDevice::Error error); // 错误处理
    void tryReconnect();                          // 自动重连

private:
    void parseReply(QModbusReply *reply);
    void updateReadTimerInterval(int newMs);    //修改定时器周期，调节周期

private:
    QModbusClient *modbusDevice;
    int serverAddress;
    QTimer readTimer;
    QTimer reconnectTimer; // 自动重连用
};


#endif // MODBUS_DATA_H
