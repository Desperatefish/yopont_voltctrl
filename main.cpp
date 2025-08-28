#include <iostream>
#include <QCoreApplication>
#include <QModbusTcpClient>
#include "modbus_data.h"
#include "Logger.h"
#include <QTextCodec>

using namespace std;

int main(int argc, char *argv[])
{

    QCoreApplication app(argc, argv);
    Logger::instance().setLogDirectory("/userdata/ems/modbus/log");
    Logger::instance().setMinLevel(Logger::Info);
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));
    qSetMessagePattern("[%{time yyyy-MM-dd hh:mm:ss.zzz}] "
                           "%{type} (%{function}) - %{message}");

    QModbusTcpClient modbusDevice;
    modbusDevice.setConnectionParameter(QModbusDevice::NetworkAddressParameter, "127.0.0.1");
    modbusDevice.setConnectionParameter(QModbusDevice::NetworkPortParameter, 9000);
    modbusDevice.setTimeout(1000);
    modbusDevice.setNumberOfRetries(3);

    ReadHandler handler(&modbusDevice, 1);

    // 这里不管 connectDevice 成功与否，程序都继续运行，靠自动重连机制保证后续连接
    modbusDevice.connectDevice();
    Logger::instance().log(Logger::Info, "正在尝试连接 Modbus 服务器...");

    QObject::connect(&app, &QCoreApplication::aboutToQuit, [&]() {
        Logger::instance().log(Logger::Info, "程序即将退出，清理资源...");
        modbusDevice.disconnectDevice();
    });

    return app.exec();
}
