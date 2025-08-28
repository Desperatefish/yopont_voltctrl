QT += core serialbus serialport
TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
TARGET = Modbus_Tool

SOURCES += main.cpp modbus_data.cpp Logger.cpp \
    disareavoltctrl.cpp




HEADERS += modbus_data.h Logger.h disareavoltctrl.h


