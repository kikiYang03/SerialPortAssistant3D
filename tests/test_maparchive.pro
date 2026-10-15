# 地图分块拼装 + CRC-32 校验的独立测试（工单 09）。
# 不并入 luaPort1.pro：那是应用本体，这个是一次性的控制台校验，各编各的。
#
#   qmake tests/test_maparchive.pro && nmake      # 或 make
#   tests/release/test_maparchive.exe             # 退出码 0 = 全过

QT += core
QT -= gui

CONFIG += console c++17
CONFIG -= app_bundle

win32-msvc*: QMAKE_CXXFLAGS += /utf-8

TARGET = test_maparchive
TEMPLATE = app

INCLUDEPATH += $$PWD/../utils/protocol

SOURCES += \
    test_maparchive.cpp \
    ../utils/protocol/maparchive.cpp

HEADERS += \
    ../utils/protocol/maparchive.h \
    ../utils/protocol/protocol_msg.h
