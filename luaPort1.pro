QT       += core gui serialport network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
CONFIG += precompile_header
PRECOMPILED_HEADER = $$PWD/stdafx.h

QMAKE_CXXFLAGS += /utf-8

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += $$PWD/src/Coordinate
INCLUDEPATH += $$PWD/src/Params
INCLUDEPATH += $$PWD/src/Serialport
INCLUDEPATH += $$PWD/src/ROSVisualizer
INCLUDEPATH += $$PWD/src/ROSVisualizer3D
INCLUDEPATH += $$PWD/utils/tcp
INCLUDEPATH += $$PWD/utils/protocol

SOURCES += \
    src/Coordinate/coordinate.cpp \
    main.cpp \
    mainwindow.cpp \
    src/Params/params.cpp \
    src/ROSVisualizer3D/rosvisualizer3d.cpp \
    src/Serialport/serialport.cpp \
    src/ROSVisualizer/rosvisualizer.cpp \
    utils/protocol/protocolhandler.cpp \
    utils/tcp/tcpclient.cpp \
    utils/config/config.cpp

HEADERS += \
    src/Coordinate/coordinate.h \
    mainwindow.h \
    src/Params/params.h \
    src/ROSVisualizer3D/rosvisualizer3d.h \
    src/Serialport/serialport.h \
    stdafx.h \
    src/ROSVisualizer/rosvisualizer.h \
    utils/protocol/protocolhandler.h \
    utils/tcp/tcpclient.h \
    utils/config/config.h

FORMS += \
    form.ui \
    src/Coordinate/coordinate.ui \
    mainwindow.ui \
    src/Params/params.ui \
    src/ROSVisualizer3D/rosvisualizer3d.ui \
    src/Serialport/serialport.ui \
    src/ROSVisualizer/rosvisualizer.ui

RC_ICONS = images/logo256.ico

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    resource.qrc

CONFIG += file_copies
COPIES += ini
ini.files = $$PWD/config/config.ini
ini.path   = $$OUT_PWD

