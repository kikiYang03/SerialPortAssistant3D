QT       += core gui serialport network opengl widgets

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
INCLUDEPATH += $$PWD/src/GL
INCLUDEPATH += $$PWD/utils/tcp
INCLUDEPATH += $$PWD/utils/protocol
INCLUDEPATH += $$PWD/thirds/Eigen

SOURCES += \
    src/Coordinate/coordinate.cpp \
    main.cpp \
    mainwindow.cpp \
    src/GL/glwidget.cpp \
    src/GL/ros3dpage.cpp \
    src/Params/params.cpp \
    src/Serialport/serialport.cpp \
    utils/protocol/protocolros3d.cpp \
    utils/protocol/protocolrouter.cpp \
    utils/protocol/tftree.cpp \
    utils/tcp/tcpclient.cpp \
    utils/config/config.cpp

HEADERS += \
    src/Coordinate/coordinate.h \
    mainwindow.h \
    src/GL/glwidget.h \
    src/GL/ros3dpage.h \
    src/Params/params.h \
    src/Serialport/serialport.h \
    stdafx.h \
    utils/protocol/protocol_msg.h \
    utils/protocol/protocolros3d.h \
    utils/protocol/protocolrouter.h \
    utils/protocol/tftree.h \
    utils/tcp/tcpclient.h \
    utils/config/config.h

FORMS += \
    form.ui \
    src/Coordinate/coordinate.ui \
    mainwindow.ui \
    src/Params/params.ui \
    src/Serialport/serialport.ui

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

win32:LIBS += -lopengl32
