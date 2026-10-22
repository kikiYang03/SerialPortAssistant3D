# svg：图标运行时着色（ui/style/IconTint 用 QSvgRenderer 把 currentColor 换色后渲染）
QT += core gui serialport network opengl widgets svg

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
INCLUDEPATH += $$PWD/ui
INCLUDEPATH += $$PWD/ui/style
INCLUDEPATH += $$PWD/ui/frameless
INCLUDEPATH += $$PWD/ui/shell
INCLUDEPATH += $$PWD/ui/widgets
INCLUDEPATH += $$PWD/admins

SOURCES += \
    admins/adminmode.cpp \
    src/Coordinate/coordinate.cpp \
    main.cpp \
    mainwindow.cpp \
    src/GL/glwidget.cpp \
    src/GL/ros3dpage.cpp \
    src/Params/params.cpp \
    src/Serialport/serialport.cpp \
    ui/dualrangeslider.cpp \
    ui/frameless/FramelessHelper.cpp \
    ui/frameless/FramelessWindow.cpp \
    ui/frameless/WindowGeometry.cpp \
    ui/style/IconTint.cpp \
    ui/style/Theme.cpp \
    ui/widgets/IconButton.cpp \
    ui/widgets/CollapsibleSection.cpp \
    ui/widgets/FloatingBar.cpp \
    ui/widgets/PanelRail.cpp \
    ui/widgets/PointsCapsule.cpp \
    ui/widgets/SegmentedControl.cpp \
    ui/shell/CommandBar.cpp \
    ui/shell/StatusBarEx.cpp \
    ui/shell/TitleBar.cpp \
    ui/waitdialog.cpp \
    utils/protocol/maparchive.cpp \
    utils/protocol/protocolros3d.cpp \
    utils/protocol/protocolrouter.cpp \
    utils/protocol/tftree.cpp \
    utils/tcp/tcpclient.cpp \
    utils/config/config.cpp

HEADERS += \
    admins/adminmode.h \
    src/Coordinate/coordinate.h \
    mainwindow.h \
    src/GL/glwidget.h \
    src/GL/ros3dpage.h \
    src/Params/params.h \
    src/Serialport/serialport.h \
    stdafx.h \
    ui/dualrangeslider.h \
    ui/frameless/FramelessHelper.h \
    ui/frameless/FramelessWindow.h \
    ui/frameless/WindowGeometry.h \
    ui/style/IconTint.h \
    ui/style/Theme.h \
    ui/widgets/IconButton.h \
    ui/widgets/CollapsibleSection.h \
    ui/widgets/RangeLimits.h \
    ui/widgets/FloatingBar.h \
    ui/widgets/PanelRail.h \
    ui/widgets/PointsCapsule.h \
    ui/widgets/SegmentedControl.h \
    ui/shell/CommandBar.h \
    ui/shell/StatusBarEx.h \
    ui/shell/TitleBar.h \
    ui/waitdialog.h \
    utils/protocol/maparchive.h \
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
    resource.qrc \
    resource/style/style.qrc \
    resource/icons/icons.qrc

CONFIG += file_copies
COPIES += ini
ini.files = $$PWD/config/config.ini
ini.path   = $$OUT_PWD

# 构建产物在 $$DESTDIR 子目录（debug\ / release\）里，config.ini 只拷到 $$OUT_PWD，
# 由 utils/config/config.cpp 的查找顺序兜住（先找 exe 目录、再找上一级），
# 所以这里不需要再拷一份到 debug\ 下面。

# win32:LIBS += -lopengl32
win32 {
    LIBS += -lopengl32
    # 无边框窗口（ui/frameless）：Win32 命中测试与窗口度量。
    # 我们自己的目标文件里直接调 user32，MSVC 链接期不会从 Qt 的 DLL 里「传递」到这些导入库。
    LIBS += -luser32
    # DwmSetWindowAttribute 设 Win11 圆角偏好
    LIBS += -ldwmapi
} else {
    LIBS += -lGL
}
