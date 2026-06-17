# luaPort1.pro修改内容：
增加：
```
win32 {
    LIBS += -lopengl32
} else {
    LIBS += -lGL
}

win32: QMAKE_CXXFLAGS += /utf-8
else: QMAKE_CXXFLAGS += -finput-charset=UTF-8 -fexec-charset=UTF-8

```

linux环境搭建：
```
# Ubuntu/Debian
sudo apt install build-essential qt5-default qtbase5-dev qtbase5-dev-tools \
    libqt5serialport5 libqt5serialport5-dev libgl1-mesa-dev libglu1-mesa-dev

# 安装Qt串口模块
sudo apt install qtbase5-dev qtbase5-dev-tools libqt5serialport5 libqt5serialport5-dev


# 或 Qt6
sudo apt install qt6-base-dev qt6-serialport-generic-dev libgl1-mesa-dev
```
# 打包方式
## 方案1：linuxdeployqt（已成功）
```
# 安装 linuxdeployqt
# 下载
wget https://github.com/probonopd/linuxdeployqt/releases/download/continuous/linuxdeployqt-continuous-x86_64.AppImage
chmod +x linuxdeployqt-continuous-x86_64.AppImage


# 编译发布版
qmake -r CONFIG+=release luaPort1.pro
make

# 打包
./linuxdeployqt-continuous-x86_64.AppImage your_app -appimage
```

## 方案2：CMake+cpack(未尝试)
```
将 .pro 转换为 CMakeLists.txt，可用 cpack 生成 .deb/.rpm/AppImage
```

# 处理图片
```
cp images/你的图标.png SerialPortAssistant.png
```

# 之后修改代码编译步骤
## 如果修改了.pro文件
cd ~/SerialPortAssistant

## 清理旧的构建文件
rm -f luaPort1 luaPort1.gch Makefile

## 重新生成 Makefile
qmake luaPort1.pro

## 编译
make -j$(nproc)

## 重新打包 AppImage
rm -f SerialPortAssistant-x86_64.AppImage
./linuxdeployqt-continuous-x86_64.AppImage ./luaPort1 -appimage

## 如果只是修改.cpp .ini文件
cd ~/SerialPortAssistant
make -j$(nproc)
rm -f SerialPortAssistant-x86_64.AppImage
./linuxdeployqt-continuous-x86_64.AppImage ./luaPort1 -appimage
