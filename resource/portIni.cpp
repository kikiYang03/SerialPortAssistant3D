#include "portIni.h"
#include <QtCore/QtCore>
#include <QFile>
#include <QDebug>

INI_File::INI_File()
{
    m_qstrFileName = QCoreApplication::applicationDirPath() + "/../portInfo.ini";

    qDebug()<<"m_qstrFileName: "<<m_qstrFileName;

    //"Config.ini"配置文件，文件存在则打开，不存在则创建
    m_psetting = new QSettings(m_qstrFileName,QSettings::IniFormat);
}

INI_File::~INI_File()
{
    delete m_psetting;
    m_psetting = NULL;
}

// 获取波特率
QString INI_File::GetBaudRate()
{
    QString baudRate = m_psetting->value("/PortInfo/baundrateCb", "115200").toString();
    qDebug() << "读取波特率:" << baudRate;
    return baudRate;
}

// 获取数据位
QString INI_File::GetDataBit()
{
    QString dataBit = m_psetting->value("/PortInfo/databitCb", "8").toString();
    return dataBit;
}

// 获取停止位
QString INI_File::GetStopBit()
{
    QString stopBit = m_psetting->value("/PortInfo/stopbitCb", "1").toString();
    return stopBit;
}

// 获取校验位
QString INI_File::GetCheckBit()
{
    QString checkBit = m_psetting->value("/PortInfo/checkbitCb", "none").toString();
    return checkBit;
}
