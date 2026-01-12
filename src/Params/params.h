#ifndef PARAMS_H
#define PARAMS_H

#include <QWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QSpinBox>
#include <QComboBox>
#include <QMessageBox>
#include "tcpclient.h"  // 添加TCP客户端头文件
#include "protocolhandler.h"

// 定义参数数据
struct Parameter {
    QString id;
    QString name;
    QString range;
    QString description;
    int defaultValue;
};

namespace Ui {
class Params;
}

class Params : public QWidget
{
    Q_OBJECT

public:
    explicit Params(QWidget *parent = nullptr);
    ~Params();

public slots:
    void updateParameter(quint8 paramIdRaw, qint16 value);

private slots:
    void onFoldButtonClicked();
    void on_readButton_clicked();
    void on_writeButton_clicked();
    void on_defaultButton_clicked();

signals:
    // 添加消息显示信号
    void appendMessage(const QString &message);


private:
    Ui::Params *ui;
    bool m_isGroupFolded;
    QList<QWidget*> valueWidgets;
    static const QVector<Parameter> s_parameters;
    // SerialPort *m_serialPort;  // 通信接口指针

    QByteArray m_receiveBuffer;  // 数据接收缓冲区
    ProtocolHandler *m_protocolHandler;  // 协议处理器
    // 添加TCP客户端访问方法
    TcpClient* getTcpClient() { return TcpClient::getInstance(); }

    void setupTable();
    void setupParameters();
    QWidget* createValueWidget(const QString &id, const QString &range, int defaultValue);
    void updateTableFromWidgets();
    void restoreDefaultValues();

    // 参数通信相关方法
    void sendParameterReadRequest(const QString &paramId);
    void sendParameterWriteRequest(const QString &paramId, int value);
    QByteArray buildParameterFrame(quint8 command, const QString &paramId, int value = 0);
    void parseParameterResponse(const QByteArray &data);

    void processSingleFrame(const QByteArray &frame);
    void updateParameterValue(const QString &paramId, int value);

    int m_expectedParamCount = 8; // 根据实际参数数量设置
    int m_receivedParamCount = 0;
};

#endif // PARAMS_H
