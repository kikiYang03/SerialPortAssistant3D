#ifndef PARAMS_H
#define PARAMS_H

#include <QWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QSpinBox>
#include <QComboBox>
#include <QMessageBox>
#include "tcpclient.h"  // 添加TCP客户端头文件


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

    QByteArray m_receiveBuffer;  // 数据接收缓冲区
    // 添加TCP客户端访问方法
    TcpClient* getTcpClient() { return TcpClient::getInstance(); }

    void setupTable();
    void setupParameters();
    QWidget* createValueWidget(const QString &id, const QString &range, int defaultValue);
    void restoreDefaultValues();

    // 参数通信相关方法
    void sendParameterWriteRequest(const QString &paramId, int value);

    int m_expectedParamCount = 8; // 根据实际参数数量设置
    int m_receivedParamCount = 0;
};

#endif // PARAMS_H
