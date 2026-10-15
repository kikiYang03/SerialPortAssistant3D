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

// 一条待写入的参数（参数号 + 值 + 名称）。参数页在点击「写入参数」那一刻把整张表收集成这个列表，
// 交给链路层（SerialPort）统一发送——参数页不持有连接、不查连接、不自己发帧。
// 工单 06 的比对基准快照也用同一份列表，所以字段是「号 + 值」而不是一串裸字节；
// 名称一并带上，是为了让链路层报不一致明细时不必反过来查参数页的表。
struct ParamWrite {
    quint8 id;
    qint16 value;
    QString name;
};
Q_DECLARE_METATYPE(ParamWrite)

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

    // 控制指令链路（TCP / UDP）状态：未连接（含只有串口）时「写入参数」不可点。
    // 与可视化页「保存地图」同一个信号源，两处门禁不会各说各话。
    void setLinkConnected(bool connected);

    // 保存参数等待期间把「模块回读值刷界面」这条路关掉：回读帧只喂链路层的比对器。
    // 不关的话，用户在结果弹出前就会看到界面上的值变成功——而那正是这一步要抓的场景
    // （模块若把某个值改成了别的，界面先被回读值覆盖，就再没人看得出它不一致了）。
    void setEchoSuppressed(bool suppressed);

    // 链路层的保存闭环有了结论（一致 / 有出入 / 超时）：把摘要落在这行状态文字上
    void setSaveSummary(const QString &summary);

private slots:
    void onFoldButtonClicked();
    void on_readButton_clicked();
    void on_writeButton_clicked();
    void on_defaultButton_clicked();

signals:
    // 添加消息显示信号
    void appendMessage(const QString &message);

    // 待写入的参数表：交给链路层发（逐条 AA 10 <参数号> <值> + 一条 AA 00 04 0A 保存终结指令）
    void writeParamsRequested(const QVector<ParamWrite> &params);


private:
    Ui::Params *ui;
    bool m_isGroupFolded;
    QList<QWidget*> valueWidgets;
    static const QVector<Parameter> s_parameters;

    QByteArray m_receiveBuffer;  // 数据接收缓冲区

    void setupTable();
    void setupParameters();
    QWidget* createValueWidget(const QString &id, const QString &range, int defaultValue);
    void restoreDefaultValues();

    int m_expectedParamCount = 8; // 根据实际参数数量设置
    int m_receivedParamCount = 0;

    // 保存参数等待期：模块回读值不刷界面（默认开着，正常读取参数时照刷）
    bool m_echoSuppressed = false;
};

#endif // PARAMS_H
