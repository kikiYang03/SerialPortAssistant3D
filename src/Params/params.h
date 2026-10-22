#ifndef PARAMS_H
#define PARAMS_H

#include <QWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QSpinBox>
#include <QComboBox>
#include <QMessageBox>
// 不带 tcpclient.h：参数页既不发帧也不查连接，读 / 写两条链都交给链路层


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

    // 读取参数的请求入口（工单 06）：命令栏的「读取参数」按钮直接调这里，
    // 与页内那颗「获取参数」按钮走**同一条**路径（数条数 → 写状态行 → 发
    // readParamsRequested），不因为入口不同而有两套行为。
    void requestRead();

    // 控制指令链路（TCP / UDP）状态：未连接（含只有串口）时「写入参数」不可点。
    // 与可视化页「保存地图」同一个信号源，两处门禁不会各说各话。
    void setLinkConnected(bool connected);

    // 保存参数等待期间把「模块回读值刷界面」这条路关掉：回读帧只喂链路层的比对器。
    // 不关的话，用户在结果弹出前就会看到界面上的值变成功——而那正是这一步要抓的场景
    // （模块若把某个值改成了别的，界面先被回读值覆盖，就再没人看得出它不一致了）。
    void setEchoSuppressed(bool suppressed);

    // 链路层的保存闭环有了结论（一致 / 有出入 / 超时）：把摘要落在这行状态文字上
    void setSaveSummary(const QString &summary);

    // 链路层的读取闭环有了结论（收齐 / 超时）：同样落在状态行上
    void setReadSummary(const QString &summary);

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

    // 读取参数请求：本页只出「表里有多少条」，帧由链路层发、收齐也由链路层判
    // （0x03 的应答没有终结帧，只能按条数收齐 —— 见 SerialPort::onReadParamsRequested）
    void readParamsRequested(int expectedCount);


private:
    Ui::Params *ui;
    QList<QWidget*> valueWidgets;
    static const QVector<Parameter> s_parameters;

    // 一个可折叠分组：表头行上的折叠按钮 + 它管辖的参数行区间（闭区间，行号 = s_parameters 下标）。
    // 折叠状态跟按钮一起存在这里，而不是挂在控件上（原先用 setProperty("folded")）：
    // cell widget 是表格的从属物，表格一重新布局就可能被重建 / 重新挂载，
    // 挂在上面的状态会跟界面脱节——图标和实际折没折对不上，就是「错位」的一半。
    struct FoldGroup {
        QPushButton *button = nullptr;
        int firstRow = -1;
        int lastRow = -1;
        bool folded = false;
    };
    QVector<FoldGroup> m_foldGroups;

    void setupTable();
    void setupParameters();
    QWidget* createValueWidget(const QString &id, const QString &range, int defaultValue);
    void restoreDefaultValues();

    // 折叠 / 展开第 groupIndex 组。三件事必须一起做（藏行、藏行里的输入控件、重摆 cell widget），
    // 只 setRowHidden 是藏不干净的——原因见 params.cpp 里的说明。
    void setGroupFolded(int groupIndex, bool folded);
    // 逼 view 重摆所有 cell widget：藏行之后它不会自己重摆，要等下一次滚动 / 改尺寸
    void refreshTableLayout();

    // 保存参数等待期：模块回读值不刷界面（默认开着，正常读取参数时照刷）
    bool m_echoSuppressed = false;
};

#endif // PARAMS_H
