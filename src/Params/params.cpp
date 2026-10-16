#include "params.h"
#include "ui_params.h"
#include <QHBoxLayout>
#include <QFontMetrics>   // 「参数值」列宽按下拉选项的文本宽度算（见 setupParameters）
#include <QDateTime>   // 状态行 / 日志里的时间戳（原先靠 tcpclient.h 顺带带进来）
#include <QTimer>      // 折叠后延后一趟重摆 cell widget（见 refreshTableLayout）
#include <QDebug>
#include <adminmode.h>

// 参数值那一列的列号（输入控件都挂在这一列上，折叠时要连它们一起藏）
namespace {
constexpr int kValueColumn = 3;
}

const QVector<Parameter> Params::s_parameters = {
    // 表序与 upper/config/通讯协议.md（唯一权威的新协议）的参数ID定义表一致，
    // 也是 AA 00 03 0A 应答与保存后回显的发送顺序。docs/通讯协议.md 那份是过时的对照件，勿照它改。
    // 本仓按 ID 字符串匹配行，顺序只影响可读性；但折叠分组的行号是按下标推导的，插行请勿打乱分组。
    // 「描述」逐字抄自上表的「说明」列，只去掉 Qt 表格不渲染的 Markdown 强调标记（** 与反引号）；
    // 不再夹带本仓自己的补充。要改说明请先改协议表——两边没有机制自动同步。
    // 0x00 取值集合只留新协议参数表列出的 4 项（删掉旧占位 12=m10 / 13=m10p）：
    // 用户不会选中一个协议表里不存在的值，也就不会踩「表外值静默丢弃、链路无任何回执」。
    {"0x00", "雷达型号", "-", "0=mid360, 1=mid360s, 10=N10, 11=N10_P，默认0", 0},
    {"-", "雷达安装位置", "-", "雷达相对于机器人中心的位置 = 安装位姿 T：base_link -> laser_link", 0},
    {"0x01", "X坐标", "-100~100", "雷达位置X坐标值，单位厘米，默认0", 0},
    {"0x02", "Y坐标", "-100~100", "雷达位置Y坐标值，单位厘米，默认0", 0},
    {"0x03", "Z坐标", "-100~100", "雷达位置Z坐标值，单位厘米，默认0", 0},
    {"0x04", "Roll角度", "-180~180", "雷达滚转角，单位度，默认0", 0},
    {"0x05", "Pitch角度", "-180~180", "雷达俯仰角，单位度，默认0", 0},
    {"0x06", "Yaw角度", "-180~180", "雷达偏航角，单位度，默认0", 0},
    {"0x11", "使能与PX4飞控通信", "0-1", "1=使能，模块可与PX4通信，作为位置传感器/执行路径规划，默认1", 1},
    {"0x12", "使能与主控串口通信", "0-1", "1=使能，模块可与主控串口通信，输出坐标/输出控制点/输入目标点，默认1", 1},
    {"0x13", "模式", "0-1", "0=建图、1=定位，默认0。定位模式需要建图模式下保存的地图", 0},
    // 本值默认 0，与协议表（v2.8/14 起）一致。导航装配 ⟺ 本值 1 且模块类型为 1 或 3，
    // 其余组合一律不装规划器；与模块类型正交。重启生效。
    {"0x14", "导航使能", "0-1", "1=使能，启用导航功能，默认0", 0},
    // v2.8/13 新增。插在 0x14 与 0x21 块之间：0x15 本来就空着，插在这里表序仍然升序。
    // 与 v2.6 删掉的老 0x27 filter_radius 不是一回事：那是删号，本号是新号新义、且真接线。
    // 单位归 x/y/z 那一族（协议面 / 缓存 / yaml 全程裸厘米整数，不乘除 100），
    // 不跟 0x21~0x29 那条「长度类 = 米」的惯例——这是有意的，不是抄漏。
    {"0x15", "过滤半径", "0~1000", "设置过滤点云的半径大小，例如过滤机器人自身结构，单位cm，默认10。圆心 = 机器人中心", 10},
    {"-", "路径规划与小车模型参数", "-", "首次使用应断开电机供电，查看规划结果无误", 0},
    {"0x21", "最大移动速度", "0~1000", "规划轨迹的最大速度，单位cm/s，默认100。", 100},
    {"0x22", "最大加速度", "0~1000", "规划轨迹的最大加速度，单位cm/s²，默认100。", 100},
    {"0x23", "起飞高度", "0~1000", "一键起飞设定的高度，单位cm，默认 100。", 100},
    // 0x24 名称取「膨胀半径」，**不**采信协议表的名称列：该表名称列写「机器人半径」、
    // 说明列写「障碍膨胀半径」；模块侧 param_store.cpp 的字段名是 inflation_radius。
    // 照名称列改成「机器人半径」等于把一个自己都说不清的字段名抄进代码。
    {"0x24", "膨胀半径", "0~1000", "障碍膨胀半径，单位cm，默认 10 = 0.10 m。建议设置为 = 机器人半径+10", 10},
    {"0x25", "规划模式", "0~2", "导航链路跑哪种规划：0=2D平面路径规划（例如地面移动小车）、1=3D自由空间路径规划（例如无人机）、2=3D沿地面空间路径规划（例如可爬楼梯机械狗，3D规划但要求轨迹贴合地面）。2D导航版模块仅支持模式0。默认0", 0},
    {"0x26", "小车运动学模型", "0~1", "0=全向，1=差速，默认0。仅在规划模式0生效", 0},
    {"0x27", "目标点容差（小车模型）", "0~1000", "到目标点多近算「到点」，单位cm，默认 15（= 0.15 m）。", 15},
    {"0x28", "最大偏航角速率（小车模型）", "0~1000", "单位度/s，默认 43（≈ 上游 0.75 rad/s）", 43},
    {"0x29", "最小转弯半径（小车模型）", "0~1000", "单位cm，默认 100（= 1.0 m）。全向模型下不生效", 100},
    {"0x99", "模块类型", "0~3", "0=3D定位模块，1=3D导航模块，2=2D定位模块，3=2D导航模块", 0},
    };

Params::Params(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Params)
{
    ui->setupUi(this);

    // 设置表格属性
    setupTable();

    // 初始化参数
    setupParameters();

    // 「写入参数」的门禁跟随链路层：初始不可点，连上 TCP / UDP 后由
    // SerialPort::controlLinkStateChanged 打开（与可视化页「保存地图」同一口径）
    ui->writeButton->setEnabled(false);
}

Params::~Params()
{
    delete ui;
}
// 初始化表格样式
void Params::setupTable()
{
    // 设置表格列数和标题
    ui->tableWidget->setColumnCount(6);
    ui->tableWidget->setHorizontalHeaderLabels(QStringList() << "" << "参数ID" << "参数名称" << "参数值" << "值范围" << "说明");

    // 设置表格属性
    ui->tableWidget->horizontalHeader()->setStretchLastSection(true); // 最后一列拉伸
    ui->tableWidget->verticalHeader()->setVisible(false);
    ui->tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->tableWidget->setAlternatingRowColors(true);

    // 设置列宽 - 减小列宽
    ui->tableWidget->setColumnWidth(0, 25);    // 减小第一列宽度
    ui->tableWidget->setColumnWidth(1, 70);    // 参数ID列
    ui->tableWidget->setColumnWidth(2, 100);   // 参数名称列
    ui->tableWidget->setColumnWidth(3, 120);   // 参数值列（只是下限：setupParameters 会按最长下拉选项再撑宽）
    ui->tableWidget->setColumnWidth(4, 100);   // 值范围列
    // 说明列自动拉伸

    // 设置自动换行和文本显示
    ui->tableWidget->setWordWrap(true);
    ui->tableWidget->setTextElideMode(Qt::ElideNone);

    // 关键：设置行高自适应且紧凑
    ui->tableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->tableWidget->verticalHeader()->setDefaultSectionSize(20); // 设置默认行高
    ui->tableWidget->verticalHeader()->setMinimumSectionSize(20); // 最小行高

    // 设置第一列居中对齐
    ui->tableWidget->horizontalHeader()->setDefaultAlignment(Qt::AlignCenter);

    // 设置支持换行的样式
    ui->tableWidget->setStyleSheet(
        "QTableWidget {"
        "    gridline-color: #d0d0d0;"
        "    selection-background-color: #cde6f7;"
        "    alternate-background-color: #fafafa;"
        "    background-color: #ffffff;"
        "    font-size: 20px;"
        "}"
        "QTableWidget::item {"
        "    padding: 1px 2px;"
        "    border: none;"
        "}"
        "QTableWidget::item:selected {"
        "    background-color: #cde6f7;"
        "}"
        "QHeaderView::section {"
        "    background-color: #f0f0f0;"
        "    padding: 4px;"
        "    border: 1px solid #d0d0d0;"
        "    font-weight: bold;"
        "}"
        );


    // 设置表格的间距属性
    ui->tableWidget->setShowGrid(true);
    ui->tableWidget->setGridStyle(Qt::SolidLine);

    // 设置布局间距
    ui->tableWidget->setContentsMargins(0, 0, 0, 0);

    ui->optLabel->setStyleSheet("font-size: 10px;");
}

// 初始化参数信息
void Params::setupParameters()
{
    const QVector<Parameter>& parameters = s_parameters;

    // 设置行数
    ui->tableWidget->setRowCount(parameters.size());

    // 折叠按钮挂在分组表头行上，表头行的参数ID固定为 "-"。
    // 行号一律按 ID 推导（参数行在表里的下标即行号），以后再插参数不会静默错位。
    auto rowOfId = [&parameters](const QString &id) {
        for (int i = 0; i < parameters.size(); ++i) {
            if (parameters.at(i).id.compare(id, Qt::CaseInsensitive) == 0)
                return i;
        }
        return -1;
    };
    int placementHeaderRow = -1;   // "雷达安装位置" 表头
    int plannerHeaderRow   = -1;   // "路径规划参数" 表头
    for (int i = 0; i < parameters.size(); ++i) {
        if (parameters.at(i).id != "-") continue;
        if (placementHeaderRow < 0) placementHeaderRow = i;
        else if (plannerHeaderRow < 0) plannerHeaderRow = i;
    }
    const int placementFirstRow = rowOfId("0x01");
    const int placementLastRow  = rowOfId("0x06");   // 0x01-0x06
    const int plannerFirstRow   = rowOfId("0x21");
    const int plannerLastRow    = rowOfId("0x29");   // 0x21-0x29

    // 填充表格
    for (int row = 0; row < parameters.size(); ++row) {
        const Parameter &param = parameters[row];

        // 在两个分组表头行上添加折叠按钮
        if (row == placementHeaderRow || row == plannerHeaderRow) {
            // 创建容器widget确保按钮居中
            QWidget *container = new QWidget();
            QHBoxLayout *layout = new QHBoxLayout(container);
            layout->setContentsMargins(0, 0, 0, 0);
            layout->setAlignment(Qt::AlignCenter);

            QPushButton *foldButton = new QPushButton();
            foldButton->setText("−");
            foldButton->setFixedSize(20, 20);
            foldButton->setToolTip("折叠 / 展开这一组参数");

            // 分组状态留在成员里，不留在按钮上：按钮是 cell widget，
            // 表格重新布局时它可以被重建，「挂在它身上的状态」就跟着没了。
            FoldGroup group;
            group.button = foldButton;
            if (row == placementHeaderRow) {
                group.firstRow = placementFirstRow;
                group.lastRow  = placementLastRow;
            } else {
                group.firstRow = plannerFirstRow;
                group.lastRow  = plannerLastRow;
            }
            m_foldGroups.append(group);

            connect(foldButton, &QPushButton::clicked, this, &Params::onFoldButtonClicked);

            layout->addWidget(foldButton);
            ui->tableWidget->setCellWidget(row, 0, container);
        } else {
            // 其他行创建空的居中对齐item
            QTableWidgetItem *emptyItem = new QTableWidgetItem();
            emptyItem->setTextAlignment(Qt::AlignCenter);
            ui->tableWidget->setItem(row, 0, emptyItem);
        }

        // 参数ID
        QTableWidgetItem *idItem = new QTableWidgetItem(param.id);
        ui->tableWidget->setItem(row, 1, idItem);

        // 参数名称
        QTableWidgetItem *nameItem = new QTableWidgetItem(param.name);
        ui->tableWidget->setItem(row, 2, nameItem);

        // 值范围
        QTableWidgetItem *rangeItem = new QTableWidgetItem(param.range);
        ui->tableWidget->setItem(row, 4, rangeItem);

        // 说明 - 正确设置文本换行
        QTableWidgetItem *descItem = new QTableWidgetItem(param.description);
        descItem->setToolTip(param.description); // 添加tooltip以便鼠标悬停时显示完整文本
        ui->tableWidget->setItem(row, 5, descItem);

        // 为参数值创建输入控件
        QWidget *valueWidget = createValueWidget(param.id, param.range, param.defaultValue);
        valueWidgets.append(valueWidget);
        ui->tableWidget->setCellWidget(row, kValueColumn, valueWidget);

        // 0x99 模块类型已改为对所有用户可见
    }

    // 「参数值」列宽按这一列里最长的一个下拉选项撑开：0x25 的三个模式名照协议表原文写，
    // 最长的是「3D沿地面空间路径规划」，setupTable 里写死的 120px 装不下，下拉框就把
    // 中间那几个字省略掉了。这里按控件自己的字体度量算，而不是再写死一个像素数——
    // 表格样式里的 font-size 一改、系统 DPI 一变，写死的值又会开始截断。
    // 数码框那几行本来就比这窄，所以拿现有列宽兜底即可。
    // 量的是表格自己的字体：font-size 是打在 QTableWidget 上的，下拉框继承的就是这一份。
    int valueColumnWidth = ui->tableWidget->columnWidth(kValueColumn);
    const QFontMetrics fm(ui->tableWidget->font());
    for (QWidget *valueWidget : valueWidgets) {
        const QComboBox *comboBox = valueWidget->findChild<QComboBox *>();
        if (!comboBox) continue;
        int textWidth = 0;
        for (int i = 0; i < comboBox->count(); ++i)
            textWidth = qMax(textWidth, fm.horizontalAdvance(comboBox->itemText(i)));
        // 文本之外还要留：下拉箭头 + 控件边框内边距（约 30）、createValueWidget 那层布局
        // 左右各 5px（= 10），再留 10 的余量，吃掉不同字体回退带来的几像素出入。
        valueColumnWidth = qMax(valueColumnWidth, textWidth + 50);
    }
    // 必须在 resizeRowsToContents 之前定宽：列一变宽，「说明」列就变窄、换行的行数跟着变，
    // 行高得按新的列宽重算一遍，否则最后一行的说明会被行高切掉。
    ui->tableWidget->setColumnWidth(kValueColumn, valueColumnWidth);

    // 设置行高自适应内容
    ui->tableWidget->resizeRowsToContents();

    // 额外优化：强制紧凑布局
    for (int row = 0; row < ui->tableWidget->rowCount(); ++row) {
        // 设置行高更紧凑
        ui->tableWidget->setRowHeight(row, qMax(ui->tableWidget->rowHeight(row), 25)); // 最小行高25px
    }

    // 行高定完再让 view 摆一遍 cell widget：上面这些 resizeRowsToContents / setRowHeight
    // 都改过行坐标，控件要等下一次滚动才会跟着动，先在这里归位。
    refreshTableLayout();
}

// 填写表格初始信息
QWidget* Params::createValueWidget(const QString &id, const QString &range, int defaultValue)
{
    QWidget *widget = new QWidget();
    QHBoxLayout *layout = new QHBoxLayout(widget);
    layout->setContentsMargins(5, 2, 5, 2);

    // 下拉框一律用 userData 存协议值，并按 findData(defaultValue) 选中 ——
    // 值集合与下标不同（如 0x00 的 10/11、0x99 的 2/3）时，把默认值当索引会显示与实际不符。
    if (id == "0x00") {
        QComboBox *comboBox = new QComboBox();
        comboBox->addItem("mid360", 0);
        comboBox->addItem("mid360s", 1);
        comboBox->addItem("N10", 10);
        comboBox->addItem("N10_P", 11);
        // 12=m10 / 13=m10p 已从新协议的取值集合里删除，不再列出（见 s_parameters 的 0x00 注释）。
        comboBox->setCurrentIndex(qMax(0, comboBox->findData(defaultValue)));
        layout->addWidget(comboBox);
    }
    else if(id == "0x11"){
        QComboBox *comboBox = new QComboBox();
        comboBox->addItem("不能", 0);
        comboBox->addItem("使能", 1);
        comboBox->setCurrentIndex(qMax(0, comboBox->findData(defaultValue)));
        layout->addWidget(comboBox);
    }
    else if(id == "0x12"){
        QComboBox *comboBox = new QComboBox();
        comboBox->addItem("不能", 0);
        comboBox->addItem("使能", 1);
        comboBox->setCurrentIndex(qMax(0, comboBox->findData(defaultValue)));
        layout->addWidget(comboBox);
    }
    else if(id == "0x13"){
        // 0x13「模式」：v2.5 起 0=建图、1=定位（旧义为避障算法使能标志）
        QComboBox *comboBox = new QComboBox();
        comboBox->addItem("建图模式", 0);
        comboBox->addItem("定位模式", 1);
        comboBox->setCurrentIndex(qMax(0, comboBox->findData(defaultValue)));
        layout->addWidget(comboBox);
    }
    else if(id == "0x14"){
        QComboBox *comboBox = new QComboBox();
        comboBox->addItem("不使能", 0);
        comboBox->addItem("使能", 1);
        comboBox->setCurrentIndex(qMax(0, comboBox->findData(defaultValue)));
        layout->addWidget(comboBox);
    }
    else if(id == "0x25"){
        // 规划模式：无单位、不做换算，必须是下拉框（落进末尾的 else 会变成数码框）
        QComboBox *comboBox = new QComboBox();
        // 选项措辞照协议表 0x25 的说明列（0=2D平面…、1=3D自由空间…、2=3D沿地面…），
        // 不再自造「平面地面 / 3D UAV 自由空间 / 3D Ground 有支撑」那套译名——
        // 用户对着协议文档挑值，下拉框里应当是文档上的那几句话。
        // 模块侧的 2d+ground / 3d+uav / 3d+ground 收进说明列，这里不占宽度。
        comboBox->addItem("2D平面路径规划", 0);
        comboBox->addItem("3D自由空间路径规划", 1);
        comboBox->addItem("3D沿地面空间路径规划", 2);
        comboBox->setCurrentIndex(qMax(0, comboBox->findData(defaultValue)));
        layout->addWidget(comboBox);
    }
    else if(id == "0x26"){
        // 运动学模型：枚举，必须下拉框
        QComboBox *comboBox = new QComboBox();
        // 同样照协议表 0x26 的说明列写「全向 / 差速」；omni / differential 是模块侧
        // 配置里的键值名，留在说明列给对着 params.yaml 调的场合看。
        comboBox->addItem("全向", 0);
        comboBox->addItem("差速", 1);
        comboBox->setCurrentIndex(qMax(0, comboBox->findData(defaultValue)));
        layout->addWidget(comboBox);
    }
    else if(id == "0x99"){
        QComboBox *comboBox = new QComboBox();
        comboBox->addItem("3D定位模块", 0);
        comboBox->addItem("3D导航模块", 1);
        comboBox->addItem("2D定位模块", 2);
        comboBox->addItem("2D导航模块", 3);
        comboBox->setCurrentIndex(qMax(0, comboBox->findData(defaultValue)));
        layout->addWidget(comboBox);
    }
    else if(id == "-")
    {

    }
    else {
        QSpinBox *spinBox = new QSpinBox();
        QStringList rangeParts = range.split("~");
        if (rangeParts.size() == 2) {
            spinBox->setRange(rangeParts[0].toInt(), rangeParts[1].toInt());
        }
        spinBox->setValue(defaultValue);
        layout->addWidget(spinBox);
    }

    return widget;
}


// 更新参数
void Params::updateParameter(quint8 paramIdRaw, qint16 value)
{
    // 保存参数等待期：模块回读值只喂链路层的比对器，界面保持「用户下发时看到的样子」。
    // 否则回读值会先把界面刷成模块里的值，等结果弹出来时已经没有对照物了。
    if (m_echoSuppressed) return;

    // 统一转成 "0xXX" 格式字符串，方便匹配表格第 1 列
    const QString idStr = QString("0x%1").arg(paramIdRaw, 2, 16, QLatin1Char('0')).toUpper();

    for (int row = 0; row < ui->tableWidget->rowCount(); ++row) {
        QTableWidgetItem *idItem = ui->tableWidget->item(row, 1);
        if (!idItem || idItem->text().compare(idStr, Qt::CaseInsensitive) != 0)
            continue;                       // ID 不匹配就继续找

        QWidget *valWidget = valueWidgets.at(row);
        if (!valWidget) break;

        QLayout *lay = valWidget->layout();
        if (!lay || lay->count() == 0) break;

        QWidget *editor = lay->itemAt(0)->widget();
        if (auto *box = qobject_cast<QComboBox *>(editor)) {
            int idx = box->findData(value);      // 雷达型号/工作模式 用 userData 保存
            if (idx >= 0) box->setCurrentIndex(idx);
        } else if (auto *spin = qobject_cast<QSpinBox *>(editor)) {
            spin->setValue(value);
        }

        // qDebug() << "[Params] 刷新界面参数" << idStr << "=" << value;
        break;                                  // 找到就结束
    }
}


// 获取当前参数
// 与「写入参数」同一条规矩：这里不持有连接、不查连接、不自己拼帧。
// 0x03 的应答是**按表序发的全表 AA 10（现 22 条）、没有终结帧**，所以「收齐了没」只能由
// 收帧的那一侧（链路层）来判——本页只负责报出「表里有多少条」这个分母。
void Params::on_readButton_clicked()
{
    // 分母从表里数：带参数ID的行才是参数，"-" 那两行是折叠分组表头。
    // 不写死 21：将来参数表插行，这里的期望条数跟着走，不会停在一个过时的常数上。
    int expectedCount = 0;
    for (int row = 0; row < ui->tableWidget->rowCount(); ++row) {
        QTableWidgetItem *idItem = ui->tableWidget->item(row, 1);
        if (idItem && idItem->text() != "-") ++expectedCount;
    }

    ui->optLabel->setText("正在获取所有参数...");
    ui->optLabel->setStyleSheet("color: blue;");

    QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss >> 用户操作: ");
    emit appendMessage(timestamp + "读取当前参数");

    // 发送与等待同处一地：链路层负责发 AA 00 03 0A、收齐后弹结果（同 writeParamsRequested）
    emit readParamsRequested(expectedCount);
}

// 写入参数按钮
// 只做一件事：把界面上要写入的参数收集成表，交给链路层（SerialPort）发送。
// 这里不持有连接、不查连接、不自己拼帧——发送与等待必须同处一地，
// 否则会出现「发了但没人等」或「等了但没发」（工单 06 就靠这一票）。
void Params::on_writeButton_clicked()
{
    QVector<ParamWrite> params;
    for (int row = 0; row < valueWidgets.size(); ++row) {
        QTableWidgetItem *idItem = ui->tableWidget->item(row, 1);
        if (!idItem || idItem->text() == "-") continue; // 跳过无参数ID的行

        bool ok = false;
        const quint8 paramId = static_cast<quint8>(idItem->text().toUShort(&ok, 0));
        if (!ok) continue;  // 表头行（"-"）已跳过，其余行的 ID 都是 "0xXX"

        QWidget *widget = valueWidgets.at(row);
        QLayout *layout = widget->layout();
        if (!layout || layout->count() == 0) continue;

        QWidget *valueControl = layout->itemAt(0)->widget();
        // 名称跟着一起收：链路层报不一致明细时要按参数号点名，不该反过来查参数页的表
        QTableWidgetItem *nameItem = ui->tableWidget->item(row, 2);
        const QString name = nameItem ? nameItem->text() : QString();
        // 按照控件类型取值：下拉框取 userData（协议值），数码框取显示的数值
        if (QComboBox *comboBox = qobject_cast<QComboBox*>(valueControl)) {
            params.append(ParamWrite{paramId, static_cast<qint16>(comboBox->currentData().toInt()), name});
        } else if (QSpinBox *spinBox = qobject_cast<QSpinBox*>(valueControl)) {
            params.append(ParamWrite{paramId, static_cast<qint16>(spinBox->value()), name});
        }
    }

    if (params.isEmpty()) {
        QMessageBox::warning(this, "警告", "没有找到需要写入的参数");
        return;
    }

    QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss >> 用户操作: ");
    emit appendMessage(timestamp + "写入参数");

    // 下面这行只说明「已下发、正在等」——成功与否由链路层的回读比对给出（工单 06），
    // 这里不能先说「写入完成」：那正是要替掉的那句假话。
    ui->optLabel->setText("正在保存参数，等待模块回读…");
    ui->optLabel->setStyleSheet("color: blue;");

    // 同线程直连交给链路层：它负责发帧、等待、比对、弹结果（不阻塞，等的是事件循环）
    emit writeParamsRequested(params);
}

// 控制指令链路状态（TCP / UDP）：未连接（含只有串口）时「写入参数」不可点
void Params::setLinkConnected(bool connected)
{
    ui->writeButton->setEnabled(connected);
}

// 保存参数等待期：模块回读值不刷界面（见 params.h 的说明）
void Params::setEchoSuppressed(bool suppressed)
{
    m_echoSuppressed = suppressed;
}

// 保存闭环的结论落在状态行上（弹窗之后再补一句，页面上也留个痕）
void Params::setSaveSummary(const QString &summary)
{
    ui->optLabel->setText(summary);
    ui->optLabel->setStyleSheet("color: blue;");
}

// 读取闭环的结论落在同一行状态文字上（收齐 / 超时）
void Params::setReadSummary(const QString &summary)
{
    ui->optLabel->setText(summary);
    ui->optLabel->setStyleSheet("color: blue;");
}
// 折叠按钮槽函数
void Params::onFoldButtonClicked()
{
    QPushButton *button = qobject_cast<QPushButton*>(sender());
    if (!button) return;

    for (int i = 0; i < m_foldGroups.size(); ++i) {
        if (m_foldGroups.at(i).button != button) continue;
        setGroupFolded(i, !m_foldGroups.at(i).folded);
        return;
    }
}

// 折叠 / 展开一组参数。
//
// 三件事必须一起做，少一件都会出问题：
//  1) setRowHidden() 藏行——但它只藏得住「行里的 item」，藏不住 setCellWidget()
//     装进去的控件（QTBUG-22490）。只做这一步，折起来那几行的数码框 / 下拉框会原地留着，
//     压到下面已经上移的行上，看着就是「折叠错位」。
//  2) 所以行里的输入控件再显式藏一遍（展开时对应地放回来）。控件只是藏起来、不销毁，
//     值还在里面：折叠状态下「写入参数」「恢复默认」照旧能读到它们。
//  3) 最后逼 view 重摆 cell widget。藏行之后 view 不会主动重摆，要等下一次滚动 /
//     改窗口尺寸才归位；这中间折叠按钮停在旧坐标上（行已经不是那一行了），
//     于是就「时而出现时而消失」。updateGeometries 就是用来干这个的。
void Params::setGroupFolded(int groupIndex, bool folded)
{
    if (groupIndex < 0 || groupIndex >= m_foldGroups.size()) return;
    FoldGroup &group = m_foldGroups[groupIndex];

    for (int row = group.firstRow; row <= group.lastRow; ++row) {
        if (row < 0 || row >= ui->tableWidget->rowCount()) continue;

        ui->tableWidget->setRowHidden(row, folded);
        if (QWidget *editor = ui->tableWidget->cellWidget(row, kValueColumn))
            editor->setVisible(!folded);
    }

    if (group.button)
        group.button->setText(folded ? "+" : "−");
    group.folded = folded;

    refreshTableLayout();
}

// 逼 QTableView 重新摆一遍 cell widget。
// updateGeometries() 是 protected 槽（qabstractitemview.h 的 protected Q_SLOTS），
// 外面直接喊不到，只能过元对象系统——这也正是「cell widget 错位」那一票的标准解法。
void Params::refreshTableLayout()
{
    QMetaObject::invokeMethod(ui->tableWidget, "updateGeometries", Qt::DirectConnection);

    // 再排一趟，让布局落定。表头是 ResizeToContents，藏行会让它延后重算一次行高
    // （QHeaderView 内部的 0 定时器），那一趟排在上面这次摆位之后，会把行坐标再挪一次，
    // 挪完 cell widget 又不跟手了——「按钮时而出现时而消失」就出在这段空窗里。
    // 这里只重复「摆位」这一个动作，不再回调本函数（否则 0 定时器自我续命、空转）。
    QTimer::singleShot(0, this, [this]() {
        QMetaObject::invokeMethod(ui->tableWidget, "updateGeometries", Qt::DirectConnection);
    });
}

// 点击恢复默认参数按钮
void Params::on_defaultButton_clicked()
{
    restoreDefaultValues();
    QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss >> 用户操作: ");
    emit appendMessage(timestamp + "恢复参数默认值");
    ui->optLabel->setText("恢复参数默认值");
    ui->optLabel->setStyleSheet("color: blue;");
    QMessageBox::information(this, "恢复默认", "上位机已恢复默认参数，需写入参数");
}

// 恢复默认参数
void Params::restoreDefaultValues()
{
    for (int row = 0; row < valueWidgets.size(); ++row) {
        QWidget *widget = valueWidgets.at(row);
        QLayout *layout = widget->layout();
        if (!layout || layout->count() == 0) continue;

        QWidget *valueControl = layout->itemAt(0)->widget();
        if (QComboBox *comboBox = qobject_cast<QComboBox*>(valueControl)) {
            // 按 userData 找下标：默认值是协议值，不是下拉框索引
            int index = comboBox->findData(s_parameters.at(row).defaultValue);
            if (index >= 0) comboBox->setCurrentIndex(index);
        } else if (QSpinBox *spinBox = qobject_cast<QSpinBox*>(valueControl)) {
            spinBox->setValue(s_parameters.at(row).defaultValue); // 直接读表
        }
    }
}
