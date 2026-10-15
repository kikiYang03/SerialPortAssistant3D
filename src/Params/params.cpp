#include "params.h"
#include "ui_params.h"
#include "protocolrouter.h"
#include <QHBoxLayout>
#include <QDebug>
#include <adminmode.h>

const QVector<Parameter> Params::s_parameters = {
    // 表序与 upper/config/通讯协议.md（唯一权威的新协议）的参数ID定义表一致，
    // 也是 AA 00 03 0A 应答与保存后回显的发送顺序。docs/通讯协议.md 那份是过时的对照件，勿照它改。
    // 本仓按 ID 字符串匹配行，顺序只影响可读性；但折叠分组的行号是按下标推导的，插行请勿打乱分组。
    // 0x00 取值集合只留新协议参数表列出的 4 项（删掉旧占位 12=m10 / 13=m10p）：
    // 用户不会选中一个协议表里不存在的值，也就不会踩「表外值静默丢弃、链路无任何回执」。
    {"0x00", "雷达型号", "-", "0=mid360, 1=mid360s, 10=N10, 11=N10_P，默认0", 0},
    {"-", "雷达安装位置", "-", "雷达相对于机器人中心的位置 = 安装位姿 T：正向 base_init -> laser_init_map，其逆 laser_link -> base_link（旧文档写的 base_link->laser_link 方向相反），遵循FLU（X前，Y左，Z上）", 0},
    {"0x01", "X坐标", "-100~100", "雷达位置X坐标值，单位厘米，默认0", 0},
    {"0x02", "Y坐标", "-100~100", "雷达位置Y坐标值，单位厘米，默认0", 0},
    {"0x03", "Z坐标", "-100~100", "雷达位置Z坐标值，单位厘米，默认0", 0},
    {"0x04", "Roll角度", "-180~180", "雷达滚转角，单位度，默认0", 0},
    {"0x05", "Pitch角度", "-180~180", "雷达俯仰角，单位度，默认0", 0},
    {"0x06", "Yaw角度", "-180~180", "雷达偏航角，单位度，默认0", 0},
    {"0x11", "使能与PX4飞控通信", "0-1", "1=使能，模块可与PX4通信，作为位置传感器/执行路径规划，默认1", 1},
    {"0x12", "使能与主控串口通信", "0-1", "1=使能，模块可与主控串口通信，输出坐标/输出控制点/输入目标点，默认1", 1},
    {"0x13", "模式", "0-1", "v2.5 起由「避障算法使能标志」换义为「模式」（同一个 ID 换义，不是新 ID）：0=建图（默认）、1=定位。默认值也由 1 变为 0。按旧义解析不会报错、只会静默反义（旧义 1=开避障，新义 1=定位）。绕障已恒开，旧义在这条链上没有落点。重启生效", 0},
    {"0x14", "导航使能", "0-1", "v2.5 新增：1=使能（默认）、0=不使能。导航装配 ⟺ 本值 1 且模块类型为 1 或 3，其余组合一律不装规划器；与模块类型正交。重启生效", 1},
    {"-", "路径规划与小车模型参数", "-", "0x21~0x29 连续无缺号（v2.6 删号不复用、不留空位，由后面依次上移填满）。首次使用应断开电机供电，查看规划结果无误", 0},
    {"0x21", "最大移动速度", "0~1000", "规划轨迹的最大速度，单位cm/s，默认100。由新规划器的动力学可行性检查消费：设小即实测限速（轨迹按时间重参数化拉长）；v2.6 起同一格还驱动跟踪上限 controller.max_speed（双映射）。设成 0 会让每条轨迹都判不可行", 100},
    {"0x22", "最大加速度", "0~1000", "规划轨迹的最大加速度，单位cm/s²，默认100。消费方同 0x21（v2.6 起还驱动 controller.max_linear_acceleration），同样设成 0 即全不可行", 100},
    {"0x23", "起飞高度", "0~1000", "一键起飞设定的高度，单位cm，默认100。2D 下它在用：pose_send 把位姿 z 抬到该高度，upper 合成 2D 地图时也把障碍层放在这个高度——同一个值，两处必须相等。消费方是 offboard / pose_send，不是规划器", 100},
    // 0x24 名称取「膨胀半径」，**不**采信新协议表的名称列：该表名称列写「机器人半径」、
    // 说明列写「障碍膨胀半径」，自相矛盾；模块侧 param_store.cpp 的字段名是 inflation_radius。
    // 照名称列改成「机器人半径」等于把一个自己都说不清的字段名抄进代码。
    {"0x24", "膨胀半径", "0~1000", "障碍膨胀半径，单位cm，默认10（= 0.10 m）；0 = 不膨胀。已真接线到规划器 planning.inflation_radius。开箱即带 10 cm 绕障余量，要回到不膨胀必须显式发 0；现场按车体外廓调大", 10},
    {"0x25", "规划模式", "0~2", "导航链路跑哪种规划：0=2D平面路径规划（例如地面移动小车，模块侧 2d+ground，默认）、1=3D自由空间路径规划（例如无人机，3d+uav）、2=3D沿地面空间路径规划（例如可爬楼梯机械狗，3D规划但要求轨迹贴合地面，3d+ground）。无单位、不做换算（模式标识，不是物理量）。重启生效；越界值（含空值）启动即报错、不回落默认。两条导航链路都读它：3D 三档都收；2D 导航版仅支持 0（发 1/2 或越界值启动即报错）；2D 定位不装规划器，此参数不参与", 0},
    {"0x26", "小车运动学模型", "0~1", "0=omni（全向，默认）、1=differential（差速）。仅在规划模式 0 生效。喂规划器 controller.motion_model。只暴露规划器认得的前两个值（本仓两条链都是地面车，uav/ackermann 不暴露）；越界值启动即报错退出、不回落 omni", 0},
    {"0x27", "目标点容差（小车模型）", "0~1000", "到目标点多近算「到点」，单位cm，默认15（= 0.15 m，上游默认）。喂 controller.goal_tolerance", 15},
    {"0x28", "最大偏航角速率（小车模型）", "0~1000", "单位度/s，默认43（≈ 上游 0.75 rad/s）。喂 controller.max_yaw_rate。归角度组：与 roll/pitch/yaw 同口径，×1、不做 cm↔m 换算", 43},
    {"0x29", "最小转弯半径（小车模型）", "0~1000", "单位cm，默认100（= 1.0 m，上游默认）。喂 controller.min_turning_radius。omni 模型下不生效（登记，不报错）", 100},
    {"0x99", "模块类型", "0~3", "0=3D定位模块，1=3D导航模块，2=2D定位模块，3=2D导航模块", 0},
    };

Params::Params(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Params)
    , m_isGroupFolded(false)
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
    ui->tableWidget->setColumnWidth(3, 120);   // 参数值列
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

            // 使用动态属性记录该按钮的状态和需要控制的行号范围
            foldButton->setProperty("folded", false);
            if (row == placementHeaderRow) {
                foldButton->setProperty("startRow", placementFirstRow);
                foldButton->setProperty("endRow", placementLastRow);
            } else {
                foldButton->setProperty("startRow", plannerFirstRow);
                foldButton->setProperty("endRow", plannerLastRow);
            }

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
        ui->tableWidget->setCellWidget(row, 3, valueWidget);

        // 0x99 模块类型已改为对所有用户可见
    }

    // 设置行高自适应内容
    ui->tableWidget->resizeRowsToContents();

    // 额外优化：强制紧凑布局
    for (int row = 0; row < ui->tableWidget->rowCount(); ++row) {
        // 设置行高更紧凑
        ui->tableWidget->setRowHeight(row, qMax(ui->tableWidget->rowHeight(row), 25)); // 最小行高25px
    }

    // （保留原有的代码，尽管使用属性后 m_isGroupFolded 可能不再用于多组折叠，但保留防止头文件报错）
    m_isGroupFolded = false;
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
        comboBox->addItem("平面地面（2d+ground）", 0);
        comboBox->addItem("3D UAV 自由空间（3d+uav）", 1);
        comboBox->addItem("3D Ground 有支撑（3d+ground）", 2);
        comboBox->setCurrentIndex(qMax(0, comboBox->findData(defaultValue)));
        layout->addWidget(comboBox);
    }
    else if(id == "0x26"){
        // 运动学模型：枚举，必须下拉框
        QComboBox *comboBox = new QComboBox();
        comboBox->addItem("omni（全向）", 0);
        comboBox->addItem("differential（差速）", 1);
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


// 槽函数
// 获取当前参数
void Params::on_readButton_clicked()
{
    TcpClient* tcpClient = TcpClient::getInstance();

    if (!tcpClient->isConnected()) {
        QMessageBox::warning(this, "错误", "请先建立TCP连接");
        return;
    }

    ui->optLabel->setText("正在获取所有参数...");
    ui->optLabel->setStyleSheet("color: blue;");

    // 构建统一读取命令: AA 00 03 0A
    QByteArray frame = ProtocolRouter::buildReadParamFrame(0xFF);

    // 发送统一读取命令
    tcpClient->sendData(frame);

    // 清空接收缓冲区
    m_receiveBuffer.clear();
    m_receivedParamCount = 0;

    qDebug() << "发送统一参数读取请求，数据:" << frame.toHex(' ');
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> 用户操作: ");
    emit appendMessage(timestamp + "读取当前参数...");
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

    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> 用户操作: ");
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
// 折叠按钮槽函数
void Params::onFoldButtonClicked()
{
    QPushButton *button = qobject_cast<QPushButton*>(sender());
    if (!button) return;

    // 从按钮属性中获取控制范围和当前状态
    bool isFolded = button->property("folded").toBool();
    int startRow = button->property("startRow").toInt();
    int endRow = button->property("endRow").toInt();

    if (isFolded) {
        // 当前为折叠状态 -> 执行展开
        for (int row = startRow; row <= endRow; ++row) {
            ui->tableWidget->setRowHidden(row, false);
        }
        button->setText("−");
        button->setProperty("folded", false); // 更新状态
    } else {
        // 当前为展开状态 -> 执行折叠
        for (int row = startRow; row <= endRow; ++row) {
            ui->tableWidget->setRowHidden(row, true);
        }
        button->setText("+");
        button->setProperty("folded", true); // 更新状态
    }
}
// 点击恢复默认参数按钮
void Params::on_defaultButton_clicked()
{
    restoreDefaultValues();
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> 用户操作: ");
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
