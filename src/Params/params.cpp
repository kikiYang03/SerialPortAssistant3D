#include "params.h"
#include "ui_params.h"
#include "protocolrouter.h"
#include <QHBoxLayout>
#include <QDebug>
#include <adminmode.h>

const QVector<Parameter> Params::s_parameters = {
    {"0x00", "雷达型号", "0~11", "0=mid360, 1=mid360s, 10=N10, 11=N10_P，默认0", 0},
    {"-", "雷达放置位置", "-", "雷达安装位置相对于机器人中心的位置，即tf树中：base_link->laser_link，坐标系遵循FLU（x为前，Y为左，Z为上）", 0},
    {"0x01", "X坐标", "-100~100", "雷达位置X坐标值，单位厘米", 0},
    {"0x02", "Y坐标", "-100~100", "雷达位置Y坐标值，单位厘米", 0},
    {"0x03", "Z坐标", "-100~100", "雷达位置Z坐标值，单位厘米", 0},
    {"0x04", "Roll角度", "-180~180", "雷达滚转角，单位度", 0},
    {"0x05", "Pitch角度", "-180~180", "雷达俯仰角，单位度", 0},
    {"0x06", "Yaw角度", "-180~180", "雷达偏航角，单位度", 0},
    // {"0x10", "建图or定位模式", "0~1", "0=建图模式，1=定位模式，保存地图后可以设置为定位模式，重启模块后则会调用保存的地图进行定位", 0},
    {"0x11", "使能与PX4飞控通信", "0-1", "1=使能，模块可与PX4飞控通信，作为位置传感器/控制飞控执行路径规划", 1},
    {"0x12", "使能与主控串口通信", "0-1", "1=使能，模块可与主控串口通信，输出坐标/输出路径规划控制点/输入目标点", 1},
    {"-", "路径规划参数", "-", "首次使用应该断开电机供电，查看路径规划结果无误", 0},
    {"0x21", "最大移动速度", "0~1000", "路径规划输出轨迹的最大移动速度，单位cm/s，默认100", 100},
    {"0x22", "最大加速度", "0~1000", "路径规划输出轨迹的最大移动速度，单位cm/s，默认100", 100},
    {"0x23", "最大高度", "0~1000", "路径规划输出轨迹的最大高度，设置虚拟天花板防止轨迹超出该高度，单位cm，默认200", 200},
    {"0x24", "起飞高度", "0~1000", "一键起飞设定的高度，2d版会在这个高度的平面内路径规划，单位cm，默认100", 100},
    {"0x25", "机器人半径", "0~1000", "根据中心点到最远端的距离加多至少5cm，设置为机器人半径作为路径规划，单位cm，默认10", 10},
    {"0x26", "安全距离", "0~1000", "设置机器人中心点到障碍的安全距离，建议=机器人半径+10，优先选择大于该安全半径的路径，单位cm，默认10", 10},
    {"0x27", "过滤半径", "0~1000", "若雷达会扫描到机器人自身结构则需加大该参数，建议=机器人半径，单位cm，默认30", 30},
    // {"0x99", "模块类型", "0~3", "0=3D定位模块，1=3D导航模块，2=2D定位模块，3=2D导航模块", 0},
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

    // 填充表格
    for (int row = 0; row < parameters.size(); ++row) {
        const Parameter &param = parameters[row];

        // 在 row == 1 ("雷达放置位置") 和 row == 10 ("路径规划参数") 添加折叠按钮
        if (row == 1 || row == 10) {
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
            if (row == 1) {
                foldButton->setProperty("startRow", 2);
                foldButton->setProperty("endRow", 7);   // 0x01-0x06
            } else if (row == 10) {
                foldButton->setProperty("startRow", 11);
                foldButton->setProperty("endRow", 17);  // 0x21-0x27
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

    if (id == "0x00") {
        QComboBox *comboBox = new QComboBox();
        comboBox->addItem("mid360", 0);
        comboBox->addItem("mid360s", 1);
        comboBox->addItem("N10", 10);
        comboBox->addItem("N10_P", 11);
        comboBox->setCurrentIndex(defaultValue == 0 ? 0 : (defaultValue == 10 ? 1 : 2));
        layout->addWidget(comboBox);
    } else if(id == "0x10"){
        QComboBox *comboBox = new QComboBox();
        comboBox->addItem("建图模式", 0);
        comboBox->addItem("定位模式", 1);
        comboBox->setCurrentIndex(defaultValue);
        layout->addWidget(comboBox);
    }
    else if(id == "0x11"){
        QComboBox *comboBox = new QComboBox();
        comboBox->addItem("不能", 0);
        comboBox->addItem("使能", 1);
        comboBox->setCurrentIndex(defaultValue);
        layout->addWidget(comboBox);
    }
    else if(id == "0x12"){
        QComboBox *comboBox = new QComboBox();
        comboBox->addItem("不能", 0);
        comboBox->addItem("使能", 1);
        comboBox->setCurrentIndex(defaultValue);
        layout->addWidget(comboBox);
    }
    else if(id == "0x99"){
        QComboBox *comboBox = new QComboBox();
        comboBox->addItem("3D定位模块", 0);
        comboBox->addItem("3D导航模块", 1);
        comboBox->addItem("2D定位模块", 2);
        comboBox->addItem("2D导航模块", 3);
        comboBox->setCurrentIndex(defaultValue);
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


// 构建帧写入参数
void Params::sendParameterWriteRequest(const QString &paramId, int value)
{
    TcpClient* tcpClient = TcpClient::getInstance();

    if (!tcpClient->isConnected()) {
        QMessageBox::warning(this, "错误", "TCP未连接");
        return;
    }

    bool ok;
    quint8 paramByte = paramId.toUShort(&ok, 0);
    if (!ok) {
        QMessageBox::warning(this, "错误", "参数ID转换失败");
        return;
    }

    QByteArray frame =  ProtocolRouter::buildWriteParamFrame(paramByte, value);
    if (frame.isEmpty()) {
        QMessageBox::warning(this, "错误", "构建写入帧失败");
        return;
    }

    // 使用TCP客户端发送数据
    tcpClient->sendData(frame);

    // qDebug() << "发送参数写入请求:" << paramId << "值:" << value << "数据:" << frame.toHex(' ');
}

// 更新参数
void Params::updateParameter(quint8 paramIdRaw, qint16 value)
{
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
void Params::on_writeButton_clicked()
{
    TcpClient* tcpClient = TcpClient::getInstance();

    if (!tcpClient->isConnected()) {
        QMessageBox::warning(this, "错误", "请先建立TCP连接");
        return;
    }

    int writeCount = 0;
    for (int row = 0; row < valueWidgets.size(); ++row) {
        QTableWidgetItem *idItem = ui->tableWidget->item(row, 1);
        if (!idItem || idItem->text() == "-") continue; // 跳过无参数ID的行

        QString paramId = idItem->text();

        QWidget *widget = valueWidgets[row];
        QLayout *layout = widget->layout();
        if (layout && layout->count() > 0) {
            QWidget *valueControl = layout->itemAt(0)->widget();
            // 按照样式按钮类型获取值
            if (QComboBox *comboBox = qobject_cast<QComboBox*>(valueControl)) {
                int value = comboBox->currentData().toInt();
                sendParameterWriteRequest(paramId, value); // 发送写入请求
                writeCount++;
            } else if (QSpinBox *spinBox = qobject_cast<QSpinBox*>(valueControl)) {
                int value = spinBox->value();
                sendParameterWriteRequest(paramId, value); // 发送写入请求
                writeCount++;
            }
        }
    }

    if (writeCount > 0) {
        // 发送完成指令帧 AA 00 04 0A
        QByteArray completionFrame = ProtocolRouter::buildFrame(0x00, QVariantMap{{"sub_cmd", 0x04}});
        tcpClient->sendData(completionFrame);

        QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> 用户操作: ");
        emit appendMessage(timestamp + "写入参数");
        QMessageBox::information(this, "写入参数", "模块已更新参数并关机，请重新上电");
        ui->optLabel->setText("参数写入完成");
        ui->optLabel->setStyleSheet("color: blue;");
    } else {
        QMessageBox::warning(this, "警告", "没有找到需要写入的参数");
    }
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
            comboBox->setCurrentIndex(s_parameters.at(row).defaultValue); // 使用参数表中定义的默认值
        } else if (QSpinBox *spinBox = qobject_cast<QSpinBox*>(valueControl)) {
            spinBox->setValue(s_parameters.at(row).defaultValue); // 直接读表
        }
    }
}
