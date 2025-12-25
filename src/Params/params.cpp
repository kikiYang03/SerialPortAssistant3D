#include "params.h"
#include "ui_params.h"

#include <QHBoxLayout>
#include <QDebug>

const QVector<Parameter> Params::s_parameters = {
    {"0x00", "雷达型号", "0-1", "0=N10, 其他=N10_P", 0},
    {"-", "雷达放置位置", "-", "雷达安装位置相对于机器人中心的位置，即tf树中：base_link->laser_link，坐标系遵循FLU（x为前，Y为左，Z为上）", 0},
    {"0x01", "X坐标", "0-65535", "雷达位置X坐标值，单位厘米", 0},
    {"0x02", "Y坐标", "0-65535", "雷达位置Y坐标值，单位厘米", 0},
    {"0x03", "Z坐标", "0-65535", "雷达位置Z坐标值，单位厘米", 0},
    {"0x04", "Roll角度", "0-360", "雷达滚转角，单位度", 0},
    {"0x05", "Pitch角度", "0-360", "雷达俯仰角，单位度", 0},
    {"0x06", "Yaw角度", "0-360", "雷达偏航角，单位度", 0},
    {"0x10", "当前模式", "0-1", "0=建图模式,1=定位模式，保存地图后可以设置为定位模式，重启模块后则会调用保存的地图进行定位", 0},
    {"0x11", "使能px4输出坐标", "0-1", "1=使能，模块可连接px4飞控用做位置传感器", 1},
    {"0x12", "使能串口输出坐标", "0-1", "1=使能，模块可串口输出坐标", 1},
    };

Params::Params(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Params)
    , m_isGroupFolded(false)
{
    ui->setupUi(this);

    // 初始化协议处理器
    m_protocolHandler = new ProtocolHandler(this);

    // 设置表格属性
    setupTable();

    // 初始化参数
    setupParameters();

    // 连接TCP客户端的数据接收信号
    TcpClient* tcpClient = TcpClient::getInstance();
    connect(tcpClient, &TcpClient::dataReceived, this, &Params::onParameterResponseReceived);
}

Params::~Params()
{
    delete ui;
}

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

void Params::setupParameters()
{
    const QVector<Parameter>& parameters = s_parameters;

    // 设置行数
    ui->tableWidget->setRowCount(parameters.size());

    // 填充表格
    for (int row = 0; row < parameters.size(); ++row) {
        const Parameter &param = parameters[row];

        // 在0x01添加折叠按钮
        if (row == 1) {
            // 创建容器widget确保按钮居中
            QWidget *container = new QWidget();
            QHBoxLayout *layout = new QHBoxLayout(container);
            layout->setContentsMargins(0, 0, 0, 0);
            layout->setAlignment(Qt::AlignCenter);

            QPushButton *foldButton = new QPushButton();
            foldButton->setText("−");
            foldButton->setFixedSize(20, 20);
            foldButton->setProperty("folded", false);
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
        // 移除错误的flags设置，改用正确的方式
        descItem->setToolTip(param.description); // 添加tooltip以便鼠标悬停时显示完整文本
        ui->tableWidget->setItem(row, 5, descItem);

        // 为参数值创建输入控件
        QWidget *valueWidget = createValueWidget(param.id, param.range, param.defaultValue);
        valueWidgets.append(valueWidget);
        ui->tableWidget->setCellWidget(row, 3, valueWidget);
    }

    // 设置行高自适应内容
    ui->tableWidget->resizeRowsToContents();

    // 额外优化：强制紧凑布局
    for (int row = 0; row < ui->tableWidget->rowCount(); ++row) {
        // 设置行高更紧凑
        ui->tableWidget->setRowHeight(row,qMax(ui->tableWidget->rowHeight(row), 25)); // 最小行高25px
    }

    // 默认展开所有行
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
        comboBox->addItem("N10", 0);
        comboBox->addItem("N10_P", 1);
        comboBox->setCurrentIndex(defaultValue);
        layout->addWidget(comboBox);
    } else if(id == "0x10"){
        QComboBox *comboBox = new QComboBox();
        comboBox->addItem("建图模式", 0);
        comboBox->addItem("定位模式", 1);
        comboBox->setCurrentIndex(defaultValue);
        layout->addWidget(comboBox);
    }else if(id == "-")
    {

    }
    else {
        QSpinBox *spinBox = new QSpinBox();
        QStringList rangeParts = range.split("-");
        if (rangeParts.size() == 2) {
            spinBox->setRange(rangeParts[0].toInt(), rangeParts[1].toInt());
        }
        spinBox->setValue(defaultValue);
        layout->addWidget(spinBox);
    }

    return widget;
}


// 写入参数
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

    QByteArray frame = m_protocolHandler->buildParameterFrame(ProtocolCommand::PARAM_WRITE, paramByte, value);
    if (frame.isEmpty()) {
        QMessageBox::warning(this, "错误", "构建写入帧失败");
        return;
    }

    // 使用TCP客户端发送数据
    tcpClient->sendData(frame);

    qDebug() << "发送参数写入请求:" << paramId << "值:" << value << "数据:" << frame.toHex(' ');
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> 用户操作: ");
    emit appendMessage(timestamp + "写入参数");
}
// 解析参数响应帧
void Params::parseParameterResponse(const QByteArray &data)
{
    // 读取响应格式: AA 10 [参数ID] [值高字节] [值低字节] 0A
    // 写入响应格式: AA 11 [参数ID] [值高字节] [值低字节] 0A

    if (data.size() != 6) {
        // qWarning() << "参数响应帧长度错误，期望6字节，实际:" << data.size();
        return;
    }

    quint8 header = static_cast<quint8>(data.at(0));
    quint8 command = static_cast<quint8>(data.at(1));
    quint8 paramId = static_cast<quint8>(data.at(2));
    quint8 valueHigh = static_cast<quint8>(data.at(3));
    quint8 valueLow = static_cast<quint8>(data.at(4));
    quint8 tail = static_cast<quint8>(data.at(5));

    if (header != 0xAA || tail != 0x0A) {
        qWarning() << "参数响应帧头尾错误";
        return;
    }

    if (command != 0x10 && command != 0x11) {
        qWarning() << "未知的命令类型:" << command;
        return;
    }

    // 提取参数值（2字节，大端序）
    int value = (valueHigh << 8) | valueLow;

    QString paramIdStr = QString("0x%1").arg(paramId, 2, 16, QLatin1Char('0')).toUpper();

    qDebug() << "收到参数响应: 命令=" << command << "参数=" << paramIdStr << "值=" << value;

    // 更新界面显示
    for (int row = 0; row < ui->tableWidget->rowCount(); ++row) {
        QTableWidgetItem *idItem = ui->tableWidget->item(row, 1);
        if (idItem && idItem->text().compare(paramIdStr, Qt::CaseInsensitive) == 0) {
            QWidget *widget = valueWidgets[row];
            QLayout *layout = widget->layout();
            if (layout && layout->count() > 0) {
                QWidget *valueControl = layout->itemAt(0)->widget();
                if (QComboBox *comboBox = qobject_cast<QComboBox*>(valueControl)) {
                    // 对于雷达型号参数，直接设置索引
                    if (paramIdStr == "0x00") {
                        comboBox->setCurrentIndex(value);
                    } else {
                        // 其他参数使用组合框的情况
                        comboBox->setCurrentIndex(value);
                    }
                } else if (QSpinBox *spinBox = qobject_cast<QSpinBox*>(valueControl)) {
                    spinBox->setValue(value);
                }
            }
            qDebug() << "更新界面参数:" << paramIdStr << "=" << value;
            break;
        }
    }
}

void Params::onParameterResponseReceived(const QByteArray &data)
{
    // 将数据添加到缓冲区
    m_receiveBuffer.append(data);

    // 使用协议处理器提取完整帧
    QList<QByteArray> frames = m_protocolHandler->extractFramesFromBuffer(m_receiveBuffer);

    // 处理每个完整帧
    for (const QByteArray &frame : frames) {
        processSingleFrame(frame);
    }
}

void Params::processSingleFrame(const QByteArray &frame)
{
    if (!m_protocolHandler->validateFrame(frame)) {
        qWarning() << "无效的帧格式";
        return;
    }

    quint8 command = m_protocolHandler->getCommandType(frame);

    // 处理参数响应帧
    if (command == ProtocolCommand::PARAM_READ) {
        quint8 paramId;
        quint16 value;
        if (m_protocolHandler->parseParameterResponse(frame, paramId, value)) {
            QString paramIdStr = QString("0x%1").arg(paramId, 2, 16, QLatin1Char('0')).toUpper();
            updateParameterValue(paramIdStr, value);
            m_receivedParamCount++;

            // 如果收到了所有预期的参数响应
            if (m_receivedParamCount >= m_expectedParamCount) {
                qDebug() << "所有参数读取完成，共收到" << m_receivedParamCount << "个参数";
                QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> 用户操作: ");
                emit appendMessage(timestamp + QString("参数读取完成"));
                ui->optLabel->setText(QString("参数读取完成"));
                ui->optLabel->setStyleSheet("color: green;");

                m_receivedParamCount = 0;
            }
        }
    }
    // 可以添加其他命令类型的处理
}

void Params::updateParameterValue(const QString &paramId, int value)
{
    for (int row = 0; row < ui->tableWidget->rowCount(); ++row) {
        QTableWidgetItem *idItem = ui->tableWidget->item(row, 1);
        if (idItem && idItem->text().compare(paramId, Qt::CaseInsensitive) == 0) {
            QWidget *widget = valueWidgets[row];
            QLayout *layout = widget->layout();
            if (layout && layout->count() > 0) {
                QWidget *valueControl = layout->itemAt(0)->widget();
                if (QComboBox *comboBox = qobject_cast<QComboBox*>(valueControl)) {
                    // 对于雷达型号参数，直接设置索引
                    if (paramId == "0x00") {
                        comboBox->setCurrentIndex(value);
                    } else {
                        // 其他参数使用组合框的情况
                        comboBox->setCurrentIndex(value);
                    }
                } else if (QSpinBox *spinBox = qobject_cast<QSpinBox*>(valueControl)) {
                    spinBox->setValue(value);
                }
            }
            qDebug() << "更新界面参数:" << paramId << "=" << value;
            break;
        }
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
    QByteArray frame = m_protocolHandler->buildCurParameterFrame();

    // 发送统一读取命令
    tcpClient->sendData(frame);

    // 清空接收缓冲区
    m_receiveBuffer.clear();
    m_receivedParamCount = 0;

    qDebug() << "发送统一参数读取请求，数据:" << frame.toHex(' ');
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> 用户操作: ");
    emit appendMessage(timestamp + "读取当前参数...");
    // QMessageBox::information(this, "读取参数", "正在获取所有参数...");
}

// 写入参数
void Params::on_writeButton_clicked()
{
    TcpClient* tcpClient = TcpClient::getInstance();

    if (!tcpClient->isConnected()) {
        QMessageBox::warning(this, "错误", "请先建立TCP连接");
        return;
    }

    // 从界面读取值并发送写入请求
    int writeCount = 0;
    for (int row = 0; row < valueWidgets.size(); ++row) {
        // 跳过折叠的行
        if (m_isGroupFolded && row >= 1 && row <= 5) continue;

        QTableWidgetItem *idItem = ui->tableWidget->item(row, 1);
        if (!idItem || idItem->text() == "-") continue; // 跳过无参数ID的行

        QWidget *widget = valueWidgets[row];
        QLayout *layout = widget->layout();
        if (layout && layout->count() > 0) {
            QWidget *valueControl = layout->itemAt(0)->widget();
            if (QComboBox *comboBox = qobject_cast<QComboBox*>(valueControl)) {
                int value = comboBox->currentData().toInt();
                // 添加小延迟，避免连续发送
                QTimer::singleShot(writeCount * 100, this, [this, id = idItem->text(), value]() {
                    sendParameterWriteRequest(id, value);
                });
                writeCount++;
            } else if (QSpinBox *spinBox = qobject_cast<QSpinBox*>(valueControl)) {
                int value = spinBox->value();
                // 添加小延迟，避免连续发送
                QTimer::singleShot(writeCount * 100, this, [this, id = idItem->text(), value]() {
                    sendParameterWriteRequest(id, value);
                });
                writeCount++;
            }
        }
    }

    if (writeCount > 0) {
        // QMessageBox::information(this, "写入参数",
        //                          QString("正在将 %1 个参数写入模块...\n写入操作无响应确认。").arg(writeCount));
        QMessageBox::information(this, "写入参数", QString("请断电重启确保参数生效"));
        ui->optLabel->setText("写入参数");
        ui->optLabel->setStyleSheet("color: blue;");
    } else {
        QMessageBox::warning(this, "警告", "没有找到需要写入的参数");
    }
}
// 折叠按钮
void Params::onFoldButtonClicked()
{
    QPushButton *button = qobject_cast<QPushButton*>(sender());
    if (!button) return;

    if (m_isGroupFolded) {
        // 展开第3-8行（索引2-7）
        for (int row = 2; row <= 7; ++row) {
            ui->tableWidget->setRowHidden(row, false);
        }
        button->setText("−");
        m_isGroupFolded = false;
    } else {
        // 折叠第3-8行（索引2-7）
        for (int row = 2; row <= 7; ++row) {
            ui->tableWidget->setRowHidden(row, true);
        }
        button->setText("+");
        m_isGroupFolded = true;
    }
}

void Params::on_defaultButton_clicked()
{
    restoreDefaultValues();
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> 用户操作: ");
    emit appendMessage(timestamp + "恢复参数默认值");
    ui->optLabel->setText("恢复参数默认值");
    ui->optLabel->setStyleSheet("color: blue;");
    QMessageBox::information(this, "恢复默认", "上位机已恢复默认参数，需写入参数");
}

// 写入参数
void Params::updateTableFromWidgets()
{
    // 从界面控件读取值
    for (int row = 0; row < valueWidgets.size(); ++row) {
        QWidget *widget = valueWidgets[row];
        QLayout *layout = widget->layout();
        if (layout && layout->count() > 0) {
            QWidget *valueControl = layout->itemAt(0)->widget();
            if (QComboBox *comboBox = qobject_cast<QComboBox*>(valueControl)) {
                int value = comboBox->currentData().toInt();
                // 处理下拉框值
                qDebug() << "参数" << row << "值:" << value;
            } else if (QSpinBox *spinBox = qobject_cast<QSpinBox*>(valueControl)) {
                int value = spinBox->value();
                // 处理数字输入框值
                qDebug() << "参数" << row << "值:" << value;
            }
        }
    }
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
            comboBox->setCurrentIndex(0);          // 枚举类仍选第 0 项
        } else if (QSpinBox *spinBox = qobject_cast<QSpinBox*>(valueControl)) {
            spinBox->setValue(s_parameters.at(row).defaultValue); // 直接读表
        }
    }
}
