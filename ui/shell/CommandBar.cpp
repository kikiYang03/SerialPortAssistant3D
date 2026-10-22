#include "CommandBar.h"

#include "IconButton.h"
#include "SegmentedControl.h"
#include "Theme.h"

#include <QHBoxLayout>
#include <QPainter>

namespace {

constexpr int kBarHeight = 44;   // 与 docs/prototype-v2.html 的 .cmdbar 一致

} // namespace

CommandBar::CommandBar(QWidget *parent)
    : QWidget(parent)
{
    setObjectName(QStringLiteral("commandBar"));
    setFixedHeight(kBarHeight);

    auto *row = new QHBoxLayout(this);
    row->setContentsMargins(Theme::Metric::Gap, 0, Theme::Metric::Gap, 0);
    row->setSpacing(Theme::Metric::GapSm);

    // 左段：三页切换。三段与中央堆栈的索引一一对应（0 连接 / 1 参数 / 2 可视化）
    seg_ = new SegmentedControl(this);
    seg_->addSegment(QStringLiteral("连接设置"));
    seg_->addSegment(QStringLiteral("参数设置"));
    seg_->addSegment(QStringLiteral("可视化"));
    row->addWidget(seg_, 0, Qt::AlignVCenter);
    row->addStretch(1);

    // 右段：读取参数（接参数页既有入口）+ 主按钮
    readButton_ = new IconButton(this);
    readButton_->setIconName(QStringLiteral("download"));
    readButton_->setLabel(QStringLiteral("读取参数"));
    readButton_->setToolTip(QStringLiteral("读取模块的全部参数，不必先切到参数页"));
    readButton_->setAppearance(IconButton::Outline);
    connect(readButton_, &IconButton::clicked, this, &CommandBar::readParamsRequested);
    row->addWidget(readButton_, 0, Qt::AlignVCenter);

    connectButton_ = new IconButton(this);
    connectButton_->setIconName(QStringLiteral("plug"));
    connect(connectButton_, &IconButton::clicked, this, &CommandBar::toggleConnectionRequested);
    row->addWidget(connectButton_, 0, Qt::AlignVCenter);

    // 只有「用户点击」才换页：currentChanged 是给外界看选中态的，不能拿来驱动堆栈，
    // 否则「堆栈改命令栏、命令栏又改堆栈」会绕成环
    connect(seg_, &SegmentedControl::segmentActivated, this, &CommandBar::pageSelected);

    // 初值：未连接。真正的初值由 SerialPort 构造末尾那次 portStateChanged 补上
    setPortConnected(false);
}

void CommandBar::setCurrentPage(int index)
{
    seg_->setCurrentIndex(index);
}

void CommandBar::setPortConnected(bool connected)
{
    // 未连接 = 强调色实底 + 「连接设备」：让「去连设备」成为一眼能看到的下一步；
    // 已连接 = 白底描边 + 「断开设备」：断开不是主动作，不该继续占着全窗最重的那个色块
    connectButton_->setLabel(connected ? QStringLiteral("断开设备")
                                       : QStringLiteral("连接设备"));
    connectButton_->setAppearance(connected ? IconButton::Outline : IconButton::Primary);
    connectButton_->setToolTip(connected
                                   ? QStringLiteral("关闭当前串口")
                                   : QStringLiteral("按串口页选中的端口与参数打开串口"));
}

void CommandBar::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.fillRect(rect(), Theme::color(QStringLiteral("cmd")));

    // 与页面之间那条 1px：外壳常驻，靠这条线把「栏」和「内容」分开
    p.setPen(Theme::color(QStringLiteral("line")));
    p.drawLine(0, height() - 1, width() - 1, height() - 1);
}
