#ifndef POINTSCAPSULE_H
#define POINTSCAPSULE_H

#include <QFont>
#include <QRect>
#include <QSize>
#include <QString>
#include <QWidget>

// 视口右上角的点数胶囊（工单 10）：实时点云 / 地图点云各一个点数。
//
// 只回答一个问题：「地图是不是还在长」。所以**只有点数**——没有帧率、没有渲染耗时，
// 也就没有任何计时埋点（spec 实现决策 20）。点数通过 GLWidget 头文件里新增的
// 两个只读 getter 取（它们只是 int 快照，上传在 GUI 线程里做，不需要锁）。
//
// 它是一个**纯读数**：整块设了 WA_TransparentForMouseEvents，鼠标穿过它落回画布，
// 在它上面按下拖动能照常旋转。也因为它不接受鼠标，不需要 ToolTip、不会吃悬停。
//
// **宽度跟着内容走**（不是钉死一个最宽的列）：点数从 "0" 长到 "24.5 k" 时 chip 该跟着宽，
// 否则启动那一屏会挂着两个中间空出一大截的胶囊，看起来像缺了数据。宽度按 4px 量化，
// 位数不变时纹丝不动 —— 一圈下来只有「跨过一位数」的那一下会动。
class PointsCapsule : public QWidget
{
    Q_OBJECT

public:
    // 与视口右上角的距离（spec 实现决策 18）
    static constexpr int kMargin = 16;

    explicit PointsCapsule(QWidget *parent = nullptr);

    // 点数快照。负数 = 还没有数据（界面显示「-」）。
    // 与上次相同的值直接返回：调用方每 500ms 推一次，不判变化就会一直重绘。
    void setCounts(int realtimePoints, int mapPoints);
    int realtimePoints() const { return realtime_; }
    int mapPoints() const { return map_; }

signals:
    // 数字位数变了（"0" → "24.5 k"），整块宽度跟着变 —— 位置是按尺寸算的，
    // 所以得请外面重新摆一次（Ros3DPage 收到后调 relayoutOverlays()）。
    // 只在**真的**变宽变窄时发，不是每次 setCounts 都发。
    void contentSizeChanged();

public:
    // 纯函数：给定视口尺寸与本控件尺寸，返回本控件在视口坐标系里的矩形。
    //   * 贴右上角、边距 kMargin；
    //   * 视口比控件还窄/矮时夹到 0（不给负坐标）；
    //   * 不居中，所以没有取整抖动的问题 —— 但仍然是纯函数，测试缝里能直接断言。
    static QRect placeRect(const QSize &viewport, const QSize &capsule);

    // 纯函数：点数 → 界面上的短写。
    //   860 → "860"；24500 → "24.5 k"；1240000 → "1.24 M"；负数 → "-"
    // 用短写是因为这一块只有两行宽，六位数在这里会把胶囊撑到视口 1/4。
    static QString formatCount(int points);

    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

protected:
    void paintEvent(QPaintEvent *e) override;

private:
    void buildGeometry();
    void paintChip(QPainter &p, const QRect &chip, const QString &key, const QString &value) const;

    QString keyRealtime_;
    QString keyMap_;
    QString valueRealtime_;   // 已经格式化好的短写（"24.5 k"），只在变化时才重算
    QString valueMap_;

    QFont keyFont_;
    QFont valueFont_;
    int   keyWidth_   = 0;   // 两枚 chip 的字段名等宽（按较长的那个量），看起来才齐
    int   valueWidth_ = 0;   // 数值列宽度：当前两个数里较宽的那个，量化到 4px

    QRect chipRealtime_;
    QRect chipMap_;

    QSize contentSize_;

    int realtime_ = -1;
    int map_      = -1;
};

#endif // POINTSCAPSULE_H
