#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>

#include <QMutex>
#include <atomic>
#include <deque>

#include "tftree.h"
#include "protocol_msg.h"
#include "adminmode.h"
#include <unordered_set>   // ← 让 std::unordered_set 可见
#include <cstdint>         // ← uint64_t

// 轨迹缓存
struct Trail {
    std::deque<Eigen::Vector3f> points;
};

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit GLWidget(QWidget *parent = nullptr);
    ~GLWidget();

    // 换成法线配色
    QVector3D normalToColor(const Eigen::Vector3f& n);
    // 实时切换方案
    enum ColorMode { Height, Normal };
    void setColorMode(ColorMode m) { colorMode_ = m; update(); }

    void setShowRealtimeCloud(bool show);   // 实时点云开关
    void setShowMapCloud(bool show);        // 地图点云开关
    void setShowScan2D(bool show);          // 2D激光雷达开关
    void setShowMap2D(bool show);           // 2D地图开关
    void setMap2DPointSize(float size);     // 设置地图点大小
    void setMap2DStep(int step);            // 设置地图采样步长

    void setZFilterRange(float minZ, float maxZ);
    void setZFilterEnabled(bool enabled);

    void setNavMode(bool enable);

    // 目标点 Z 跟随机器人当前高度（界面「Z = 当前机器人高度」那个勾）。
    //
    // 置位后**三处**目标点 Z 一律取机器人当前高度：进入指点模式时、在画布上按下取点时、
    // 以及微调框回填时（界面那边把 Z 框置灰，喂过来的值不该被信）。另外每收到一帧 `0x01`
    // 位姿还会把已有目标点的 Z 顶上去 —— 「当前高度」就该是当前，不是一个快照。
    //
    // 没收到过任何位姿（`base_init → base_link` 那条边还没到）时**保持原值**：凭空把 Z
    // 归零会让人以为机器人贴在地面上。是否置位只由界面的勾选决定，本类不自作判断。
    void setNavZFollowRobot(bool on);

    // 参数 0x13「模式」的当前取值：0 = 建图、1 = 定位、-1 = 本会话还没读到过。
    // 由串口层随 AA 10 下发（工单 03，与 0x99 同一条缓存路径）。「未读到过」与「建图」
    // 必须分开处理 —— 没读到过时按建图处理会在图上留下一个说不清来源的标记。
    int moduleMode() const { return moduleMode_; }

    // 把当前的开机点状态推一遍（工单 09）：界面接好信号之后**主动调一次**。
    // 原因是初值：状态有可能在界面接上之前就已定型（早到的 `0x13` / `0x02`），而信号只在变化时发 ——
    // 不推这一次，该摆出来的那几个数就永远到不了界面。（`0x13` 没读到过那种「本来就不显示」
    // 不在此列：界面初值正好就是它，推与不推长得一样。）
    void publishOriginMarkerState();

    // ---------------------------------------------------------------- 点数快照（工单 10）
    //
    // 视口右上角那两块「实时点云 / 地图点云」的读数来源。**只读、内联、无锁**：
    // cloudPts_ / mapPts_ 只在 doUploadCloud() / doUploadMap() 里被写，而那两个是
    // **GUI 线程上的槽**（跨线程的数据先经 queued 连接排队，不在这里算），
    // 界面在同一个线程里读两个 int，不可能读到半个值。
    //
    // 之所以做成头文件里的内联 getter 而不是加一对信号：点数每帧都变，发信号等于
    // 把 30Hz 的刷新推给界面，而界面只要「偶尔看一眼趋势」——轮询更便宜也更稳。
    // glwidget.cpp 一行不动（这是本次改造的硬约束）。
    int realtimePointCount() const { return cloudPts_; }   // 实时点云（雷达当前帧）
    int mapPointCount() const { return mapPts_; }          // 地图点云（累积建图结果）


signals:
    void tfInfoChanged(double x, double y, double z,
                       int yaw_deg, int pitch_deg, int roll_deg);   //显示TF相关数据
    // 添加消息显示信号
    void appendMessage(const QString &message);

    void navGoalSet(double x, double y, double z, double yaw);
    void navModeChanged(bool isNavMode);

    // 通知UI更新目标点数值
    void navTargetUpdated(double x, double y, double z, double yaw_deg);

    // 2D 扫描「现在画不出来」的说明（工单 07）：**空串 = 正常**（扫描正按最近一帧 `0x01` 摆着）。
    // 非空时界面上要把这句话显出来——不画是有原因的（`0x01` 陈旧或从没到过），
    // 不说出来就会被当成卡顿。只在文案变化时发一次（扫描按 10 Hz 重发）。
    void scan2DNoteChanged(const QString &note);

    // 开机点（工单 09）：`shown` = 这一趟画不画（true 时 `x` / `y` / `yawDeg` 是有效数值，
    // 此时 `note` 必为空）；false 时 `note` 是界面上要显出来的那句说明（建图模式 / 0x02 还没到），
    // **也可能是空串** —— 那是「本会话还没读到过模式参数（0x13）」：一趟还没开张，没什么可说的，
    // 界面上什么都不显示（数值整块一起收着）。
    // 「画不画 / 数值 / 那句说明」走**同一条**信号：它们是同一个判断的三面，分两条发就迟早会出现
    // 「有数值但说不画」这种自相矛盾的状态。只在真变化时发（0x02 可能是周期帧）。
    void originMarkerChanged(bool shown, const QString &note, double x, double y, double yawDeg);

public slots:
    void onRobotPose(const RobotPoseMsg &);              // 机器人位姿 (base_init → base_link)
    void onRelocalizationPose(const RelocalizationPoseMsg &);  // 重定位结果 (base_init_map → base_init)
    void onCloud(const CloudMsg &);
    void onScan2D(const Scan2DMsg &);        // 2D激光雷达数据 (0x05)
    void onMap2D(const Map2DMsg &);          // 2D地图数据 (0x06)

    // 参数 0x13「模式」（0 = 建图、1 = 定位、-1 = 未读到过）：串口层在收到 AA 10 时缓存，
    // 变化时下发到这里。取值一变，开机点标记的画法与那句说明跟着改（工单 09，见 originState()）。
    void onModuleModeChanged(int mode);

    void clearMap();        // 清理点云地图
    void clearTrail();  // 新增
    void clearCloud();  // 新增
    void setCamera();     // 初始化相机位置
    void saveMapToFile();   // 保存地图


    void addYaw  (int degrees);   // 正数右转，负数左转
    void addPitch(int degrees);   // 正数下俯，负数上仰

    // 接收UI微调框传来的目标点数值
    void updateNavTargetFromUI(double x, double y, double z, double yaw_deg);

    void onGoalPath(const PathMsg &m);

private slots:   // 新增
    void doUploadCloud();   // 在主线程里把 cloudCpu_ 塞进 vboCloud_
    void doUploadMap();     // 在主线程里把 mapInterleavedCpu_ 塞进 vboMap_
    void doUploadScan2D();  // 在主线程里把 scan2DCpu_ 塞进 vboScan2D_
    void doUploadMap2D();   // 在主线程里把 map2DCpu_ 塞进 vboMap2D_

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mouseReleaseEvent(QMouseEvent *e) override;

private:

    // =========== 新协议TF相关 ===========
    // 本层**不再持有任何位姿矩阵**：位姿一律喂给 tf_（工单 04）。
    // 以前这里的 T_map_base_link_ / T_map_laser_ 是同一份数据的第二副本，
    // 且 T_map_laser_（`base_init → laser_link`）正是 v2.6 那条要消灭的旧边。
    //
    // **但不是每个数据都得问它要**：`0x03` 点云与 `0x06` 栅格地图的载荷本来就在图的原点系里，
    // 上位机对它们**一行变换都不做**，直接存载荷坐标（工单 06）。问 tf_ 要的只有 `0x05` 扫描
    // 那一路（它才是 base_link 系、要摆进图里）。
    //
    // 注意没有「是否收到位姿」的布尔量：那件事问 tf_ 即可（edgeTransform 有值 =
    // 这条边到过），多存一个标志就多一处能跟模块对不上的地方。

    // 修改这行：
    Eigen::Vector3d screenToWorld(const QPoint& pos, double targetZ);

    bool isNavMode_ = false;
    bool hasNavTarget_ = false;  // 是否已经生成了箭头
    bool isNavGoalSet_ = false;  // 是否已经最终确认了目标（退出指点模式后）

    // 👇 新增：记录是否正在按住左键拖动设定朝向
    bool isDraggingNavGoal_ = false;

    Eigen::Vector3d navTarget3D_{0, 0, 1}; // 目标坐标，Z默认1m
    double navYaw_ = 0.0;

    // 目标点 Z 是否跟随机器人当前高度（见 setNavZFollowRobot）。默认关：Z 由微调框 / 取点平面定。
    bool navZFollowRobot_ = false;

    // 机器人当前高度（图系 Z）。`false` = 这一趟还没收到过 `0x01`，没有「当前高度」可言。
    // 只从 tf_ 那条边取，不另存位姿副本 —— 与「位姿只往 tf_ 喂」同一条规矩。
    bool robotZ(double* z) const;

    ColorMode colorMode_ = Height;

    // 位姿摆放**唯一**入口（utils/protocol/tftree.h）。两条边都从图的原点系出发：
    //   Edge::BaseLink ← 0x01（随帧更新）    Edge::BaseInit ← 0x02（静态、仅供显示）
    // 规矩：位姿只往这里喂，**要摆的点**只从 placeInMap() 摆进图里，别在下面各处自建矩阵。
    // 今天走 placeInMap() 的只有 `0x05` 扫描这一路 —— `0x03` 点云与 `0x06` 地图本来就在
    // 图系里，不该进这个入口（进了反而会被它「超时不摆放」的判据连累，见 onCloud 那段说明）。
    TfTree tf_;

    // 点云大小
    float cloudPtSize_ = 3.0f;
    float mapPtSize_   = 3.0f;

    // ---------------- CPU 缓存 + dirty 标志 ----------------
    QMutex dataMtx_;

    std::vector<Eigen::Vector3f> cloudCpu_;
    std::vector<Eigen::Vector3f> mapInterleavedCpu_;

    std::atomic_bool cloudDirty_{false};
    std::atomic_bool mapDirty_{false};

    // GPU 对象
    QOpenGLBuffer vboCloud_{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer vboMap_  {QOpenGLBuffer::VertexBuffer};
    QOpenGLVertexArrayObject vaoCloud_, vaoMap_;
    int cloudPts_ = 0, mapPts_ = 0;

    // 2D数据 GPU对象
    std::vector<Eigen::Vector3f> scan2DCpu_;
    std::vector<Eigen::Vector3f> map2DCpu_;       // 位置+颜色交错存储
    QOpenGLBuffer vboScan2D_{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer vboMap2D_{QOpenGLBuffer::VertexBuffer};
    QOpenGLVertexArrayObject vaoScan2D_, vaoMap2D_;
    int scan2DPts_ = 0, map2DPts_ = 0;
    std::atomic_bool scan2DDirty_{false};
    std::atomic_bool map2DDirty_{false};

    // 上一次发出去的「2D 扫描为什么不画」的文案（工单 07）：空 = 正常。
    // 存着只为**变化时才发信号**（扫描 10 Hz 重发，不判变化就会一直刷界面）。
    QString scan2DNote_;
    // 2D 扫描缓存的唯一清理入口。**调用时必须已持有 dataMtx_**（见 .cpp 里的说明）。
    void clearScan2DCacheLocked();
    // 改这句话并按需发信号（同上，只在变化时发）。空串 = 恢复正常。
    void setScan2DNote(const QString &note);

    // ---------------- 开机点标记（工单 09） ----------------
    // 「这一趟开机那一刻，机器人中心落在图里哪儿」。数据来源就是 `0x02` 的原始位置与四元数，
    // 它存在模块里那条静态边 `base_init_map → base_init` 上（工单 04 起就是它）；
    // 本层**不另存一份载荷副本**（第二份迟早跟前一份对不上），也不自建矩阵。
    //
    // 画不画、数值显示什么、界面上那句「为什么不画」——三件事都从**同一个判据**推出来
    // （originState()）。分开各判一次，就会出现「图上画着、界面却说没画」这种自相矛盾的状态，
    // 而这一趟改动要消灭的正是这类静默不一致。
    enum class OriginState {
        UnknownMode,   // 本会话还没读到过 0x13（或读到一个不认识的取值）⇒ 不画
        Mapping,       // 建图模式：开机点恒等于图原点，标记只是压在图原点坐标轴上的噪声 ⇒ 不画
        NoReloc,       // 定位模式，但 `0x02` 还没到过 ⇒ 没有可画的东西
        Ready          // 定位模式 + `0x02` 到过 ⇒ 画
    };
    OriginState originState() const;

    // 上一次发出去的状态，存着只为**变化时才发信号**：`0x02` 若按周期重发，
    // 每帧都推一次会把界面刷个不停（与 07 的扫描说明同一条规矩）。
    bool    originMarkerShown_ = false;         // 上一次发出去的「画不画」（空 `note` 有两种意思，靠它分）
    QString originMarkerNote_;                  // 不画时那句说明；空串 = 没话说（见信号声明）
    double  originMarkerX_ = 0.0;
    double  originMarkerY_ = 0.0;
    double  originMarkerYawDeg_ = 0.0;
    bool    originMarkerPublished_ = false;     // 还没推过 ⇒ 第一次一定发（见 .cpp 里的说明）

    QOpenGLShaderProgram progSimple_;
    QOpenGLShaderProgram progColorCloud_;

    Eigen::Matrix4d proj_, view_;

    // 相机控制
    float distance_ = 30.0f;
    float pitch_ = -45.0f;
    float yaw_ = 180.0f;
    QVector3D center_{0, 0, 0};
    QPoint lastMousePos_;

    Trail trail_;
    static constexpr size_t kMaxTrail = 5000;

    // 地图高度范围（用于颜色）
    float mapMinZ_ = 0.0f;
    float mapMaxZ_ = 0.0f;

    /* 去重用：空间哈希表 */
    std::unordered_set<uint64_t> mapVoxelSet_;

    // 工具函数
    static QMatrix4x4 toQMatrix(const Eigen::Matrix4d &m);
    void drawAxis(const Eigen::Matrix4d &T, float len = 5.f);
    void drawGrid(const Eigen::Matrix4d &T, int cells = 20, float step = 1.f);
    // 颜色由调用方给：同一个几何体既画机器人箭头（默认红），也画开机点标记（蓝，工单 09）——
    // 两个箭头会同时在图上，光看形状分不出哪个是机器人。
    void drawSolidArrow(const Eigen::Matrix4d &T, float len, float radius = 0.08f,
                        const QVector3D &color = QVector3D(1.0f, 0.0f, 0.0f));
    QVector3D heightToColor(float z, float minZ, float maxZ);

    // 鼠标事件
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;

    QOpenGLBuffer vboAxis_;
    QOpenGLVertexArrayObject vaoAxis_;

    QOpenGLBuffer vboGrid_;
    QOpenGLVertexArrayObject vaoGrid_;

    QOpenGLBuffer vboTrail_;
    QOpenGLVertexArrayObject vaoTrail_;

    QOpenGLBuffer vboArrow_;
    QOpenGLVertexArrayObject vaoArrow_;

    void createArrowGeometry();
    void createTrailVAO();
    void createAxisGeometry();
    void createGridGeometry();

    bool glReady_ = false;

    bool showRealtimeCloud_ = true;         // 默认显示
    bool showMapCloud_      = true;

    // 2D数据显示控制
    bool showScan2D_ = true;
    bool showMap2D_  = true;
    float map2DPointSize_ = 6.0f;  // 2D地图点大小
    int map2DStep_ = 1;             // 2D地图采样步长 (1=全部, 2=隔1取1, 3=隔2取1)


    // Z轴范围控制
    float zMinFilter_ = -10.0f;   // 默认最小Z值
    float zMaxFilter_ =  10.0f;   // 默认最大Z值
    bool  enableZFilter_ = false; // 是否启用Z轴过滤

    // 添加新的过滤点云渲染方法
    void drawFilteredPoints(QOpenGLShaderProgram &program,
                            QOpenGLVertexArrayObject &vao,
                            QOpenGLBuffer &vbo,
                            const std::vector<Eigen::Vector3f>& points,
                            int pointCount,
                            float pointSize,
                            const QVector3D& color = QVector3D(1,1,1),
                            bool useColorProgram = false);

    std::vector<Eigen::Vector3f> optimalPathPts_; // 存储规划路径点
    QOpenGLBuffer vboOptimalPath_;
    QOpenGLVertexArrayObject vaoOptimalPath_;
    bool hasOptimalPath_ = false;

    // 2D模式标志（通过接收2D激光雷达数据判断）
    bool is2DMode_ = false;

    // 参数 0x13「模式」：0 = 建图、1 = 定位、-1 = 本会话还没读到过。
    // 与上面那些渲染开关无关，它描述的是模块自己处于哪种模式，见 moduleMode()
    int moduleMode_ = -1;
};

#endif // GLWIDGET_H
