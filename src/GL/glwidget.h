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

class QTimer;

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

    // 参数 0x13「模式」的当前取值：0 = 建图、1 = 定位、-1 = 本会话还没读到过。
    // 由串口层随 AA 10 下发（工单 03，与 0x99 同一条缓存路径）。「未读到过」与「建图」
    // 必须分开处理 —— 没读到过时按建图处理会在图上留下一个说不清来源的标记。
    int moduleMode() const { return moduleMode_; }

    // 把当前的开机点状态推一遍（工单 09）：界面接好信号之后**主动调一次**。
    // 原因是初值：本会话没读到过 `0x13` 时是「不画」，而它不会再变，
    // 而信号只在变化时发 —— 不推这一次，那句「为什么不画」就永远到不了界面。
    void publishOriginMarkerState();


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

    // 「定位状态」显示位（工单 08）：**已定位 / 未定位**。判据**唯一**取自最后一条 `0x02`
    // 的到达时刻（问的就是模块里被 `0x02` 喂的那条边还新不新鲜），三种文案都是**当前状态**、
    // 只在变化时发一次（判定节拍见 .cpp 里那个常量）。
    // **与 `0x0B` 驱动的「重定位 / 规划」状态标签是两件事**：那两个是模块的**内部状态自述**，
    // 这一个说的是「`base_init_map → base_init` 这条边发出来了没有」，二者可以不一致
    // （例如丢包），各占各的显示位、互不覆盖。
    void localizationStatusChanged(const QString &status);

    // 开机点（工单 09）：**空串 = 画着**（`x` / `y` / `yawDeg` 有效），非空 = 这一趟不画，
    // 串本身就是界面上要显出来的那句说明（还没读到过模式 / 建图模式 / 0x02 还没到）。
    // 数值与说明走**同一条**信号：它们是同一个判断的两面，分两条发就迟早会出现
    // 「有数值但说不画」这种自相矛盾的状态。只在真变化时发（0x02 可能是周期帧）。
    void originMarkerChanged(const QString &note, double x, double y, double yawDeg);

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

    // ---------------- 「定位状态」显示位（工单 08） ----------------
    // 判据在模块里（`tf_.isFresh(Edge::BaseInit, now)` + 那条边到没到过），这里只负责
    // 「按点问一次、文案变了就发信号」。**需要定时器**的原因：`0x02` 停发之后不会再有
    // 任何事件来提醒我们它超时了（`0x01` 那条 10 Hz 照旧在发，拿帧到达当节拍只会得到一个
    // 永远停在「已定位」的标签），只能自己按点去问模块。
    QTimer* localizationTimer_ = nullptr;
    QString localizationStatus_;   // 上次发出去的文案（空 = 本次会话还没发过）
    // 重算文案并按需发信号。帧到达时、清图时、以及定时器到点都走这一个入口。
    // 只问模块、不自己存「0x02 到没到过」的标志（标志会跟模块对不上，见上面那段说明）。
    void refreshLocalizationStatus();

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
    // 每帧都推一次会把界面刷个不停（与 07 的扫描说明、08 的定位状态同一条规矩）。
    QString originMarkerNote_;                  // 空串 = 画着（下面三个数值才有效）
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
