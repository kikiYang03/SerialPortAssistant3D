// 地图分块拼装 + CRC-32 校验的独立测试（工单 09）。
//
// 这是本轮唯一的新测试缝：被测的 maparchive 模块无 I/O、无链路、无界面，
// 所以这里不搭任何框架，一个 main() 喂分块、断结论就够。
// 不属于 luaPort1.pro 的构建目标，单独用 tests/test_maparchive.pro 编。
//
// 覆盖：顺序 / 乱序到达、缺块、重复块、长度不符、CRC 不符、空包，
// 外加序号越界、各块元信息打架、CRC 文本读不出来的兜底分支。

#include <cstdio>

#include "maparchive.h"

using MapArchive::Status;

static int g_checked = 0;
static int g_failed  = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        ++g_checked;                                                       \
        if (!(cond)) {                                                     \
            ++g_failed;                                                    \
            printf("    FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);      \
        }                                                                  \
    } while (0)

// 结论比对：对不上就把两个结论的名字打出来，别只报行号。
#define CHECK_STATUS(res, expected)                                        \
    do {                                                                   \
        ++g_checked;                                                       \
        if ((res).status != (expected)) {                                  \
            ++g_failed;                                                    \
            printf("    FAIL %s:%d  status=%s, 期望 %s\n", __FILE__,        \
                   __LINE__, MapArchive::statusText((res).status),         \
                   MapArchive::statusText(expected));                      \
        }                                                                  \
    } while (0)

/* ---------- 造数据 ---------- */

// 可重现的伪随机字节：同一 (n, seed) 永远得到同一串，便于比对拼装结果。
static QByteArray makeBytes(int n, int seed)
{
    QByteArray b;
    b.resize(n);
    for (int i = 0; i < n; ++i)
        b[i] = static_cast<char>((i * 31 + seed * 7) & 0xFF);
    return b;
}

// 线上形状：定宽十六进制文本，带 0x 前缀。
static QString hexCrc(quint32 v)
{
    QString s = QString::number(v, 16);
    while (s.size() < 8)
        s.prepend('0');
    return QStringLiteral("0x") + s;
}

// 把整包按 chunkSize 切块，字段全按线上形状填对（seq 从 1 起、total、bytes、crc32）。
// 这里用被测的 crc32() 算整包 CRC——自引用没关系，crc32 自身另有已知向量钉住（testCrc32）。
static QVector<MapChunkMsg> splitPackage(const QByteArray &pkg,
                                         const QString &fileName,
                                         int chunkSize)
{
    QVector<MapChunkMsg> chunks;
    const int total = (pkg.size() + chunkSize - 1) / chunkSize;
    const quint32 crc = MapArchive::crc32(pkg);

    for (int i = 0; i < total; ++i) {
        MapChunkMsg m;
        m.topic = QStringLiteral("map_archive");
        m.name  = fileName;
        m.seq   = i + 1;
        m.total = total;
        m.bytes = pkg.size();
        m.crc32 = hexCrc(crc);
        m.data  = pkg.mid(i * chunkSize, chunkSize);
        chunks.append(m);
    }
    return chunks;
}

/* ---------- 用例 ---------- */

// CRC-32 是自实现的，所以必须拿外部已知值钉住：这两个向量是 zlib 的标准检验值。
// 少了这条，「拼装全对但 CRC 算法本身错」会一路绿到底。
static bool testCrc32()
{
    g_failed = 0;
    CHECK(MapArchive::crc32(QByteArray()) == 0x00000000u);
    CHECK(MapArchive::crc32(QByteArray("123456789")) == 0xCBF43926u);
    CHECK(MapArchive::crc32(QByteArray("The quick brown fox jumps over the lazy dog"))
          == 0x414FA339u);
    CHECK(MapArchive::crc32(makeBytes(4096, 3)) == MapArchive::crc32(makeBytes(4096, 3)));
    return g_failed == 0;
}

// 拼装按 seq 走，与到达顺序无关：同一组块，顺序喂与乱序喂得到同一个包。
static bool testOrderIndependence()
{
    g_failed = 0;
    const QByteArray pkg = makeBytes(700, 1);
    const QVector<MapChunkMsg> ordered = splitPackage(pkg, QStringLiteral("map_2d.zip"), 64);
    CHECK(ordered.size() == 11);

    const MapArchive::Result inOrder = MapArchive::assembleMapArchive(ordered);
    CHECK_STATUS(inOrder, Status::Ok);
    CHECK(inOrder.ok());
    CHECK(inOrder.package == pkg);

    // 倒着到、以及打乱到
    QVector<MapChunkMsg> reversed;
    for (int i = ordered.size() - 1; i >= 0; --i)
        reversed.append(ordered.at(i));

    QVector<MapChunkMsg> shuffled = ordered;
    static const int order[] = {5, 0, 9, 2, 10, 7, 1, 3, 8, 4, 6};
    shuffled.clear();
    for (int i = 0; i < 11; ++i)
        shuffled.append(ordered.at(order[i]));

    const MapArchive::Result back = MapArchive::assembleMapArchive(reversed);
    const MapArchive::Result mix  = MapArchive::assembleMapArchive(shuffled);
    CHECK_STATUS(back, Status::Ok);
    CHECK_STATUS(mix, Status::Ok);
    CHECK(back.package == pkg);
    CHECK(mix.package == pkg);
    return g_failed == 0;
}

// 单块包：total=1 也要能过（两块以上的逻辑不能把 1 当特例漏掉）。
static bool testSingleChunk()
{
    g_failed = 0;
    const QByteArray pkg = makeBytes(37, 2);
    const MapArchive::Result r =
        MapArchive::assembleMapArchive(splitPackage(pkg, QStringLiteral("map_3d.zip"), 4096));
    CHECK_STATUS(r, Status::Ok);
    CHECK(r.package == pkg);
    return g_failed == 0;
}

// 缺块：中间少一块。收全之前不许给出字节。
static bool testMissingChunk()
{
    g_failed = 0;
    const QByteArray pkg = makeBytes(700, 4);
    QVector<MapChunkMsg> chunks = splitPackage(pkg, QStringLiteral("map_2d.zip"), 64);
    chunks.removeAt(3);                       // 丢掉 seq=4

    const MapArchive::Result r = MapArchive::assembleMapArchive(chunks);
    CHECK_STATUS(r, Status::MissingChunks);
    CHECK(!r.ok());
    CHECK(r.package.isEmpty());               // 半截包不许流出去
    CHECK(!r.detail.isEmpty());
    return g_failed == 0;
}

// 缺的是最后一块：块数少了，但序号没断——同样要判缺块，不能因为「前面都连号」就放过。
static bool testMissingLastChunk()
{
    g_failed = 0;
    const QByteArray pkg = makeBytes(700, 5);
    QVector<MapChunkMsg> chunks = splitPackage(pkg, QStringLiteral("map_2d.zip"), 64);
    chunks.removeLast();

    const MapArchive::Result r = MapArchive::assembleMapArchive(chunks);
    CHECK_STATUS(r, Status::MissingChunks);
    CHECK(r.package.isEmpty());
    return g_failed == 0;
}

// 重复块：同一 seq 到两次。
static bool testDuplicateChunk()
{
    g_failed = 0;
    const QByteArray pkg = makeBytes(700, 6);
    QVector<MapChunkMsg> chunks = splitPackage(pkg, QStringLiteral("map_2d.zip"), 64);
    const MapChunkMsg again = chunks.at(2);   // 先拷出来：往自己身上 append 一个内部引用，
    chunks.append(again);                     // 在 realloc 时是悬垂的（Qt 与 shim 都不该赌）

    const MapArchive::Result r = MapArchive::assembleMapArchive(chunks);
    CHECK_STATUS(r, Status::DuplicateSeq);
    CHECK(r.package.isEmpty());

    // 重复块把「块数」补齐成 total 的情况（丢一块 + 一块重复），必须先报重复：
    // 否则块数对得上，缺块会被长度校验顶掉，报出来的原因就指错了地方。
    QVector<MapChunkMsg> swapped;
    for (int i = 0; i < chunks.size() - 1; ++i) {
        if (i != 1)
            swapped.append(chunks.at(i));
    }
    swapped.append(chunks.at(2));
    CHECK_STATUS(MapArchive::assembleMapArchive(swapped), Status::DuplicateSeq);
    return g_failed == 0;
}

// 长度不符：块齐了，但拼出来的长度和整包声明的 bytes 对不上。
static bool testLengthMismatch()
{
    g_failed = 0;
    const QByteArray pkg = makeBytes(700, 7);
    QVector<MapChunkMsg> chunks = splitPackage(pkg, QStringLiteral("map_2d.zip"), 64);
    for (int i = 0; i < chunks.size(); ++i)
        chunks[i].bytes = pkg.size() + 1;     // 少一个字节也照样是「不符」的反向情形

    const MapArchive::Result r = MapArchive::assembleMapArchive(chunks);
    CHECK_STATUS(r, Status::LengthMismatch);
    CHECK(r.package.isEmpty());
    CHECK(!r.detail.isEmpty());
    return g_failed == 0;
}

// CRC 不符：长度分毫不差，内容是坏的。这条挡的正是「长度过得去但包坏了」。
static bool testCrcMismatch()
{
    g_failed = 0;
    const QByteArray pkg = makeBytes(700, 8);
    QVector<MapChunkMsg> chunks = splitPackage(pkg, QStringLiteral("map_2d.zip"), 64);

    QByteArray corrupt = chunks.at(5).data;   // 翻其中一块里的一个字节，长度不变
    corrupt[0] = static_cast<char>(corrupt[0] ^ 0x01);
    chunks[5].data = corrupt;

    const MapArchive::Result r = MapArchive::assembleMapArchive(chunks);
    CHECK_STATUS(r, Status::CrcMismatch);
    CHECK(!r.ok());
    CHECK(r.package.isEmpty());               // 坏包不返回，落盘那步就没有拿错东西的机会
    CHECK(!r.detail.isEmpty());
    return g_failed == 0;
}

// 空包：一块都没收到（模块下传失败时一帧都不发，上位机按超时判失败，此处的空序列是它的下游）。
static bool testEmpty()
{
    g_failed = 0;
    const MapArchive::Result r = MapArchive::assembleMapArchive(QVector<MapChunkMsg>());
    CHECK_STATUS(r, Status::Empty);
    CHECK(r.package.isEmpty());

    // total 声明为 0：同样没有可拼的东西。
    QVector<MapChunkMsg> chunks;
    MapChunkMsg m;
    m.topic = QStringLiteral("map_archive");
    m.name  = QStringLiteral("map_2d.zip");
    m.seq   = 1;
    m.total = 0;
    m.bytes = 0;
    m.crc32 = QStringLiteral("0x00000000");
    chunks.append(m);
    CHECK_STATUS(MapArchive::assembleMapArchive(chunks), Status::Empty);
    return g_failed == 0;
}

// 序号越界：seq 不在 1..total 内。0 与「没发过」分不开，协议从 1 起——这里替协议兜住。
static bool testBadSeq()
{
    g_failed = 0;
    const QByteArray pkg = makeBytes(200, 9);

    QVector<MapChunkMsg> zero = splitPackage(pkg, QStringLiteral("map_2d.zip"), 64);
    zero[0].seq = 0;
    CHECK_STATUS(MapArchive::assembleMapArchive(zero), Status::BadSeq);

    QVector<MapChunkMsg> tooBig = splitPackage(pkg, QStringLiteral("map_2d.zip"), 64);
    tooBig[1].seq = tooBig.at(1).total + 1;
    CHECK_STATUS(MapArchive::assembleMapArchive(tooBig), Status::BadSeq);
    return g_failed == 0;
}

// 各块自报的包信息打架：这几块根本不是同一个包（或混进了上一包的块）。
static bool testInconsistentMeta()
{
    g_failed = 0;
    const QByteArray pkg = makeBytes(700, 10);

    QVector<MapChunkMsg> diffTotal = splitPackage(pkg, QStringLiteral("map_2d.zip"), 64);
    diffTotal[4].total = diffTotal.at(4).total + 1;
    CHECK_STATUS(MapArchive::assembleMapArchive(diffTotal), Status::InconsistentMeta);

    QVector<MapChunkMsg> diffBytes = splitPackage(pkg, QStringLiteral("map_2d.zip"), 64);
    diffBytes[4].bytes = 12345;
    CHECK_STATUS(MapArchive::assembleMapArchive(diffBytes), Status::InconsistentMeta);

    QVector<MapChunkMsg> diffCrc = splitPackage(pkg, QStringLiteral("map_2d.zip"), 64);
    diffCrc[4].crc32 = QStringLiteral("0xdeadbeef");
    CHECK_STATUS(MapArchive::assembleMapArchive(diffCrc), Status::InconsistentMeta);
    return g_failed == 0;
}

// CRC 文本读不出来：声明的东西没法核，不能当通过。
static bool testUnparseableCrc()
{
    g_failed = 0;
    const QByteArray pkg = makeBytes(200, 11);

    QVector<MapChunkMsg> chunks = splitPackage(pkg, QStringLiteral("map_2d.zip"), 64);
    for (int i = 0; i < chunks.size(); ++i)
        chunks[i].crc32 = QStringLiteral("zzz");
    CHECK_STATUS(MapArchive::assembleMapArchive(chunks), Status::CrcMismatch);

    // 大小写与 0x 前缀的写法差异不该被当成「不符」：同一块板子换个写法还是同一个值。
    QVector<MapChunkMsg> upper = splitPackage(pkg, QStringLiteral("map_2d.zip"), 64);
    for (int i = 0; i < upper.size(); ++i) {
        QString s = upper.at(i).crc32.mid(2).toUpper();
        upper[i].crc32 = i % 2 == 0 ? QStringLiteral("0x") + s : s;  // 一半带前缀、一半不带
    }
    CHECK_STATUS(MapArchive::assembleMapArchive(upper), Status::Ok);
    return g_failed == 0;
}

/* ---------- 跑 ---------- */

int main()
{
    struct Case {
        const char *name;
        bool (*fn)();
    };
    const Case cases[] = {
        {"crc32 known vectors",        testCrc32},
        {"assembly order-independent", testOrderIndependence},
        {"single chunk package",       testSingleChunk},
        {"missing chunk",              testMissingChunk},
        {"missing last chunk",         testMissingLastChunk},
        {"duplicate chunk",            testDuplicateChunk},
        {"length mismatch",            testLengthMismatch},
        {"crc mismatch",               testCrcMismatch},
        {"empty package",              testEmpty},
        {"seq out of range",           testBadSeq},
        {"inconsistent chunk meta",    testInconsistentMeta},
        {"unparseable crc text",       testUnparseableCrc},
    };

    printf("maparchive: %d cases\n", static_cast<int>(sizeof(cases) / sizeof(cases[0])));
    int failed = 0;
    for (const Case &c : cases) {
        const bool ok = c.fn();
        printf("  %-4s %s\n", ok ? "ok" : "FAIL", c.name);
        if (!ok)
            ++failed;
    }

    printf("%d cases, %d failed\n", static_cast<int>(sizeof(cases) / sizeof(cases[0])), failed);
    return failed == 0 ? 0 : 1;
}
