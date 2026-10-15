#ifndef MAPARCHIVE_H
#define MAPARCHIVE_H

#include <QByteArray>
#include <QString>
#include <QVector>

#include "protocol_msg.h"

// 地图下传分块（0x0C）的**拼装与校验**。
//
// 这一层是纯函数式的：不认识链路、不认识界面、不做任何 I/O，输入是「分块序列」，
// 输出是「结论 + 整包字节」。调用方（工单 10 的下载流程）收一块攒一块，收满之后再问它。
//
// 两条校验各挡一种失效：按 `bytes` 核长度挡「丢块」，按 `crc32` 核内容挡「包本身坏了」。
// 结论不是 Ok 时**不返回字节**——坏包不能伪装成好包流到落盘那一步。
namespace MapArchive {

// 拼装与校验的结论。除 Ok 外，一律不落盘。
enum class Status {
    Ok,               // 收全、长度相符、CRC 相符
    Empty,            // 一块都没收到（或模块声明的总块数 <= 0）
    BadSeq,           // 块序号越界：seq 不在 1..total 内
    DuplicateSeq,     // 同一个 seq 到了两次
    InconsistentMeta, // 各块自报的「整包」信息（total / bytes / crc32）对不上
    MissingChunks,    // 序号不连续：中间有块没到
    LengthMismatch,   // 拼出来的长度与整包声明的 bytes 不符（多为丢块）
    CrcMismatch,      // 长度对了，但内容不是那个包；CRC 文本读不出来也归这里
};

struct Result {
    Status status = Status::Empty;
    QByteArray package; // 仅 Ok 时非空；其余结论一律为空
    QString detail;     // 面向日志/弹窗的一句话：差在哪一块、差多少

    bool ok() const { return status == Status::Ok; }
};

// zlib 的 CRC-32：poly 0xEDB88320（反转形式），初值与终值均取反——模块侧用的就是这条。
// 自实现，不引第三方库（Qt 只公开 CRC-16 的 qChecksum）。
quint32 crc32(const QByteArray &data);

// 按 seq 拼装并校验，**与到达顺序无关**：乱序到达与顺序到达得到同一个结果。
Result assembleMapArchive(const QVector<MapChunkMsg> &chunks);

// 结论的短名，供日志用。
const char *statusText(Status s);

} // namespace MapArchive

#endif // MAPARCHIVE_H
