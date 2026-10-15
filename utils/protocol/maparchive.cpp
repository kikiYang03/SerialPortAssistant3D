#include "maparchive.h"

namespace MapArchive {

namespace {

/* CRC-32 查表：表在首次用到时现建一次。函数内 static 的初始化自 C++11 起是线程安全的。 */
struct Crc32Table {
    quint32 v[256];
    Crc32Table()
    {
        for (int i = 0; i < 256; ++i) {
            quint32 c = static_cast<quint32>(i);
            for (int k = 0; k < 8; ++k)
                c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            v[i] = c;
        }
    }
};

const Crc32Table &crc32Table()
{
    static const Crc32Table t;
    return t;
}

/* "0x1234abcd" / "1234ABCD" 都收（大小写不敏感，0x 前缀可有可无）；读不出来返回 false。 */
bool parseDeclaredCrc(const QString &text, quint32 *out)
{
    QString s = text.trimmed();
    if (s.size() > 2 && s.at(0) == '0' && (s.at(1) == 'x' || s.at(1) == 'X'))
        s = s.mid(2);
    if (s.isEmpty())
        return false;

    bool ok = false;
    const uint parsed = s.toUInt(&ok, 16);
    if (!ok)
        return false;

    *out = static_cast<quint32>(parsed);
    return true;
}

/* 0x1234abcd 这种定宽写法：界面上两个 CRC 值对齐才看得出差别。 */
QString hex32(quint32 v)
{
    QString s = QString::number(v, 16);
    while (s.size() < 8)
        s.prepend('0');
    return QStringLiteral("0x") + s;
}

/* 缺的序号列出来，但最多列 8 个——整包几十块时把全表糊到弹窗上没人看。 */
QString joinSeqs(const QVector<int> &seqs)
{
    const int shown = seqs.size() < 8 ? seqs.size() : 8;
    QString s;
    for (int i = 0; i < shown; ++i) {
        if (i > 0)
            s += QStringLiteral("、");
        s += QString::number(seqs.at(i));
    }
    if (seqs.size() > shown)
        s += QStringLiteral(" 等 %1 块").arg(seqs.size());
    return s;
}

} // namespace

quint32 crc32(const QByteArray &data)
{
    const Crc32Table &t = crc32Table();
    quint32 c = 0xFFFFFFFFu;
    const uchar *p = reinterpret_cast<const uchar *>(data.constData());
    for (int i = 0; i < data.size(); ++i)
        c = t.v[(c ^ p[i]) & 0xFFu] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

Result assembleMapArchive(const QVector<MapChunkMsg> &chunks)
{
    Result r;

    if (chunks.isEmpty()) {
        r.status = Status::Empty;
        r.detail = QStringLiteral("一块都没收到");
        return r;
    }

    const MapChunkMsg &first = chunks.first();
    const int total = first.total;
    const int bytes = first.bytes;

    if (total < 1) {
        r.status = Status::Empty;
        r.detail = QStringLiteral("模块声明总块数为 %1").arg(total);
        return r;
    }

    quint32 declaredCrc = 0;
    if (!parseDeclaredCrc(first.crc32, &declaredCrc)) {
        r.status = Status::CrcMismatch;
        r.detail = QStringLiteral("整包 CRC-32 读不出来：%1").arg(first.crc32);
        return r;
    }

    // 1) 每块自身的形状：序号落在 1..total，且自报的「整包」信息与首块一致。
    //    这几项里任何一项对不上，说明这些块根本不是同一个包（或混进了别的包的块）。
    for (const MapChunkMsg &c : chunks) {
        if (c.seq < 1 || c.seq > total) {
            r.status = Status::BadSeq;
            r.detail = QStringLiteral("块序号 %1 不在 1~%2 内").arg(c.seq).arg(total);
            return r;
        }

        quint32 chunkCrc = 0;
        if (c.total != total || c.bytes != bytes
            || !parseDeclaredCrc(c.crc32, &chunkCrc) || chunkCrc != declaredCrc) {
            r.status = Status::InconsistentMeta;
            r.detail = QStringLiteral("第 %1 块自报的包信息与其它块不一致").arg(c.seq);
            return r;
        }
    }

    // 2) 按 seq 归位。位置被占两次 = 重复块；空位 = 缺块。
    QVector<const MapChunkMsg *> slot(total, nullptr);
    for (const MapChunkMsg &c : chunks) {
        if (slot.at(c.seq - 1) != nullptr) {
            r.status = Status::DuplicateSeq;
            r.detail = QStringLiteral("第 %1 块收到了两次").arg(c.seq);
            return r;
        }
        slot[c.seq - 1] = &c;
    }

    QVector<int> missing;
    for (int i = 0; i < total; ++i) {
        if (slot.at(i) == nullptr)
            missing.append(i + 1);
    }
    if (!missing.isEmpty()) {
        r.status = Status::MissingChunks;
        r.detail = QStringLiteral("收到 %1/%2 块，缺 seq %3")
                       .arg(chunks.size()).arg(total).arg(joinSeqs(missing));
        return r;
    }

    // 3) 顺序拼装：走的是 seq 槽位，不是到达顺序。
    int sum = 0;
    for (const MapChunkMsg *c : slot)
        sum += c->data.size();

    QByteArray package;
    package.reserve(sum);
    for (const MapChunkMsg *c : slot)
        package.append(c->data);

    // 4) 长度：挡住丢块留下的短包。
    if (package.size() != bytes) {
        r.status = Status::LengthMismatch;
        r.detail = QStringLiteral("拼出 %1 字节，整包声明 %2 字节")
                       .arg(package.size()).arg(bytes);
        return r;
    }

    // 5) 内容：长度对得上也照样可能是坏包，这一条才是真正在核内容。
    const quint32 actual = crc32(package);
    if (actual != declaredCrc) {
        r.status = Status::CrcMismatch;
        r.detail = QStringLiteral("CRC-32 不符：算出 %1，声明 %2")
                       .arg(hex32(actual)).arg(hex32(declaredCrc));
        return r;
    }

    r.status = Status::Ok;
    r.package = package;
    r.detail = QStringLiteral("整包 %1 字节，CRC-32 %2 校验通过")
                   .arg(package.size()).arg(hex32(actual));
    return r;
}

const char *statusText(Status s)
{
    switch (s) {
    case Status::Ok:               return "Ok";
    case Status::Empty:            return "Empty";
    case Status::BadSeq:           return "BadSeq";
    case Status::DuplicateSeq:     return "DuplicateSeq";
    case Status::InconsistentMeta: return "InconsistentMeta";
    case Status::MissingChunks:    return "MissingChunks";
    case Status::LengthMismatch:   return "LengthMismatch";
    case Status::CrcMismatch:      return "CrcMismatch";
    }
    return "Unknown";
}

} // namespace MapArchive
