#ifndef INI_FILE_H
#define INI_FILE_H

#include <QString>
#include <QSettings>

class INI_File
{
public:
    INI_File();
    virtual ~INI_File();

    void CreateFile(QString qstrFilePath, QString qstrFileName);
    QString GetBaudRate();
    QString GetDataBit();
    QString GetStopBit();
    QString GetCheckBit();
private:
    QString m_qstrFileName;
    QSettings *m_psetting;
};

#endif // INI_FILE_H
