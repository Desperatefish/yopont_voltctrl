#include "Logger.h"
#include <QCoreApplication>
#include <QDir>
#include <QFileInfoList>
#include <QDateTime>
#include <QDebug>
#include <QTextStream>

Logger::Logger()
    : minLevel(Info),
      maxFileSize(1 * 1024 * 1024) // 5MB
{
    logDir = QCoreApplication::applicationDirPath() + "/logs"; // 默认日志目录
}

Logger::~Logger()
{
    if (logFile.isOpen())
        logFile.close();
}

Logger& Logger::instance()
{
    static Logger logger;
    return logger;
}

void Logger::setLogDirectory(const QString& dir)
{
    QMutexLocker locker(&mutex);
    if (dir.isEmpty() || dir == logDir)
        return;

    logDir = dir;
    rotateLogFile(true); // 强制切换日志文件
}

void Logger::setMinLevel(Level level)
{
    QMutexLocker locker(&mutex);
    minLevel = level;
}

void Logger::log(Level level, const QString &message)
{
    if (level < minLevel)
        return;

    QMutexLocker locker(&mutex);

    rotateLogFile();  // 每次写入前检查是否需要切换文件

    QString line = QString("[%1] [%2] %3")
            .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz"))
            .arg(levelToString(level))
            .arg(message);

    writeToFile(line);

    // 同时输出到控制台
    switch (level)
    {
        case Info:  qInfo().noquote() << line; break;
        case Warn:  qWarning().noquote() << line; break;
        case Error: qCritical().noquote() << line; break;
    }
}

QString Logger::levelToString(Level level)
{
    switch (level)
    {
        case Info:  return "INFO";
        case Warn:  return "WARN";
        case Error: return "ERROR";
    }
    return "UNKNOWN";
}

void Logger::rotateLogFile(bool force)
{
    QDir().mkpath(logDir);

    QString baseFilePath = logDir + "/modbus.log";

    // 如果文件已打开并且大小没超过5MB就不切换
    if (!force && logFile.isOpen() && logFile.size() < maxFileSize)
        return;

    // 如果已有日志文件且大小超限 → 重命名旧文件
    if (logFile.isOpen())
    {
        logFile.close();
        QString rotatedName = QString("%1/modbus_%2.log")
                                .arg(logDir)
                                .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
        QFile::rename(baseFilePath, rotatedName);
    }

    // 打开新的日志文件
    logFile.setFileName(baseFilePath);
    if (!logFile.open(QIODevice::Append | QIODevice::Text))
    {
        qWarning() << "无法打开日志文件:" << baseFilePath;
        return;
    }

    stream.setDevice(&logFile);
    stream.setCodec("UTF-8"); // 确保UTF-8

    checkLogDirSize();

    qInfo() << "日志文件切换到:" << baseFilePath;
}

void Logger::writeToFile(const QString &line)
{
    if (logFile.isOpen())
    {
        stream << line << "\n";
        stream.flush();
    }
}

void Logger::checkLogDirSize()
{
    // 遍历日志目录
    QDir dir(logDir);
    QFileInfoList fileList = dir.entryInfoList(QStringList() << "*.log", QDir::Files, QDir::Time);

    quint64 totalSize = 0;
    for (const QFileInfo &fi : fileList)
        totalSize += fi.size();

    const quint64 maxTotalSize = 1024ull * 1024 * 1024; // 1GB

    // 超过1GB则删除最旧文件
    int idx = fileList.size() - 1;
    while (totalSize > maxTotalSize && idx >= 0)
    {
        quint64 size = fileList[idx].size();
        QFile::remove(fileList[idx].absoluteFilePath());
        totalSize -= size;
        idx--;
    }
}
