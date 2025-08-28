#ifndef LOGGER_H
#define LOGGER_H

#include <QString>
#include <QFile>
#include <QTextStream>
#include <QMutex>

class Logger
{
public:
    enum Level {
        Info = 0,
        Warn,
        Error
    };

    static Logger& instance();

    // 设置日志目录，立即切换日志文件
    void setLogDirectory(const QString& dir);

    // 设置日志最小输出级别
    void setMinLevel(Level level);

    // 写日志
    void log(Level level, const QString &message);

private:
    Logger();
    ~Logger();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void rotateLogFile(bool force = false);
    void writeToFile(const QString &line);
    void checkLogDirSize();
    QString levelToString(Level level);

private:
    QMutex mutex;
    QFile logFile;
    QTextStream stream;
    QString logDir;
    Level minLevel;

    qint64 maxFileSize; // 单个日志文件最大大小（字节），默认5MB
};

#endif // LOGGER_H
