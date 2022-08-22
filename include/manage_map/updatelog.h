#ifndef UPDATELOG_H
#define UPDATELOG_H

#include <QDialog>
#include <QPushButton>
#include <QtWidgets>

class ChooseMapUi;
#include "choosemapui.h"

class UpdateLog : public QDialog
{
    Q_OBJECT
public:
    explicit UpdateLog(QWidget *parent = nullptr);
    QString getMapLog();
    QString getVmapLog();
    QString aa{"0"};

private:
    QTextEdit *MapLog;
    QTextEdit *VmapLog;

    QString mapLog;
    QString vmapLog;

public Q_SLOTS:
    void getTextContent();

Q_SIGNALS:
//    void submitMapLog();
//    void submitVmapLog();

public Q_SLOTS:
};

#endif // UPDATELOG_H
