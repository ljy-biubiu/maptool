/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the demonstration applications of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "mainwindow.h"

#include <QApplication>
#include <QPainterPath>
#include <QPainter>
#include <QMap>
#include <QDebug>

#include "qdir.h"
#include "QDateTime"
#include "qmutex.h"
#include "qfile.h"
#include "qsplashscreen.h"
#include "QSharedMemory"
#include "qmessagebox.h"



static QString qtLogPath = QString::fromStdString(home_str) + "/maptool-candela/qtLog/";

static QString warningName = "warning-log.txt";
static QString crittcalName = "debug-log.txt";
static QString debugName = "debug-log.txt";

static QString warningPath = qtLogPath + "warning-log.txt";
static QString crittcalPath = qtLogPath + "critical-log.txt";
static QString debugPath = qtLogPath +"debug-log.txt";

//MainWindow* p = nullptr;  // 界面指针

void render_qt_text(QPainter *painter, int w, int h, const QColor &color)
{
}

static void usage()
{
    qWarning() << "Usage: mainwindow [-SizeHint<color> <width>x<height>] ...";
    exit(1);   //exception exit
}

enum ParseCommandLineArgumentsResult {
    CommandLineArgumentsOk,
    CommandLineArgumentsError,
    HelpRequested
};

// here static that this function only can use in this file
static ParseCommandLineArgumentsResult
    parseCustomSizeHints(const QStringList &arguments, MainWindow::CustomSizeHintMap *result)
{
    result->clear();
    const int argumentCount = arguments.size();
    for (int i = 1; i < argumentCount; ++i) {
        const QString &arg = arguments.at(i);
        if (arg.startsWith(QLatin1String("-SizeHint"))) {
            const QString name = arg.mid(9);
            if (name.isEmpty())
                return CommandLineArgumentsError;
            if (++i == argumentCount)
                return CommandLineArgumentsError;
            const QString sizeStr = arguments.at(i);
            const int idx = sizeStr.indexOf(QLatin1Char('x'));
            if (idx == -1)
                return CommandLineArgumentsError;
            bool ok;
            const int w = sizeStr.leftRef(idx).toInt(&ok);
            if (!ok)
                return CommandLineArgumentsError;
            const int h = sizeStr.midRef(idx + 1).toInt(&ok);
            if (!ok)
                return CommandLineArgumentsError;
            result->insert(name, QSize(w, h));
        } else if (arg == QLatin1String("-h") || arg == QLatin1String("--help")) {
            return HelpRequested;
        } else {
            return CommandLineArgumentsError;
        }
    }

    return CommandLineArgumentsOk;
}


static void createLogFile() {
    QDir* log = new QDir;
    if ( !log->exists( qtLogPath ) ) { log->mkpath(qtLogPath); }

    QFile warningFile(warningPath);
    if(!warningFile.exists())
    {
        bool res2 = warningFile.open( QIODevice::ReadWrite | QIODevice::Text );
        qDebug() << "新建infoＦile文件是否成功:" << res2;
    }
    QFile criticalFile(crittcalPath);
    if(!criticalFile.exists())
    {
        bool res2 = criticalFile.open(QIODevice::WriteOnly | QIODevice::Text );
        qDebug() << "新建criticalＦile文件是否成功:" << res2;
    }
    QFile debugFile(debugPath);
    if(!debugFile.exists())
    {
        //debugFile.setCurrentWriteChannel(qtLogPath);
      bool res2 = debugFile.open(QIODevice::WriteOnly | QIODevice::Text );
      qDebug() << "新建debugＦile文件是否成功:" << res2;
    }
}

/**
 * @brief outputMessage MsgHandler回调函数,用于向日志文件中输出日志。
 * @param type 信息的类型，{QtDebugMsg，QtInfoMsg，QtWarningMsg，QtCriticalMsg，QtFatalMsg}之一
 * @param context 信息的资料
 * @param msg 信息的文本
 */
static void outputMessage( QtMsgType type, const QMessageLogContext& context, const QString& msg ) {
    Q_UNUSED( context )
    static QMutex mutex;
    mutex.lock();
    QByteArray localMsg = msg.toLocal8Bit();
    QString    msg_type;
    switch ( type ) {
    case QtDebugMsg:
        msg_type = QString( "[Debug]" );
        fprintf( stderr, "Debug: %s\n", localMsg.constData() );
        break;

    case QtInfoMsg:
        msg_type = QString( "[Info]" );
        fprintf( stderr, "Info: %s\n", localMsg.constData() );
        break;

    case QtWarningMsg:
        msg_type = QString( "[Warning]" );
        fprintf( stderr, "Warning: %s \n", localMsg.constData() );
        break;

    case QtCriticalMsg:
        msg_type = QString( "[Critical]" );
        fprintf( stderr, "Critical: %s \n", localMsg.constData() );
        break;

    case QtFatalMsg:
        msg_type = QString( "[Fatal]" );
        fprintf( stderr, "Fatal: %s \n", localMsg.constData() );
        abort();
    }
    /***************************info***************************/
    if ( type == QtWarningMsg ) {
        QFile warn_file( warningPath );
        //info_file.setFileName( infoLogName );

        warn_file.open( QIODevice::WriteOnly | QIODevice::Append );
        QTextStream warn_text_stream( &warn_file );
        if ( warn_file.size() == 0 )  //新文件加首行
        {
            warn_text_stream << QDateTime::currentDateTime().toString( "yyyy-MM-dd hh_mm" ) << ",QTLOG INIT FINISHLY\r\n";
        }
        warn_text_stream << QDateTime::currentDateTime().toString( "MM/dd hh:mm:ss:zzz" ) << "," << msg << "\r\n";
        warn_file.flush();
        warn_file.close();
    }

    /***************************debug***************************/
    if ( type == QtDebugMsg ) {
        QFile debug_file( debugPath );
        //info_file.setFileName( infoLogName );

        debug_file.open( QIODevice::WriteOnly | QIODevice::Append );
        QTextStream debug_text_stream( &debug_file );
        if ( debug_file.size() == 0 )  //新文件加首行
        {
            debug_text_stream << QDateTime::currentDateTime().toString( "yyyy-MM-dd hh_mm" ) << ",QTLOG INIT FINISHLY\r\n";
        }
        debug_text_stream << QDateTime::currentDateTime().toString( "MM/dd hh:mm:ss:zzz" ) << "," << msg << "\r\n";
        debug_file.flush();
        debug_file.close();
    }

//    /***************************crittcal***************************/
    if ( type == QtCriticalMsg ){
        QFile critical_file( crittcalPath );
        critical_file.open( QIODevice::WriteOnly | QIODevice::Append );
        QTextStream critical_text_stream( &critical_file );
        if ( critical_file.size() == 0 )  //新文件加首行
        {
            critical_text_stream << QDateTime::currentDateTime().toString( "yyyy-MM-dd hh_mm" ) << ",QTLOG INIT FINISHLY\r\n";
        }
        critical_text_stream << QDateTime::currentDateTime().toString( "MM/dd hh:mm:ss:zzz" ) << "," << msg << "\r\n";
        critical_file.flush();
        critical_file.close();
    }

    mutex.unlock();
}


int main(int argc, char **argv)
{
    QApplication app(argc, argv); //register gui
    MainWindow::CustomSizeHintMap customSizeHints;
    switch (parseCustomSizeHints(QCoreApplication::arguments(), &customSizeHints)) {
    case CommandLineArgumentsOk:
        break;
    case CommandLineArgumentsError:
        usage();
        return -1;
    case HelpRequested:
        usage();
        return 0;
    }
    // /************************************** 创建内存共享，检查软件运行是否唯一 **************************************/
    // QSharedMemory shared( "MapTool" );

    // if ( shared.attach(QSharedMemory::ReadOnly) )  // 共享内存被占用则直接返回
    // {
    //     QMessageBox::information( nullptr, QStringLiteral( "Warning" ), QStringLiteral( "程序已在运行!" ) );
    //     return 0;
    // }
    // shared.create( 1 );                        // 共享内存没有被占用则创建UI
    /***************************************************************************************************************/

    //    /******************************************** 设置保存日志文件 ********************************************/
    createLogFile();
    qInstallMessageHandler( outputMessage );  // 注册MsgHandler回调函数
    //    /*******************************************************************************************************/

    /******************************************** 加载启动动画 ******************************************************/
    QPixmap       pixmap( ":/res/images/startMoiveBackGournd.jpg" );
    QSplashScreen screen( pixmap );
    screen.show();
    /**************************************************************************************************************/

    /******************************************** 设置qss样式表 ******************************************************/
    QFile file(":/res/qss/myQss.qss");
    file.open(QIODevice::ReadOnly);
    QString stylesheet = QLatin1String(file.readAll());
    qApp->setStyleSheet(stylesheet);
    /****************************************************************************************************/

    // QTime dieTime = QTime::currentTime().addMSecs(1000);
    // while( QTime::currentTime() < dieTime )
    // QCoreApplication::processEvents(QEventLoop::AllEvents, 100);

    MainWindow mainWin(customSizeHints);
    screen.finish( &mainWin );  // 关闭启动动画
    mainWin.showMaximized();

    return app.exec();
}
