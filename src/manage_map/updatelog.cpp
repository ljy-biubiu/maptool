#include "updatelog.h"
#include "qpalette.h"
#include "allstylesheets.h"


UpdateLog::UpdateLog(QWidget *parent) : QDialog(parent)
{
    setWindowTitle(tr("项目上传日志"));
//    extern MainWindow* p;
//    qDebug()<<p<<"=====================================";
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //QScrollArea *scrollArea = new QScrollArea(this);

    QVBoxLayout * mainLayout = new QVBoxLayout();
    QHBoxLayout * logFrame = new QHBoxLayout();
    //QHBoxLayout * MAP_H_Layout = new QHBoxLayout();
    QVBoxLayout * MapFrame_V_Layout  = new QVBoxLayout();
    QVBoxLayout * VmapFrame_V_Layout  = new QVBoxLayout();
    QHBoxLayout * button_H_Layout  = new QHBoxLayout();

    //create a search box
    //QLabel * x_label = new QLabel("搜索:");

    MapLog = new QTextEdit();
    VmapLog = new QTextEdit();

    MapLog->setStyleSheet(myStyleSheets::myTextEdit::updateLogTextEdit);
    VmapLog->setStyleSheet(myStyleSheets::myTextEdit::updateLogTextEdit);

    QLabel * MapTile = new QLabel("ＭＡＰ更新日志");
    QLabel * VmapTile = new QLabel("ＶＭＡＰ更新日志");

    MapTile->setStyleSheet(myStyleSheets::myLabel::updateLogLabel);
    VmapTile->setStyleSheet(myStyleSheets::myLabel::updateLogLabel);

    QPushButton* button_= new QPushButton();
    int recWidth = 600;
    int recHeight = 50;

    button_->setMinimumSize(recWidth,recHeight);
    button_->setMaximumSize(recWidth,recHeight);
    button_->setText("确认提交");

//    QPalette p;
//    p.setColor(QPalette::Button,QColor(79,79,79));
//    button_->setPalette(p);
    button_->setStyleSheet(myStyleSheets::myPushbutton::upDateLogButton);

    button_H_Layout->addStretch();
    button_H_Layout->addWidget(button_);
    button_H_Layout->addStretch();

    int heightSize = 380;
    int widthSize = 300;

    MapLog->setMinimumSize(widthSize,heightSize);
    MapLog->setMaximumSize(widthSize,heightSize);
    VmapLog->setMinimumSize(widthSize,heightSize);
    VmapLog->setMaximumSize(widthSize,heightSize);

    MapFrame_V_Layout->addSpacing(20);
    MapFrame_V_Layout->addWidget(MapTile);
    MapFrame_V_Layout->addWidget(MapLog);

    VmapFrame_V_Layout->addSpacing(20);
    VmapFrame_V_Layout->addWidget(VmapTile);
    VmapFrame_V_Layout->addWidget(VmapLog);

    logFrame->addSpacing(20);
    logFrame->addLayout(MapFrame_V_Layout);
    logFrame->addLayout(VmapFrame_V_Layout);
    logFrame->addSpacing(20);

    mainLayout->addSpacing(10);
    mainLayout->addLayout(logFrame);
    mainLayout->addStretch();
    mainLayout->addLayout(button_H_Layout);

    int DialogHeightSize = 550;
    int DialogWidthSize = 700;

    QPalette palBackGround(this->palette());

    //设置背景黑色
    //QStringList colorList = QColor::colorNames();
//    palBackGround.setColor(QPalette::Background, QColor(54,54,54));
//    this->setAutoFillBackground(true);
//    this->setPalette(palBackGround);
    this->setStyleSheet(myStyleSheets::myWidget::mapChooseUiWidget);

    this->setMinimumSize(DialogWidthSize,DialogHeightSize);
    this->setMaximumSize(DialogWidthSize,DialogHeightSize);

    this->setLayout(mainLayout);


    connect(button_,SIGNAL(clicked(bool)),this,SLOT(getTextContent()));

}

QString UpdateLog::getMapLog()
{
    return this->mapLog;
}
QString UpdateLog::getVmapLog()
{
    return this->vmapLog;
}

void UpdateLog::getTextContent()
{
    mapLog = MapLog->toPlainText();
    vmapLog = VmapLog->toPlainText();
    this->close();
}
