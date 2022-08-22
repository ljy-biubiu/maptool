#include "loadingdataui.h"
#include "qdebug.h"

LoadingDataUi::LoadingDataUi(QWidget *parent) : QDialog(parent)
{
    setWindowFlags(Qt::FramelessWindowHint);//无边框
    setAttribute(Qt::WA_TranslucentBackground);//背景透明

    m_label = new QLabel(this);
    m_textLabel = new QLabel("　　请稍等......",this);
    m_label->setGeometry(150,130,20,20);
    m_textLabel->setGeometry(150,250,120,40);
    QSize movieSize(100,100);
    m_movie = new QMovie(":/res/images/loading.gif");
    m_movie->setScaledSize(movieSize);

    m_label->setFixedSize(10,10);

    m_label->setScaledContents(true);//缩放label使其适应其内容
    m_label->setFixedSize(120,120);
    m_label->setMovie(m_movie);
    m_movie->start();
}

