#ifndef LOADINGDATAUI_H
#define LOADINGDATAUI_H

#include <QDialog>

#include <qlabel.h>
#include <qmovie.h>

class LoadingDataUi : public QDialog
{
    Q_OBJECT
public:
    explicit LoadingDataUi(QWidget *parent = nullptr);

private:
    QLabel * m_label;
    QLabel * m_textLabel;
    QMovie * m_movie;

Q_SIGNALS:

public Q_SLOTS:
};

#endif // LOADINGDATAUI_H
