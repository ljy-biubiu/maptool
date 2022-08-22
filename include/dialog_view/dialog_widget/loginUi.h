#ifndef LOGINUI_H
#define LOGINUI_H

#include <QDialog>
#include <QApplication>
#include <QSettings>

class MainWindow;
#include "mainwindow.h"

class LoginDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LoginDialog(MainWindow *mainwindow_, QWidget *parent = nullptr);
    QString getUserName() const { return userName->text(); }
    QString getPassWord() const { return passWord->text(); }
    void readRemberedPassword();
    void setRemberedPassword();

    QCheckBox *prerelease_checkbox = nullptr;
    QCheckBox *produce_checkbox = nullptr;
    QCheckBox *ordinary_checkbox = nullptr;
    QCheckBox *deploy_checkbox = nullptr;

public Q_SLOTS:
    void remberBox_checkbox_clicked(bool);
    void prerelease_checkbox_Clicked(bool);
    void ordinary_checkbox_Clicked(bool);
    void deploy_checkbox_Clicked(bool);
    void produce_checkbox_Clicked(bool);

private:
    void setQLineEditStyle(QLineEdit * & lineedit);
    void setQCheckBoxStyle(QCheckBox * & checkbox);

    QLineEdit *userName;
    QLineEdit *passWord;
    QCheckBox *remberBox;
    bool isRemberFlag_;

    MainWindow *mainwindow;
};

#endif // LOGINUI_H
