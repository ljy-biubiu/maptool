#include "allstylesheets.h"


namespace myStyleSheets{

    namespace myPushbutton {

        QString CardCommonStyleSheet = "border:10px solid rgba(0,0,0,0);"
                               "font-family:'Microsoft YaHei';"
                               "font-size:12px;color:black;"
                               "border-radius: 40px;}"
                               "QPushButton:hover{background-color: rgb(47,79,79);"
                               "border:none;color:rgb(255, 255, 255);}";

        QString mapChooseUiNormal = "QPushButton{background:rgba(200,200,200,255);"+CardCommonStyleSheet;//灰白
        QString mapChooseUiWarn = "QPushButton{background:rgb(205,184,150);"+CardCommonStyleSheet;//黃色
        QString mapChooseUiError = "QPushButton{background:rgba(255,182,193,125);"+CardCommonStyleSheet;//粉色;
        QString mapChooseUiMain = "QPushButton{color:rgb(230,225,235);"
                                        "background:rgba(79,79,79,255);"
                                        "border:10px solid rgba(0,0,0,0);"
                                        "font-family:'wenquanyi';"
                                        "font-size:16px;"
                                        "border-radius: 5px;}"
                                        "QPushButton:hover{background-color: rgb(192,192,192);"
                                        "border:none;color:rgb(255, 255, 255);}";
        QString upDateLogButton = "QPushButton{background:rgba(79,79,79,255);"
                                  "border:none;color:rgb(230, 225, 235);"
                                  "font-family:'wenquanyi';"
                                  "font-size:25px;"
                                  "border-radius: 5px;"
                                  "}";

    }

    namespace myLineEdit {
        QString mapChooseUiSearchBox = "background-color:rgba(205,201,201,255)";
    }

    namespace myLabel {
        QString mapChooseUiSearchLable ="QLabel {\
                font-family: \"wenquanyi\";\
                font-size: 20px;\
                color: rgb(205,201,201);\
                font-style: normal;\
                font-weight: normal;\
                \
                border-radius: 20px;\
                \
                padding-left: 20px;\
                padding-top: 0px;\
                \
                background-repeat: no-repeat;\
                background-position: left center;\
            }";
        QString updateLogLabel =" QLabel {\
                font-family: \"Microsoft YaHei\";\
                font-size: 24px;\
                color: rgb(196,196,196);\
                font-style: normal;\
                font-weight: bold;\
            }";
        //border-style: solid;\
        //border-width: 4px;\
        //border-color: aqua;\
        //border-radius: 20px;\
        //\
        //padding-left: 20px;\
        //padding-top: 0px;\
        //\
        //background-color: #2E3648;\
        //background-repeat: no-repeat;\
        //background-position: left center;
    }

    namespace  myWidget {
        // QString mapChooseUiWidget = "background-color:rgb(54,54,54)";
        QString mapChooseUiWidget;
    }

    namespace myTree {

        QString tasklistUiTree = "QTreeView\
            {\
                background-color: rgb(200,200,200);\
                font-size:13px;\
                color: black;\
            }\
            QTreeView::item:hover\
            {\
                background-color: rgb(120,120,120);\
                border: 1px solid;\
            }\
            QTreeView::item:selected:active\
            {\
                background-color: rgb(120,120,120);\
            }\
            QTreeView::item:selected:!active\
            {\
                background-color: rgb(120,120,120);\
            }";

    }

    namespace myCheckBox {
        QString tasklistUiCheckBox = "QCheckBox\
            {\
                color:rgb(73,73,73);\
                border-radius: 3px;\
                background-color: rgb(200,200,200);\
                padding: 2px;\
            }\
            QCheckBox:disabled{\
                border:1px solid black;\
                background-color: rgb(105,105,105);\
                color: rgb(220,220,220);\
            }\
            QCheckBox:hover{\
                border:1px solid black;\
                background-color: rgb(120,120,120);\
            }\
            QCheckBox::indicator\
            {\
                width: 20px;\
                height: 20px;\
            }";

    }
    namespace myLineEdit {
        QString tasklistUiLineEdit ="QLineEdit\
            {\
                border: 2px solid gray;\
                border-radius: 3px;\
                padding: 1px 2px 1px 2px;\
                background-color: rgb(200,200,200);\
            }\
            QLineEdit:disabled{\
                border:1px solid black;\
                background-color: rgb(105,105,105);\
                color: rgb(220,220,220);\
            }\
            QLineEdit:hover{\
                border:1px solid black;\
                background-color: rgb(119,136,153);\
            }";

    }

    namespace mySpinBox {
        QString tasklistUiSpinBox = "QSpinBox\
            {\
                border: 2px solid gray;\
                border-radius: 3px;\
                padding: 1px 2px 1px 2px;\
                background-color: rgb(200,200,200);\
            }\
            QSpinBox:hover{\
                border:1px solid black;\
                background-color: rgb(119,136,153);\
            }";

    }

    namespace myCombox {
        QString tasklistUiComboBox =" QComboBox \
            {\
                    border: 1px solid gray;\
                    border-radius: 3px;\
                    padding: 1px 18px 1px 3px; \
                    color: black;\
                    font:15px;\
                    background: rgb(200,200,200);\
            }\
                    QComboBox:disabled{\
                    border:1px solid black;\
                    background-color: rgb(105,105,105);\
                    color: rgb(220,220,220);\
            }\
                    QComboBox:hover{\
                    border:1px solid black;\
                    background-color: rgb(119,136,153);\
            }\
                    QComboBox QAbstractItemView {\
                    outline: 0px solid gray; \
                    border: 1px solid;\
                    background-color: rgb(200,200,200);\
            }";

    }

    namespace myTextEdit {
        QString updateLogTextEdit =" QTextEdit \
        {\
            background-color:rgba(205,201,201,190);\
            border-radius: 10px;\
        }";
    }

    namespace myMenu {
        QString updateChooseUiHoverMenu = "";
    } 
}
