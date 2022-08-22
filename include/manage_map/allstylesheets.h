#ifndef ALLSTYLESHEETS_H
#define ALLSTYLESHEETS_H

#include "qstring.h"

namespace myStyleSheets{

    namespace myPushbutton {

         extern QString CardCommonStyleSheet;

         extern QString mapChooseUiNormal;//灰白
         extern QString mapChooseUiWarn;//黃色
         extern QString mapChooseUiError;//粉色;
         extern QString upDateLogButton;

         extern QString mapChooseUiMain;
    }

    namespace myLineEdit {
         extern QString mapChooseUiSearchBox;
    }

    namespace myLabel {
         extern QString mapChooseUiSearchLable;
         extern QString updateLogLabel;
                //background-image: url("./image.png");
    }

     namespace  myWidget {
          extern QString mapChooseUiWidget;
     }

     namespace myTree {

          extern QString tasklistUiTree;

     }

     namespace myCheckBox {
          extern QString tasklistUiCheckBox;

     }
     namespace myLineEdit {
          extern QString tasklistUiLineEdit;

     }

     namespace mySpinBox {
          extern QString tasklistUiSpinBox;

     }

     namespace myCombox {
          extern QString tasklistUiComboBox;

     }

     namespace myTextEdit {
          extern QString updateLogTextEdit;

     }

     namespace myMenu {
          extern QString updateChooseUiHoverMenu;
     }
}

#endif // ALLSTYLESHEETS_H
