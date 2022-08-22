#include "choosemapui.h"
#include "updatelog.h"
#include "loadingdataui.h"
#include "allstylesheets.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QIntValidator>
#include <QDoubleValidator>

#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QLineEdit>

#include <QApplication>
#include <QDesktopWidget>

#include <QDebug>

ChooseMapUi::ChooseMapUi(QWidget *parent) : QWidget(parent)
{
    setMouseTracking(true);
    initTaskRightMenuAndAct();
    initRightMenuAndAct();
    ordinaryIndex_ = -1;
    taskIndex_ = -1;
    scrollArea = nullptr;
}

bool ChooseMapUi::getDownLoadDataSuccess()
{
    return downLoadDataSuccess;
}
QScrollArea *ChooseMapUi::getScrollArea()
{
    return scrollArea;
}

void ChooseMapUi::loadingDatas()
{
    loadingDataUi = new LoadingDataUi();
    loadingDataUi->exec();
}

void ChooseMapUi::EndloadingDatas()
{
    loadingDataUi->close();
}

void ChooseMapUi::searchBox(QString DATA)
{

    //要使用的正则表达式
    const QString PATTERN_STRING = DATA;
    //QRegExp::cap的参数
    const int NTH = 0;

    QRegExp regExp;
    regExp.setPattern(PATTERN_STRING);
    //将匹配设置为最小（短）匹配
    regExp.setMinimal(true);
    button_list_status.clear();
    //任务栏不为true 被按下 搜索数据为任务栏
    if (!taskProjectButton->isEnabled())
    {
        // qDebug() << "task";
        if (DATA == "")
        {
            for (int i = 0; taskbutton_list_.size() > i; i++)
            {
                taskbutton_list_.at(i)->show();
                button_list_status.push_back(true);
            }
            reloadButtons(taskbutton_list_);
            return;
        }
        //开始匹配
        for (int i = 0; taskbutton_list_.size() > i; i++)
        {
            int pos;
            pos = 0;
            bool flag{false};
            while ((pos = regExp.indexIn(taskbutton_list_.at(i)->text(), pos)) != -1)
            {
                QString str = regExp.cap(NTH);
                //qDebug()<<str;
                pos += regExp.matchedLength();
                flag = true;
            }
            button_list_status.push_back(flag);
        }
        reloadButtons(taskbutton_list_);
    } 
    else if (!ordinaryProjectButton->isEnabled()) 
    { //项目栏不为true 被按下
        // qDebug() << "ordinary";
        if (DATA == "")
        {
            for (int i = 0; button_list.size() > i; i++)
            {
                button_list.at(i)->show();
                button_list_status.push_back(true);
            }
            reloadButtons(button_list);
            return;
        }
        //开始匹配
        for (int i = 0; button_list.size() > i; i++)
        {
            int pos;
            pos = 0;
            bool flag{false};
            while ((pos = regExp.indexIn(button_list.at(i)->text(), pos)) != -1)
            {
                QString str = regExp.cap(NTH);
                //qDebug()<<str;
                pos += regExp.matchedLength();
                flag = true;
            }
            button_list_status.push_back(flag);
        }
        reloadButtons(button_list);
    }
}
//根据对应数据，存储数据搜索结果
void ChooseMapUi::reloadButtons(QList< QPushButton *> &button_lists)
{
    QList<int> switchButttonsRank;
    for (int i = 0; button_list_status.size() > i; i++)
    {
        if (button_list_status.at(i) == false)
        {
            button_lists.at(i)->hide();
        }
        else
        {
            button_lists.at(i)->show();
            switchButttonsRank.append(i);
        }
    }
    layoutCardButtons(switchButttonsRank,button_lists);
}

//所有项目卡片状态
void ChooseMapUi::reLoadMapUiSlot(int rank)
{
    QString statu{"正常"};
    button_list.at(rank)->setText("项目：" + listMapDatasCP_.at(rank).find(projectKey).value() + 
                                  "\n\nMAP版本：" + listMapDatasCP_.at(rank).find(MAPkey).value() + 
                                  "\n时间：" + listMapDatasCP_.at(rank).find(MAPtime).value() + 
                                  "\n\nVMAP版本：" + listMapDatasCP_.at(rank).find(VMAPkey).value() + 
                                  "\n时间：" + listMapDatasCP_.at(rank).find(VMAPtime).value() + 
                                  "\n\n状态：" + statu);

    button_list.at(rank)->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiNormal);
}
//待绘图任务卡片状态
void ChooseMapUi::reLoadTaskUiSlot(int rank)
{
    QString statu{"正常"};
    int index = listOptimizeDatasCP_.size()+rank;
    taskbutton_list_.at(index)->setText("项目：" + listTaskDatasCP_.at(rank).find(projectKey).value() + 
                                  "\n\nDATA版本：" + listTaskDatasCP_.at(rank).find(TASKkey).value() + 
                                  "\n\n时间：" + listTaskDatasCP_.at(rank).find(TASKtime).value() + 
                                  "\n\n完成度：" + listTaskDatasCP_.at(rank).find(DATAsign).value() + 
                                  "\n\n状态：" + statu);

    taskbutton_list_.at(index)->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiNormal);
}
//待优化任务卡片状态
void ChooseMapUi::reLoadOptimizeUiSlot(int rank)
{
    QString statu{"正常"};
    taskbutton_list_.at(rank)->setText("项目：" + listOptimizeDatasCP_.at(rank).find(projectKey).value() + 
                                  "\n\n日期：" + listOptimizeDatasCP_.at(rank).find(OPTIMIZEKey).value() + 
                                  "\n\n时间：" + listOptimizeDatasCP_.at(rank).find(OPTIMIZEtime).value() + 
                                  "\n\n状态：" + statu);

    taskbutton_list_.at(rank)->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiNormal);
}


/*删除layout*/
void ChooseMapUi::deleteItem(QLayout *layout)
{
    QLayoutItem *child;
    while ((child = layout->takeAt(0)) != nullptr)
    {
        //setParent为NULL，防止删除之后界面不消失
        if (child->widget())
        {
            child->widget()->setParent(nullptr);
            delete child->widget();
        }
        else if (child->layout())
        {
            deleteItem(child->layout());
            child->layout()->deleteLater();
        }
        delete child;
    }
}

//根据对应数据搜索信息 输入对应搜索数据
void ChooseMapUi::layoutCardButtons(const QList<int> &switchButttonsRank, QList< QPushButton *> &button_lists)
{
    static QList<int> switchButttonsRank_STATIC;
    static bool flag{false};

    if (flag == false)
    {
        for (int i{0}; button_lists.size() > i; i++)
        {
            button_lists[i]->setParent(NULL);
            cardLayout->removeWidget(button_lists[i]);
        }
        flag=true;
    }
    else
    {
        for (int i{0}; switchButttonsRank_STATIC.size() > i; i++)
        {
            button_lists[switchButttonsRank_STATIC[i]]->setParent(NULL);
            cardLayout->removeWidget(button_lists[switchButttonsRank_STATIC[i]]);
        }
    }
    // qDebug() << "serached...";
    int row{0};
    int column{0};
    for (int rank{0}; switchButttonsRank.size() > rank; rank++)
    {
        if (row == 5)
        {
            row = 0;
            column++;
        }
        cardLayout->addWidget(button_lists.at(switchButttonsRank.at(rank)), column, row); //往网格的不同坐标添加不同的组件
        row++;
    }
    switchButttonsRank_STATIC = switchButttonsRank;
}

void ChooseMapUi::getMapDataTotal(bool flag, QList<QMap<QString, QString>> listMapDatas)
{
    if (flag == false)
    {
        QMessageBox::warning(this, QString(), tr("无法连接到网络，请检查网络状态，再打开软件"));
        qCritical() << "无法连接到网络，请检查网络状态，再打开软件";
        return;
    }
    listMapDatasCP_ = listMapDatas;
    downLoadDataSuccess = true;
    widget = new QWidget(this);
    scrollArea = new QScrollArea(this);

    QPalette palLine;
    palLine.setColor(QPalette::WindowText, QColor(160, 160, 160));

    //分割线
    QFrame *line_V = new QFrame();
    line_V->setFrameShape(QFrame::VLine);
    line_V->setFrameShadow(QFrame::Plain);
    line_V->setLineWidth(7);
    line_V->setGeometry(QRect(40, 180, 400, 3));
    line_V->setPalette(palLine);
    line_V->raise();

    QHBoxLayout *mainLayout_H = new QHBoxLayout();
    QVBoxLayout *mainLayout_V = new QVBoxLayout();
    QHBoxLayout *Search_H_Layout = new QHBoxLayout();
    QVBoxLayout *image_H_Layout = new QVBoxLayout();

    //create a search box
    QLabel *x_label = new QLabel("搜索:");
    x_label->setStyleSheet(myStyleSheets::myLabel::mapChooseUiSearchLable);

    searchBox_ = new QLineEdit();
    searchBox_->setStyleSheet(myStyleSheets::myLineEdit::mapChooseUiSearchBox);
    connect(searchBox_, SIGNAL(textChanged(QString)), this, SLOT(searchBox(QString)));

    QHBoxLayout *search_H_Layout = new QHBoxLayout();
    search_H_Layout->addSpacing(50);
    search_H_Layout->addWidget(x_label);
    search_H_Layout->addWidget(searchBox_);

    upgradeProjectTimeButton = new QPushButton(this);
    upgradeProjectTimeButton->setMinimumSize(150, 50);
    upgradeProjectTimeButton->setMaximumSize(150, 50);
    upgradeProjectTimeButton->setText("刷新");

    ordinaryProjectButton = new QPushButton(this);
    ordinaryProjectButton->setMinimumSize(150, 50);
    ordinaryProjectButton->setMaximumSize(150, 50);
    ordinaryProjectButton->setText("项目栏");
    ordinaryProjectButton->setEnabled(false);
    // ordinaryProjectButton->setDisabled()

    taskProjectButton = new QPushButton(this);
    taskProjectButton->setMinimumSize(150, 50);
    taskProjectButton->setMaximumSize(150, 50);
    taskProjectButton->setText("任务栏");
    // taskProjectButton->setDisabled(true);

    createNewProjectButton = new QPushButton(this);
    createNewProjectButton->setMinimumSize(150, 50);
    createNewProjectButton->setMaximumSize(150, 50);
    createNewProjectButton->setText("新建工程");

    QButtonGroup *buttonbox = new QButtonGroup;
    // 设置是否互斥
    buttonbox->setExclusive(true);
    // 将需要互斥的按钮全部添加到 QButtonGroup 中
    buttonbox->addButton(ordinaryProjectButton);
    buttonbox->addButton(taskProjectButton);

    // drawHelpButton->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiMain);
    upgradeProjectTimeButton->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiMain);
    createNewProjectButton->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiMain);
    taskProjectButton->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiMain);
    ordinaryProjectButton->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiMain);

    connect(createNewProjectButton, SIGNAL(clicked(bool)), this, SLOT(getNewProjectPath()));
    connect(upgradeProjectTimeButton, SIGNAL(clicked(bool)), this, SLOT(getNewProjectTime()));
    connect(taskProjectButton, SIGNAL(clicked(bool)), this, SLOT(taskProjectList_pbtn_clicked()));
    connect(ordinaryProjectButton, SIGNAL(clicked(bool)), this, SLOT(ordinaryProjectList_pbtn_clicked()));

    QVBoxLayout *button_V_Layout = new QVBoxLayout();

    mainLayout_V->addSpacing(20);
    button_V_Layout->addSpacing(100);
    button_V_Layout->addWidget(upgradeProjectTimeButton);
    button_V_Layout->addSpacing(50);
    button_V_Layout->addWidget(ordinaryProjectButton);
    button_V_Layout->addSpacing(50);
    button_V_Layout->addWidget(taskProjectButton);
    button_V_Layout->addSpacing(50);
    button_V_Layout->addWidget(createNewProjectButton);
    button_V_Layout->addStretch();

    // int numb = 1;
    const int recSize = 250;

    QPushButton *button_;
    cardLayout = new QGridLayout();
    // taskLayout = new QGridLayout();
    signalMapper = new QSignalMapper(this);
    //create card button

    //初始创建项目卡片
    for (int rank{0}; listMapDatas.size() > rank; rank++)
    {
        button_ = new QPushButton();
        createButtons(button_, recSize, rank+1, listMapDatas);
        button_list.append(button_);

        //设置signal　mapper三
        signalMapper->setMapping(button_, rank);
        connect(button_, SIGNAL(clicked()), signalMapper, SLOT(map()));
        connect(button_, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(ordinarycustomContextMenuRequested(QPoint)));
    }

    //layout card button
    cardLayout->setSpacing(65); //设置间距
    cardLayout->setMargin(40);

    int row{0};
    int column{0};
    for (int rank{0}; button_list.size() > rank; rank++)
    {
        if (row == 5)
        {
            row = 0;
            column++;
        }
        cardLayout->addWidget(button_list.at(rank), column, row); //往网格的不同坐标添加不同的组件
        row++;
    }

    connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(sendButtonRank(int)));

    //设置布局
    Search_H_Layout->addLayout(search_H_Layout);
    mainLayout_V->addSpacing(15);
    mainLayout_V->addLayout(Search_H_Layout);
    mainLayout_V->addSpacing(25);
    mainLayout_V->addLayout(cardLayout);
    mainLayout_V->addStretch(); //在下方添加一个弹簧填空

    mainLayout_H->addLayout(button_V_Layout);
    mainLayout_H->addSpacing(10);
    mainLayout_H->addWidget(line_V);
    mainLayout_H->addLayout(mainLayout_V);

    //设置背景黑色
    QPalette palBackGround(widget->palette());
    palBackGround.setColor(QPalette::Background, QColor(54, 54, 54));
    widget->setAutoFillBackground(true);
    widget->setPalette(palBackGround);
    //widget->setStyleSheet(myStyleSheets::myWidget::mapChooseUiWidget);
    widget->setLayout(mainLayout_H);

    //这一句setWidget必须放在pWgt里面的内容都准备完毕之后，否则显示有问题
    scrollArea->setWidget(widget);
    //    scrollArea->setWidgetResizable(true);
}

//获取任务栏任务列表
void ChooseMapUi::getOptimizeDataList(bool flag, QList<QMap<QString, QString>> listOptimizeDatas) {
    if (flag == false)
    {
        qCritical() << "暂时没有可执行的优化任务！";
    }
    listOptimizeDatasCP_ = listOptimizeDatas;
    optimizesignalMapper = new QSignalMapper(this);
}
void ChooseMapUi::getTaskDataList(bool flag, QList<QMap<QString, QString>> listTaskDatas) {
    if (flag == false)
    {
        // QMessageBox::information(NULL, "提示", "暂时没有可执行的任务！", QMessageBox::Yes);   qCritical
        qCritical() << "暂时没有可执行的绘图任务！";
    }
    listTaskDatasCP_ = listTaskDatas;
    tasksignalMapper = new QSignalMapper(this);
}

void ChooseMapUi::getNewProjectPath()
{
    QString path = QString::fromStdString(home_str);
    QString filter = "*.pcd";
    QString newPcdFile = QFileDialog::getOpenFileName(this->createNewProjectButton, tr("选择pcd文件"), path, filter);
    if (newPcdFile == "")
        return;
    Q_EMIT createNewProjectSwitch(newPcdFile);
}

void ChooseMapUi::getNewProjectTime() 
{
    Q_EMIT refreshChooseUi();
}
//点击项目栏，创建项目卡片
void ChooseMapUi::ordinaryProjectList_pbtn_clicked() {
    int row{0};
    int column{0};
    taskProjectButton->setEnabled(true);
    ordinaryProjectButton->setEnabled(false);
    //清空layout
    deleteItem(cardLayout);
    button_list.clear();
    // menu_list_.clear();
    QPushButton *button_;
    for (int rank{0}; listMapDatasCP_.size() > rank; rank++)
    {
        button_ = new QPushButton();
        createButtons(button_, 250, rank+1, listMapDatasCP_);
        button_list.append(button_);

        //设置signal　mapper三
        signalMapper->setMapping(button_, rank);
        connect(button_, SIGNAL(clicked()), signalMapper, SLOT(map()));
        connect(button_, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(ordinarycustomContextMenuRequested(QPoint)));
    }

    for (int rank{0}; button_list.size() > rank; rank++)
    {   
        if (row == 5)
        {
            row = 0;
            column++;
        }
        cardLayout->addWidget(button_list.at(rank), column, row); //往网格的不同坐标添加不同的组件
        row++;
    }
    connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(sendButtonRank(int)));
}
//点击任务栏，创建任务卡片
void ChooseMapUi::taskProjectList_pbtn_clicked() {
    if (listTaskDatasCP_.size() == 0 && listOptimizeDatasCP_.size() == 0)
    {
        QMessageBox::information(NULL, "提示", "暂时没有可执行的任务！", QMessageBox::Yes);
    }
    
    int row{0};
    int column{0};
    taskProjectButton->setEnabled(false);
    ordinaryProjectButton->setEnabled(true);
    //清空layout
    deleteItem(cardLayout);
    taskbutton_list_.clear();
    const int recSize = 250;

    QPushButton *button_;
    // QMenu *taskMenu;
    //创建待优化任务卡片
    for (int rank{0}; listOptimizeDatasCP_.size() > rank; rank++)
    {
        button_ = new QPushButton();
        // taskMenu = new QMenu(button_);
        createOptimizeButtons(button_, 250, rank, listOptimizeDatasCP_);
        taskbutton_list_.append(button_);
        // taskmenu_list_.append(taskMenu);

        //设置signal　mapper三
        //原始信号传递给signalmapper 
        optimizesignalMapper->setMapping(button_, rank);
        //设置signalmapper的转发规则, 转发为参数为int类型的信号， 并把rank的内容作为实参传递。
        connect(button_, SIGNAL(clicked()), optimizesignalMapper, SLOT(map()));
        connect(button_, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(taskcustomContextMenuRequested(QPoint)));
    }
    //创建待绘图任务卡片
    int index = 0;
    int allnum = listOptimizeDatasCP_.size() + listTaskDatasCP_.size();
    for (int rank = listOptimizeDatasCP_.size(); allnum > rank; rank++)
    {
        button_ = new QPushButton();
        // taskMenu = new QMenu(button_);
        createTaskButtons(button_, 250, index, listTaskDatasCP_);
        taskbutton_list_.append(button_);
        // taskmenu_list_.append(taskMenu);

        //设置signal　mapper三
        //原始信号传递给signalmapper 
        tasksignalMapper->setMapping(button_, index);
        //设置signalmapper的转发规则, 转发为参数为int类型的信号， 并把rank的内容作为实参传递。
        connect(button_, SIGNAL(clicked()), tasksignalMapper, SLOT(map()));
        connect(button_, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(taskcustomContextMenuRequested(QPoint)));
        index = index+1;
    }

    for (int rank{0}; taskbutton_list_.size() > rank; rank++)
    {
        if (row == 5)
        {
            row = 0;
            column++;
        }
        cardLayout->addWidget(taskbutton_list_.at(rank), column, row); //往网格的不同坐标添加不同的组件
        row++;
    }
    connect(optimizesignalMapper, SIGNAL(mapped(int)), this, SLOT(sendOptimizeButtonRank(int)));
    connect(tasksignalMapper, SIGNAL(mapped(int)), this, SLOT(sendTaskButtonRank(int)));
}

void ChooseMapUi::uploadProjectName(QString name)
{
    static bool flag{false};
    int code = QMessageBox::information(NULL, "提示", "是否执行上传操作？", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    int confirmCode = 16384;
    if (code != confirmCode)
        return;

    flag = true;
    qDebug() << "开始执行上传操作";
    Q_EMIT uploadProjectNameSwitch(name);
}

void ChooseMapUi::deleteMysqlProject(QString name) {
    Q_EMIT sentDeleteMysqlProject(name);
}

void ChooseMapUi::modifyMysqlProject() {
    Q_EMIT sentModifyMysql();
}

void ChooseMapUi::createMysqlProject() {
    Q_EMIT sentCreateMysql();
}

void ChooseMapUi::sendButtonRank(int rank)
{
    Q_EMIT sendButtonArrayData(rank);
}

void ChooseMapUi::sendTaskButtonRank(int rank)
{
    qDebug() << "触发任务";
    // int 
    Q_EMIT sendTaskButtonArrayData(rank);
}

void ChooseMapUi::sendOptimizeButtonRank(int rank)
{
    qDebug() << "触发待优化";
    Q_EMIT sendOptimizeButtonArrayData(rank);
}

void ChooseMapUi::createDialog()
{
    this->updatelog = new UpdateLog(this);
    //    connect(updatelog,SIGNAL(submitMapLog()),,)
    updatelog->exec();
}

void ChooseMapUi::createButtons(QPushButton *button_, const int &recSize, const int &numb, 
                                const QList<QMap<QString, QString>> &listMapDatas)
{
    //button_->setFlat(true);
    button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    //添加右键菜单策略,以响应customContextMenuRequested()信号
    button_->setContextMenuPolicy(Qt::CustomContextMenu);
    //加入事件过滤
    button_->installEventFilter(this);

    QString statu;
    if (listMapDatas.at(numb - 1).find(COMPAREVERTIONkey).value() != "TURE")
    {
        statu = "服务器map版本与vmap版本不匹配";
        qCritical() << listMapDatas.at(numb - 1).find(projectKey).value() << " : 服务器map版本与vmap版本不匹配";
    }
    else if (listMapDatas.at(numb - 1).find(MAPkey).value() == "V-.-.-" || listMapDatas.at(numb - 1).find(VMAPkey).value() == "V-.-.-")
    {
        statu = "服务器上缺少MAP或在VMAP文件";
        qCritical() << listMapDatas.at(numb - 1).find(projectKey).value() << " : 服务器上缺少MAP或在VMAP文件";
    }
    else if (listMapDatas.at(numb - 1).find(LOCALVMAPkey).value() == "NULL" && listMapDatas.at(numb - 1).find(LOCALMAPkey).value() == "NULL")
    {
        statu = "缺少本地map或vmap文件";
        //qWarning()<<listMapDatas.at(numb-1).find(projectKey).value()<<" : 缺少本地map或vmap文件";
    }
    else if (listMapDatas.at(numb - 1).find(LOCALMAPkey).value() != listMapDatas.at(numb - 1).find(MAPkey).value() || 
             listMapDatas.at(numb - 1).find(LOCALVMAPkey).value() != listMapDatas.at(numb - 1).find(VMAPkey).value())
    {
        statu = "服务器map或vmap版本与本地不同";
        qWarning() << listMapDatas.at(numb - 1).find(projectKey).value() << " : 服务器map或vmap版本与本地不同";
    }
    else if (listMapDatas.at(numb - 1).find(LOCALVMAPtime).value() != listMapDatas.at(numb - 1).find(VMAPtime).value() || 
             listMapDatas.at(numb - 1).find(LOCALMAPtime).value() != listMapDatas.at(numb - 1).find(MAPtime).value()) 
    {
        statu = "服务器map或vmap时间与本地不同";
        qWarning() << listMapDatas.at(numb - 1).find(projectKey).value() << " : 服务器map或vmap时间与本地不同";
    }
    else
    {
        statu = "正常";
    }

    QPalette p;
    p.setColor(QPalette::Button, QColor(200, 200, 200, 120));
    button_->setPalette(p);

    button_->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiNormal);

    button_->setMinimumSize(recSize, recSize);
    button_->setMaximumSize(recSize, recSize);
    button_->setText("项目：" + listMapDatas.at(numb - 1).find(projectKey).value() + 
                     "\n\nMAP版本：" + listMapDatas.at(numb - 1).find(MAPkey).value() + 
                     "\n时间：" + listMapDatas.at(numb - 1).find(MAPtime).value() +
                     "\n\nVMAP版本：" + listMapDatas.at(numb - 1).find(VMAPkey).value() + 
                     "\n时间：" + listMapDatas.at(numb - 1).find(VMAPtime).value() +
                     "\n\n状态：" + statu);

    if (listMapDatas.at(numb - 1).find(LOCALMAPkey).value() != listMapDatas.at(numb - 1).find(MAPkey).value() || 
        listMapDatas.at(numb - 1).find(LOCALVMAPkey).value() != listMapDatas.at(numb - 1).find(VMAPkey).value() ||
        listMapDatas.at(numb - 1).find(LOCALVMAPtime).value() != listMapDatas.at(numb - 1).find(VMAPtime).value() || 
        listMapDatas.at(numb - 1).find(LOCALMAPtime).value() != listMapDatas.at(numb - 1).find(MAPtime).value()
        /*||listMapDatas.at(numb-1).at(5).find(COMPAREVERTIONkey).value() != "TURE"*/)
    {
        std::cout << "--- " << listMapDatas.at(numb - 1).find(projectKey).value().toStdString() << " : map " << listMapDatas.at(numb - 1).find(LOCALMAPkey).value().toStdString()
                  << " vmap " << listMapDatas.at(numb - 1).find(LOCALVMAPkey).value().toStdString() << std::endl;
        button_->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiWarn); //黃色
    }

    if (listMapDatas.at(numb - 1).find(COMPAREVERTIONkey).value() != "TURE" || 
        listMapDatas.at(numb - 1).find(MAPkey).value() == "V-.-.-" || 
        listMapDatas.at(numb - 1).find(VMAPkey).value() == "V-.-.-")
    {
        button_->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiError); //粉色
        button_->setDisabled(true);
    }
}
//创建待绘图任务卡片
void ChooseMapUi::createTaskButtons(QPushButton *button_, const int &recSize, const int &rank, 
                                    const QList<QMap<QString, QString>> &listTaskDatas)
{
    //button_->setFlat(true);
    button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    button_->setContextMenuPolicy(Qt::CustomContextMenu);
    //加入事件过滤
    button_->installEventFilter(this);

    QString statu;
    if (listTaskDatas.at(rank).find(TASKkey).value() == "V-.-.-")
    {
        statu = "服务器上缺少DATA文件";
        qCritical() << listTaskDatas.at(rank).find(projectKey).value() << " : 服务器上缺少DATA文件";
    }
    else if (listTaskDatas.at(rank).find(LOCALTASKkey).value() == "NULL")
    {
        statu = "缺少本地data文件";
        //qWarning()<<listTaskDatas.at(numb-1).find(projectKey).value()<<" : 缺少本地data文件";
    }
    else if (listTaskDatas.at(rank).find(LOCALTASKkey).value() != listTaskDatas.at(rank).find(TASKkey).value())
    {
        statu = "服务器data版本与本地不同";
        qWarning() << listTaskDatas.at(rank).find(projectKey).value() << " : 服务器data版本与本地不同";
    }
    else
    {
        statu = "正常";
    }

    QPalette p;
    p.setColor(QPalette::Button, QColor(200, 200, 200, 120));
    button_->setPalette(p);

    button_->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiNormal);

    button_->setMinimumSize(recSize, recSize);
    button_->setMaximumSize(recSize, recSize);
    button_->setText("项目：" + listTaskDatasCP_.at(rank).find(projectKey).value() + 
                     "\n\nDATA版本：" + listTaskDatasCP_.at(rank).find(TASKkey).value() + 
                     "\n\n时间：" + listTaskDatasCP_.at(rank).find(TASKtime).value() + 
                     "\n\n完成度：" + listTaskDatasCP_.at(rank).find(DATAsign).value() + 
                     "\n\n状态：" + statu);

    if (listTaskDatas.at(rank).find(LOCALTASKkey).value() != listTaskDatas.at(rank).find(TASKkey).value()
        /*||listTaskDatas.at(numb-1).at(5).find(COMPAREVERTIONkey).value() != "TURE"*/)
    {
        button_->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiWarn); //黃色
    }

    if (listTaskDatas.at(rank).find(TASKkey).value() == "V-.-.-")
    {
        button_->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiError); //粉色
        button_->setDisabled(true);
    }
}
//创建待优化任务卡片
void ChooseMapUi::createOptimizeButtons(QPushButton *button_, const int &recSize, const int &rank, 
                                    const QList<QMap<QString, QString>> &listOPtimizeDatas)
{
    //button_->setFlat(true);
    button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    // setContextMenuPolicy  设置菜单
    // 参数Qt.CustomContextMenu  自定义菜单-此时右击控件不会产生contextMenuEvent事件
    // Qt.DefaultContextMenu  默认菜单；也就是右击时发生contextMenuEvent事件(方法一)
    button_->setContextMenuPolicy(Qt::CustomContextMenu);
    //加入事件过滤
    button_->installEventFilter(this);
    // 会向槽函数传递鼠标点击点的坐标-相对于控件
    // self.customContextMenuRequested.connect(self.menushow)  // 右击时发出菜单请求信号

    QString statu;
    if (listOPtimizeDatas.at(rank).find(OPTIMIZEKey).value() == "NULL")
    {
        statu = "缺少本地data文件";
        //qWarning()<<listOPtimizeDatas.at(numb-1).find(projectKey).value()<<" : 缺少本地data文件";
    }
    else if (listOPtimizeDatas.at(rank).find(LOCALOPIMIZEKey).value() != listOPtimizeDatas.at(rank).find(OPTIMIZEKey).value() ||
             listOPtimizeDatas.at(rank).find(LOCALOPTIMIZEtime).value() != listOPtimizeDatas.at(rank).find(OPTIMIZEtime).value())
    {
        statu = "服务器data版本与本地不同";
        qWarning() << listOPtimizeDatas.at(rank).find(projectKey).value() << " : 服务器data版本与本地不同";
    }
    else
    {
        statu = "正常";
    }

    QPalette p;
    p.setColor(QPalette::Button, QColor(200, 200, 200, 120));
    button_->setPalette(p);

    button_->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiNormal);

    button_->setMinimumSize(recSize, recSize);
    button_->setMaximumSize(recSize, recSize);
    button_->setText("项目：" + listOptimizeDatasCP_.at(rank).find(projectKey).value() + 
                     "\n\n日期：" + listOptimizeDatasCP_.at(rank).find(OPTIMIZEKey).value() + 
                     "\n\n时间：" + listOptimizeDatasCP_.at(rank).find(OPTIMIZEtime).value() + 
                     "\n\n状态：" + statu);

    if (listOPtimizeDatas.at(rank).find(LOCALOPIMIZEKey).value() != listOPtimizeDatas.at(rank).find(OPTIMIZEKey).value()
        /*||listTaskDatas.at(numb-1).at(5).find(COMPAREVERTIONkey).value() != "TURE"*/)
    {
        button_->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiWarn); //黃色
    }

    if (listOPtimizeDatas.at(rank).find(OPTIMIZEKey).value() == "V-.-.-")
    {
        button_->setStyleSheet(myStyleSheets::myPushbutton::mapChooseUiError); //粉色
        button_->setDisabled(true);
    }
}

bool ChooseMapUi::eventFilter(QObject *obj, QEvent *event)
{
    //是否是任务栏事件
    for (size_t i = 0; i < taskbutton_list_.size(); i++)
    {
        QPushButton *taskbutton = taskbutton_list_.at(i);
        if (obj == (QObject*)taskbutton) {
            if (event->type() == QEvent::Enter)
            {
                ordinaryIndex_ = -1;
                taskIndex_ = i;
                // qDebug() << "选中： " << taskIndex_;
                return true;
            } 
        }
    }
    //是否是项目栏事件
    for (size_t i = 0; i < button_list.size(); i++)
    {
        QPushButton *taskbutton = button_list.at(i);
        if (obj == (QObject*)taskbutton) {
            if (event->type() == QEvent::Enter)
            {
                taskIndex_ = -1;
                ordinaryIndex_ = i;
                // qDebug() << "选中项目： " << ordinaryIndex_;
                return true;
            } 
        }
    }
    //一定要在末尾添加返回语句return QWidget::eventFilter(obj, event)，否则注册的控件显示不出来
    return QWidget::eventFilter(obj, event);
}
//项目栏右键菜单栏
void ChooseMapUi::initRightMenuAndAct() 
{
  actList_.clear();
  
  //功能菜单项
  deletOrdinaryAct_ = new QAction("删除", this);
  modifyOrdinaryAct_ = new QAction("修改", this);
  //菜单
  ordinaryMenu_ = new QMenu();
  //将菜单项加入菜单中
  dialogView_->setupMenuAddAction(deletOrdinaryAct_,ordinaryMenu_,actList_);
  dialogView_->setupMenuAddAction(modifyOrdinaryAct_,ordinaryMenu_,actList_);
  
  //触发动作，槽函数
  connect(deletOrdinaryAct_, SIGNAL(triggered(bool)), this, SLOT(deletOrdinary_act_triggered()));
  connect(modifyOrdinaryAct_, SIGNAL(triggered(bool)), this, SLOT(modifyOrdinary_act_triggered()));
}

//任务栏右键菜单栏
void ChooseMapUi::initTaskRightMenuAndAct() 
{
  actList_.clear();
  
  //功能菜单项
  multipleAct_ = new QAction("多选", this);
  deletAct_ = new QAction("删除", this);
  completeAct_ = new QAction("完成", this);
  abandonAct_ = new QAction("放弃", this);
  modifyAct_ = new QAction("修改", this);
  //菜单
  taskMenu_ = new QMenu();
//   for (auto cmenu : taskmenu_list_)
//   {
  //将菜单项加入菜单中
  dialogView_->setupMenuAddAction(multipleAct_,taskMenu_,actList_);
  dialogView_->setupMenuAddAction(completeAct_,taskMenu_,actList_);
  dialogView_->setupMenuAddAction(modifyAct_,taskMenu_,actList_);
  dialogView_->setupMenuAddAction(abandonAct_,taskMenu_,actList_);
  dialogView_->setupMenuAddAction(deletAct_,taskMenu_,actList_);
//   }
  
  //触发动作，槽函数
  connect(multipleAct_, SIGNAL(triggered(bool)), this, SLOT(multiple_act_triggered()));
  connect(deletAct_, SIGNAL(triggered(bool)), this, SLOT(delet_act_triggered()));
  connect(completeAct_, SIGNAL(triggered(bool)), this, SLOT(complete_act_triggered()));
  connect(abandonAct_, SIGNAL(triggered(bool)), this, SLOT(abandon_act_triggered()));
  connect(modifyAct_, SIGNAL(triggered(bool)), this, SLOT(modify_act_triggered()));
}
//彻底删除项目
void ChooseMapUi::deletOrdinary_act_triggered()
{
    qDebug() << "选中ordinaryIndex_: " << ordinaryIndex_;
    //删除对应任务
    if (ordinaryIndex_ < listMapDatasCP_.size() && ordinaryIndex_ >= 0)
    {
        Q_EMIT sendMapDataButtonDelet(ordinaryIndex_);
    }
    //刷新
    Q_EMIT refreshChooseUi();
}
//修改项目信息
void ChooseMapUi::modifyOrdinary_act_triggered()
{
    
}

//多选任务进行优化
void ChooseMapUi::multiple_act_triggered()
{
    //弹出复选列表悬浮框
    MultipleOptimizeDialog dialog(this);
    dialog.readDialogShowPose();
    //设置layout
    dialog.setCheckBoxLayout(listOptimizeDatasCP_);
    if (dialog.exec() == QDialog::Rejected) {
        dialog.setDialogClosePose();
        return;
    }

    dialog.setDialogClosePose();
    //获得选中的索引号
    std::vector<int> listBoxsIndex;
    dialog.getCheckBoxIndex(listBoxsIndex);
    Q_EMIT sendMultipleOptimizeButtonData(listBoxsIndex);
}

//彻底删除任务
void ChooseMapUi::delet_act_triggered()
{
    qDebug() << "选中taskindex_: " << taskIndex_;
    //删除对应任务
    if (taskIndex_ < listOptimizeDatasCP_.size() && taskIndex_ >= 0)
    {
        Q_EMIT sendOptimizeButtonDelet(taskIndex_);
    }
    else if (taskIndex_ >= listOptimizeDatasCP_.size() && taskIndex_ >= 0)
    {
        int rank = taskIndex_ - listOptimizeDatasCP_.size();
        Q_EMIT sendTaskButtonDelet(rank);
    }
    //刷新
    Q_EMIT refreshChooseUi();
}
//主动完成任务
void ChooseMapUi::complete_act_triggered()
{
    qDebug() << "选中taskindex_: " << taskIndex_;
    //放弃对应任务
    if (taskIndex_ < listOptimizeDatasCP_.size() && taskIndex_ >= 0)
    {
        Q_EMIT sendOptimizeButtonComplete(taskIndex_);
    }
    else if (taskIndex_ >= listOptimizeDatasCP_.size() && taskIndex_ >= 0)
    {
        int rank = taskIndex_ - listOptimizeDatasCP_.size();
        Q_EMIT sendTaskButtonComplete(rank);
    }
    //刷新
    // Q_EMIT refreshChooseUi();
}
//放弃任务
void ChooseMapUi::abandon_act_triggered()
{
    qDebug() << "选中taskindex_: " << taskIndex_;
    //放弃对应任务
    if (taskIndex_ < listOptimizeDatasCP_.size() && taskIndex_ >= 0)
    {
        Q_EMIT sendOptimizeButtonAbandon(taskIndex_);
    }
    else if (taskIndex_ >= listOptimizeDatasCP_.size() && taskIndex_ >= 0)
    {
        int rank = taskIndex_ - listOptimizeDatasCP_.size();
        Q_EMIT sendTaskButtonAbandon(rank);
    }
    //刷新
    // Q_EMIT refreshChooseUi();
}
//修改任务信息
void ChooseMapUi::modify_act_triggered()
{
    qDebug() << "选中taskindex_: " << taskIndex_;
    //放弃对应任务
    if (taskIndex_ < listOptimizeDatasCP_.size() && taskIndex_ >= 0)
    {
        Q_EMIT sendOptimizeButtonModify(taskIndex_);
    }
    else if (taskIndex_ >= listOptimizeDatasCP_.size() && taskIndex_ >= 0)
    {
        int rank = taskIndex_ - listOptimizeDatasCP_.size();
        Q_EMIT sendTaskButtonModify(rank);
    }
}

void ChooseMapUi::ordinarycustomContextMenuRequested(const QPoint &pos)
{
//   QMenu *menu = menu_list_.at(0);
    ordinaryMenu_->exec(QCursor::pos());//呼出菜单
}

void ChooseMapUi::taskcustomContextMenuRequested(const QPoint &pos)
{
//   QMenu *menu = taskmenu_list_.at(0);
    taskMenu_->exec(QCursor::pos());//呼出菜单
}