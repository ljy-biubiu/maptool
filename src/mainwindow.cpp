#include "mainwindow.h"

#include <QAction>
#include <QLayout>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QFile>
#include <QDataStream>
#include <QDialogButtonBox>
#include <QApplication>
#include <QPainter>
#include <QMouseEvent>
#include <QTextEdit>
#include <QDebug>
#include <QThread>
#include <QInputDialog>
#include <QButtonGroup>
#include <QRadioButton>
#include <QHBoxLayout>
#include <QTime>
#include <thread>

MainWindow::MainWindow(const CustomSizeHintMap &customSizeHints,
                       QWidget *parent, Qt::WindowFlags flags)
    : QMainWindow(parent, flags)
{
    filePath_vmap_ = "/home/";
    filePath_pcd_ = "/home/";
    mode_ = 0;
    lidar_height_ = 1.63;
    current_point_index_ = 0;
    purge_arrows_.clear();
    setObjectName("MainWindow");
    setWindowTitle("MapTool Main Window");
    setWindowIcon(QIcon(":/res/images/toolIcon6.png")); // 设置程序图标

    mapwidget_ = new MapWidget();
    dialogtools_ = new DialogWidgetTools();
    chooseMapUi_ = new ChooseMapUi(this);
    unoptimize_map_ = new Unoptimized_map(chooseMapUi_,this);
    task_map_ = new Task_map(chooseMapUi_,this);
    choose_map_ = new Choose_map(task_map_,chooseMapUi_,this,mapwidget_);
    dockviewdialog_ = new DocksVehicleViewDialog(this,mapwidget_);
    regulardialog_ = new RegularInfoDialog(this);
    taginfodialog_ = new TagInfoDialog(this);
    griddialog_ = new GetGridValueDialog(mapwidget_,this);
    exportdialog_ = new ExportCloudPcdDialog(this);
    logindialog_ = new LoginDialog(this);
    pubdialog_ = new CloudPubDialog(this);
    setgiddialog_ = new SetGidDialog(this);

    initMainView();
    setupMenuBar();
    setupToolBar();
    InitcreatModeGroup();
    setupDockWidgets();
    setupotherWidgets();
    setQWidgetRegularStatus();
    setupRightInfoWidget();
    updateCurrentTypeTags();
    updateQWidgetEnableStatus();
    getAllProjectName();
    
    if (isDownLoad_ == false) {
      changeMainWindow->setEnabled(false);
      uploadAct->setEnabled(false);
    } else {
      udpateMenuActEnabled(false);
    }
    //状态栏刷新---0.2s
    {
      connect(&update_timer_, &QTimer::timeout, this, [this](){
        updateStatusBar();
      });
      update_timer_.start(200);
    }
    //自动保存倒计时/车辆调试刷新---1s
    {
    // update_timer_.setInterval(1000);    // 1000---1s
      connect(&coutdown_timer_, &QTimer::timeout, this, [this](){
        static int sec_ = 60;
        QString countdowm_num = QString::number(sec_) + "s"; 
        autosave_label->setText(countdowm_num);
        if (sec_ != 0) {
          sec_ = sec_ - 1;
        }  else {
          sec_ = 60;
        }
      });
      coutdown_timer_.start(1000);
    }
    //定时自动保存
    {
      // autosave_timer_.setInterval(60000);  // 1000---1s 60000---1m
      connect(&autosave_timer_, &QTimer::timeout, this, [this](){
        std::string auto_save_path = "/tmp/vmap";
        QDir dir(QString::fromStdString(auto_save_path));
        if (!dir.exists()) {
          dir.mkdir(QString::fromStdString(auto_save_path));
        }
        // for auto save vmap
        {
          if (ProjectName_ == "") {
            auto_save_path = "/tmp/vmap/auto";
          } else {
            auto_save_path = "/tmp/vmap/" + ProjectName_.toStdString();
          }
          QDir dirs(QString::fromStdString(auto_save_path));
          if (!dirs.exists()) {
            dirs.mkdir(QString::fromStdString(auto_save_path));
          }
        }
        mapwidget_->writeVmapFiles(auto_save_path);
        {
          auto_save_path = auto_save_path + "/hdmap.osm";
          if (!mapwidget_->laneletmapEmpty()) {
            mapwidget_->autoSaveFile(auto_save_path);
            std::cout << "auto save filished ..." << std::endl;
          }
        }
        // getVehicleViewInfo();
      });
      autosave_timer_.start(60000);
    }
}

/* 主界面初始化
   窗口，标题栏初始化
   group以及控件初始化*/
void MainWindow::initMainView() {
    verticalLayoutMain = new QVBoxLayout(this);
    verticalLayoutMain->setContentsMargins(0,0,0,0);
    splitterMain = new QSplitter(Qt::Horizontal, this);
    splitterLeft = new QSplitter(Qt::Vertical, splitterMain);
    splitterCenter = new QSplitter(Qt::Vertical, splitterMain);
    splitterRight = new QSplitter(Qt::Vertical, splitterMain);
    splitteChoice = new QSplitter(Qt::Vertical, splitterMain);
    extraSplitters.append(splitterLeft);
    extraSplitters.append(splitterCenter);
    extraSplitters.append(splitterRight);

    horizontalLayout = new QHBoxLayout(splitterCenter);
    horizontalLayout->addWidget(mapwidget_->getRenderPanel());
    choicehoriLayout = new QHBoxLayout(splitteChoice);
    choicehoriLayout->addWidget(chooseMapUi_->getScrollArea());

    QHBoxLayout *righthorizontalLayout = new QHBoxLayout();
    righthorizontalLayout->setContentsMargins(0,0,0,0);
    rightWidget = new QWidget(splitterRight);
    rightWidget->setContentsMargins(0,0,0,0);
    rightWidget->setFixedWidth(22);
    r_toolButton = new QToolButton();
    r_toolButton->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding); 
    r_toolButton->setArrowType(Qt::LeftArrow);
    righthorizontalLayout->addWidget(r_toolButton);

    QHBoxLayout *lefthorizontalLayout = new QHBoxLayout();
    lefthorizontalLayout->setContentsMargins(0,0,0,0);
    leftWidget = new QWidget(splitterLeft);
    leftWidget->setContentsMargins(0,0,0,0);
    leftWidget->setFixedWidth(22);
    l_toolButton = new QToolButton();
    l_toolButton->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
    l_toolButton->setArrowType(Qt::RightArrow);
    lefthorizontalLayout->addWidget(l_toolButton);
    
    rightWidget->setLayout(righthorizontalLayout);
    leftWidget->setLayout(lefthorizontalLayout);
    verticalLayoutMain->addWidget(splitterMain);
    isDownLoad_ = chooseMapUi_->getDownLoadDataSuccess();
    if(isDownLoad_ == true)
    {
      for (size_t i = 0; i < extraSplitters.size(); i++)
      {
        QSplitter *listQsplitter = (QSplitter *)extraSplitters.at(i);
        listQsplitter->setVisible(false);
      }
      splitteChoice->setVisible(true);
      chooseMapUi_->getScrollArea()->setWidgetResizable(true);
    } 
    setCentralWidget(splitterMain);
    connect(choose_map_, SIGNAL(refreshMapUi()), this, SLOT(refrushMapSelectiveUi()));
    connect(choose_map_,SIGNAL(loadMapAndVmap(QString)),this,SLOT(closeMapChooseUiAndOpenPaint(QString)));
    connect(unoptimize_map_,SIGNAL(loadOptimizeData(QString)),this,SLOT(openOptimizeToolAndUpload(QString)));
    connect(unoptimize_map_,SIGNAL(loadMultipleOptimizeData(QList<QString>)),this,SLOT(openToolAndDealMultiple(QList<QString>)));
    connect(task_map_,SIGNAL(loadTaskData(QString)),this,SLOT(closeMapAndChooseDataView(QString)));
    connect(this,SIGNAL(uploadMapInfoSignal(QString)),chooseMapUi_,SLOT(uploadProjectName(QString)));
    connect(this,SIGNAL(deleteMysqlInfo(QString)),chooseMapUi_,SLOT(deleteMysqlProject(QString)));
    connect(this,SIGNAL(modifyMysqlInfo()),chooseMapUi_,SLOT(modifyMysqlProject()));
    connect(this,SIGNAL(createMysqlInfo()),chooseMapUi_,SLOT(createMysqlProject()));
    connect(this, SIGNAL(updateMapTool()), choose_map_, SLOT(versionUpdate()));

    connect(mapwidget_->getRenderPanel(), SIGNAL(mouseEventClicked(float,float,float)), this, SLOT(onMouseEventClicked(float,float,float)));
    connect(mapwidget_->getRenderPanel(), SIGNAL(rightmouseEventClicked(float,float,float,int,int)), this, SLOT(onrightMouseEventClicked(float,float,float,int,int)));
}
void MainWindow::InitcreatModeGroup() {
    // pcdzmin_edit = new QLineEdit("--");
    // pcdzmax_edit = new QLineEdit("--");
    localx_edit = new QLineEdit("--");
    localy_edit = new QLineEdit("--");
    localz_edit = new QLineEdit("--");
    localw_edit = new QLineEdit("--");
    latitude_edit = new QLineEdit("--");
    longitude_edit = new QLineEdit("--");
    
    limitvelocity_edit = new QLineEdit("--");
    linktypenum_edit = new QLineEdit("--");
    laneletid_edit = new QLineEdit("--");
    count_edit = new QLineEdit("--");
    length_edit = new QLineEdit("--");
    width_edit = new QLineEdit("--");
    regularid_edit = new QLineEdit("--");

    maptypes_comboBox = new QComboBox();
    maptypetype_comboBox = new QComboBox();
    waytypetype_comboBox = new QComboBox();
    bypasstype_comboBox = new QComboBox();
    traveldirectiontype_comboBox = new QComboBox();
    traveltype_comboBox = new QComboBox();
    travelslopedtype_comboBox = new QComboBox();
    roomtype_comboBox = new QComboBox();
    linktype_comboBox = new QComboBox();

    maptype_spinBox = new QSpinBox();
    maptype_spinBox->setMaximum(1000);

    vmap_mode_checkbox = new QCheckBox();
    lanelet2map_mode_checkbox = new QCheckBox();
    vmap_mode_checkbox->setChecked(true);

    vmapnum_label = new QLabel("--");
    lanelet2id_label = new QLabel("--");
    status_label = new QLabel("查看！");
    dialogtools_->setStatusBarLabelView((int)60, status_label);
    fps_label = new QLabel("--");
    dialogtools_->setStatusBarLabelView((int)40, fps_label);
    view_label = new QLabel("*3D.");
    dialogtools_->setStatusBarLabelView((int)60, view_label);
    camera_base_label = new QLabel("--");
    dialogtools_->setStatusBarLabelView((int)810, camera_base_label);
    pcd_z_label = new QLabel("--");
    dialogtools_->setStatusBarLabelView((int)810, pcd_z_label);
    autosave_label = new QLabel("--");
    dialogtools_->setStatusBarLabelView((int)40, autosave_label);

    for (int i = 0; i < Type_Num; i++) {
        maptypes_comboBox->addItem(type_name_str[i].c_str());
    }
    for (int i = 0; i < Type_Num; i++) {
        linktype_comboBox->addItem(
        type_name_str[i] == "lane" ? "NULL" : type_name_str[i].c_str());
    }

    connect(localx_edit, SIGNAL(editingFinished()), this, SLOT(localx_edit_Finished()));
    connect(localy_edit, SIGNAL(editingFinished()), this, SLOT(localy_edit_Finished()));
    connect(localz_edit, SIGNAL(editingFinished()), this, SLOT(localz_edit_Finished()));
    connect(localw_edit, SIGNAL(editingFinished()), this, SLOT(localw_edit_Finished()));
    connect(latitude_edit, SIGNAL(editingFinished()), this, SLOT(latitude_edit_Finished()));
    connect(longitude_edit, SIGNAL(editingFinished()), this, SLOT(longitude_edit_Finished()));
    // connect(pcdzmin_edit, SIGNAL(editingFinished()), this, SLOT(pcdzmin_edit_Finished()));
    // connect(pcdzmax_edit, SIGNAL(editingFinished()), this, SLOT(pcdzmax_edit_Finished()));
    connect(limitvelocity_edit, SIGNAL(editingFinished()), this, SLOT(limitvelocity_edit_Finished()));
    connect(linktypenum_edit, SIGNAL(editingFinished()), this, SLOT(linktypenum_edit_Finished()));
    connect(laneletid_edit, SIGNAL(editingFinished()), this, SLOT(laneletid_edit_Finished()));
    connect(count_edit, SIGNAL(editingFinished()), this, SLOT(countcell_edit_Finished()));
    connect(length_edit, SIGNAL(editingFinished()), this, SLOT(celllength_edit_Finished()));
    connect(width_edit, SIGNAL(editingFinished()), this, SLOT(cellwidth_edit_Finished()));
    connect(regularid_edit, SIGNAL(editingFinished()), this, SLOT(regularid_edit_Finished()));

    connect(maptypes_comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(maptypes_comboBox_IndexChanged()));
    connect(maptypetype_comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(maptypetype_comboBox_IndexChanged(int)));
    connect(waytypetype_comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(waytypetype_comboBox_IndexChanged(int)));
    connect(bypasstype_comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(bypasstype_comboBox_IndexChanged(int)));
    connect(traveldirectiontype_comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(traveldirectiontype_comboBox_IndexChanged(int)));
    connect(traveltype_comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(traveltype_comboBox_IndexChanged(int)));
    connect(travelslopedtype_comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(travelslopedtype_comboBox_IndexChanged(int)));
    connect(roomtype_comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(roomtype_comboBox_IndexChanged(int)));
    connect(linktype_comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(linktype_comboBox_IndexChanged(int)));
    
    connect(maptype_spinBox, SIGNAL(valueChanged(int)), this, SLOT(maptype_spinBox_valueChanged(int)));

    connect(lanelet2map_mode_checkbox, SIGNAL(clicked(bool)), this, SLOT(lanelet2map_mode_checkbox_clicked(bool)));
    connect(vmap_mode_checkbox, SIGNAL(clicked(bool)), this, SLOT(vmap_mode_checkbox_clicked(bool)));

    updateVmapcomboBoxInfo();
    initStatusBar();
}
void MainWindow::setupToolBar()
{
    toolbar_ = new ToolBar(tr("文件管理栏"), this);
    toolBars.append(toolbar_);
    addToolBar(toolbar_);
}
void MainWindow::setupMenuBar()
{ 
  //释放action指针
    dialogtools_->deleteQActionsIter(actionLists);
    dialogtools_->deleteQPushButtonsIter(listVmapButtons);

    changeMainWindow = new QToolButton();
    dialogtools_->creatQToolButton(changeMainWindow,":/res/images/Display.png","首页",true);
    
    signalMapper = new QSignalMapper(this);
    QActionGroup *modegroup = new QActionGroup(this);
    modemenu = new QMenu("模式", this);
    rBtn_addNone = new QAction("查看",this);
    dialogtools_->setupMenuAddAction(rBtn_addNone,modemenu,modegroup,actionLists);
    rBtn_addNewmap = new QAction("画图",this);
    dialogtools_->setupMenuAddAction(rBtn_addNewmap,modemenu,modegroup,actionLists);
    rBtn_pointCorrection = new QAction("修正",this);
    dialogtools_->setupMenuAddAction(rBtn_pointCorrection,modemenu,modegroup,actionLists);
    
    isviewmenu = new QMenu("预览", this);
    objectvisible_checkbox = new QAction("物体显示",this);
    dialogtools_->setupMenuAddAction(objectvisible_checkbox,isviewmenu,actionLists);
    mapvisible_checkbox = new QAction("地图显示",this);
    dialogtools_->setupMenuAddAction(mapvisible_checkbox,isviewmenu,actionLists);
    docksvisible_checkbox = new QAction("桩点显示", this);
    dialogtools_->setupMenuAddAction(docksvisible_checkbox,isviewmenu,actionLists);
    initpose_checkbox = new QAction("初始点位显示", this);
    dialogtools_->setupMenuAddAction(initpose_checkbox,isviewmenu,actionLists);
    rBtn_addNewmap->setShortcut(tr("ctrl+p"));
    rBtn_pointCorrection->setShortcut(tr("ctrl+e"));

    clearmenu = new QMenu("清除", this);
    clear_pBtn = new QAction("移除高精度地图", this);
    dialogtools_->setupMenuAddAction(clear_pBtn,clearmenu,actionLists);
    mapremove_pbtn = new QAction("移除地图", this);
    dialogtools_->setupMenuAddAction(mapremove_pbtn,clearmenu,actionLists);
    docksremove_pbtn = new QAction("移除桩点", this);
    dialogtools_->setupMenuAddAction(docksremove_pbtn,clearmenu,actionLists);

    mapvisible_checkbox->setCheckable(true);
    rBtn_pointCorrection->setCheckable(true);
    rBtn_addNone->setCheckable(true);
    rBtn_addNewmap->setCheckable(true);
    rBtn_pointCorrection->setCheckable(true);

    objectvisible_checkbox->setCheckable(true);
    objectvisible_checkbox->setChecked(true);
    mapvisible_checkbox->setChecked(true);
    docksvisible_checkbox->setCheckable(true);
    docksvisible_checkbox->setChecked(false);
    initpose_checkbox->setCheckable(true);
    initpose_checkbox->setChecked(true);
    rBtn_addNone->setChecked(true);

    mapremove_pbtn->setEnabled(false);
    docksremove_pbtn->setEnabled(false);

//file list
    filemenu = menuBar()->addMenu(tr("&文件"));
    pcdImport = new QAction("打开pcd",this);
    dialogtools_->setupMenuAddAction(pcdImport,":/res/images/pcd_file.png",filemenu,actionLists);
    pcdImport->setShortcut(tr("ctrl+o"));
    connect(pcdImport, SIGNAL(triggered(bool)), this, SLOT(loadPcds_Action()));
    blockPcdImport = new QAction("打开分块pcd",this);
    dialogtools_->setupMenuAddAction(blockPcdImport,":/res/images/blocks_pcd.png",filemenu,actionLists);
    connect(blockPcdImport, SIGNAL(triggered(bool)), this, SLOT(loadBlockPcd_Action()));

    filemenu->addSeparator();
    allmapImport = new QAction("导入所有文件",this);
    dialogtools_->setupMenuAddAction(allmapImport,":/res/images/vmap_file.png",filemenu,actionLists);
    allmapImport->setShortcut(tr("ctrl+m"));
    connect(allmapImport, SIGNAL(triggered(bool)), this, SLOT(readAllMap_Action()));

    slamfileImport_Act = new QAction("导入数据文件",this);
    dialogtools_->setupMenuAddAction(slamfileImport_Act,":/res/images/file_frame.png",filemenu,actionLists);
    slamfileImport_Act->setShortcut(tr("ctrl+h"));
    connect(slamfileImport_Act, SIGNAL(triggered(bool)), this, SLOT(readSlamfile_act_triggered()));

    filemenu->addSeparator();
    slamdataImport_Act = new QAction("导入原始数据",this);
    dialogtools_->setupMenuAddAction(slamdataImport_Act,":/res/images/slamdata.png",filemenu,actionLists);
    slamdataImport_Act->setShortcut(tr("ctrl+h"));
    connect(slamdataImport_Act, SIGNAL(triggered(bool)), this, SLOT(readSlamData_act_triggered()));

    filemenu->addSeparator();
    allmapSave = new QAction("保存所有文件",this);
    dialogtools_->setupMenuAddAction(allmapSave,":/res/images/save.png",filemenu,actionLists);
    allmapSave->setShortcut(tr("ctrl+s"));
    connect(allmapSave, SIGNAL(triggered(bool)), this, SLOT(saveAllMap_Action()));
    onlymapSave = new QAction("保存单个文件",this);
    dialogtools_->setupMenuAddAction(onlymapSave,":/res/images/save.png",filemenu,actionLists);
    connect(onlymapSave, SIGNAL(triggered(bool)), this, SLOT(saveOnlyMap_Action()));
    laneletSave = new QAction("保存lanelet文件", this);
    dialogtools_->setupMenuAddAction(laneletSave,":/res/images/save.png",filemenu,actionLists);
    connect(laneletSave, SIGNAL(triggered(bool)), this, SLOT(saveLanelet_Action()));

    exportPcd_Act = new QAction("导出pcd文件",this);
    dialogtools_->setupMenuAddAction(exportPcd_Act,":/res/images/export_pcd.png",filemenu,actionLists);
    connect(exportPcd_Act, SIGNAL(triggered(bool)), this, SLOT(exportCloudPcd_act_triggered()));

    filemenu->addSeparator();
    uploadAct = new QAction("上传",this);
    dialogtools_->setupMenuAddAction(uploadAct,":/res/images/upload.png",filemenu,actionLists);
    connect(uploadAct, SIGNAL(triggered(bool)), this, SLOT(upLoad_Action()));
    uploadAct->setShortcut(tr("F1"));

    testMapAct = new QAction("路线测试",this);
    dialogtools_->setupMenuAddAction(testMapAct,":/res/images/testMap.png",filemenu,actionLists);
    connect(testMapAct, SIGNAL(triggered(bool)), this, SLOT(testMap_Action()));
    testMapAct->setShortcut(tr("F2"));

    docksviewAct = new QAction("桩点显示(调试)",this);
    dialogtools_->setupMenuAddAction(docksviewAct,":/res/images/truck.png",filemenu,actionLists);
    connect(docksviewAct, SIGNAL(triggered(bool)), this, SLOT(docksview_Act_triggered()));

    filemenu->addSeparator();
    QAction *framePoseAct = new QAction("数据点显示",this);
    dialogtools_->setupMenuAddAction(framePoseAct,":/res/images/file_frame.png",filemenu,actionLists);
    connect(framePoseAct, SIGNAL(triggered(bool)), this, SLOT(framePose_Act_triggered()));

    filemenu->addSeparator();
    exitAct = new QAction("离开",this);
    dialogtools_->setupMenuAddAction(exitAct,":/res/images/exit.png",filemenu,actionLists);
    exitAct->setShortcut(tr("esc"));
    connect(exitAct, SIGNAL(triggered(bool)), this, SLOT(exit_Action()));

//  workwidget
    undoact = new QToolButton();
    dialogtools_->creatQToolButton(undoact,":/res/images/undo.png","撤回",false);
    connect(undoact, SIGNAL(clicked()), this, SLOT(delBackpBtn()));
    updatevmap_tBtn = new QToolButton();
    dialogtools_->creatQToolButton(updatevmap_tBtn,":/res/images/update.png","更新",false);
    connect(updatevmap_tBtn, SIGNAL(clicked()), this, SLOT(updatevmap_tBtn_Clicked()));

    refreshPcdAct = new QToolButton();
    dialogtools_->creatQToolButton(refreshPcdAct,":/res/images/refresh.png","重载",false);
    connect(refreshPcdAct, SIGNAL(clicked()), this, SLOT(refreshPcdClicked()));

    postureEcho_tBtn = new QToolButton();
    dialogtools_->creatQToolButton(postureEcho_tBtn,":/res/images/posture.png","位姿",false);
    connect(postureEcho_tBtn, SIGNAL(clicked()), this, SLOT(postureEcho_tBtn_clicked()));

    rulerDis_tBtn = new QToolButton();
    dialogtools_->creatQToolButton(rulerDis_tBtn,":/res/images/ruler.png","量尺",false);
    connect(rulerDis_tBtn, SIGNAL(clicked()), this, SLOT(rulerDis_tBtn_clicked()));
    rulerDis_tBtn->setChecked(false);

    postureEcho_tBtn->setEnabled(false);
    rulerDis_tBtn->setEnabled(false);

    userLogin_tBtn = new QToolButton();
    dialogtools_->creatQToolButton(userLogin_tBtn,":/res/images/user.png","用户",false);
    connect(userLogin_tBtn, SIGNAL(clicked()), this, SLOT(userLogin_tBtn_clicked()));

    // dialogtools_->creatQToolButton(cloudLink_tBtn,":/res/images/cloudlink.png","发布",false);
    // connect(cloudLink_tBtn, SIGNAL(clicked()), this, SLOT(cloudLink_tBtn_clicked()));
    QActionGroup *platforgroup = new QActionGroup(this);
    platformmenu = new QMenu(tr("平台"), this);
//--(暂时)
    create_batch_act = new QAction("平台-新建", this);
    modify_batch_act = new QAction("平台-修改", this);
    delete_batch_act = new QAction("平台-删除", this);
    signaldelete_batch_act = new QAction("平台-单数据删除", this);
    signaldelete_batch_act->setEnabled(false);

    connect(create_batch_act, SIGNAL(triggered(bool)), this, SLOT(create_batch_act_Action()));
    connect(modify_batch_act, SIGNAL(triggered(bool)), this, SLOT(modify_batch_act_Action()));
    connect(delete_batch_act, SIGNAL(triggered(bool)), this, SLOT(delete_batch_act_Action()));
    connect(signaldelete_batch_act, SIGNAL(triggered(bool)), this, SLOT(signaldelete_batch_act_Action()));

    dialogtools_->setGroupAction(create_batch_act,platformmenu,platforgroup,actionLists);
    dialogtools_->setGroupAction(modify_batch_act,platformmenu,platforgroup,actionLists);
    dialogtools_->setGroupAction(delete_batch_act,platformmenu,platforgroup,actionLists);
    dialogtools_->setGroupAction(signaldelete_batch_act,platformmenu,platforgroup,actionLists);

    versionmenu = new QMenu("版本", this);
    update_Act = new QAction("版本更新", this);
    dialogtools_->setupMenuAddAction(update_Act,versionmenu,actionLists);
    inspect_Act = new QAction("查看当前版本", this);
    dialogtools_->setupMenuAddAction(inspect_Act,versionmenu,actionLists);

    // reload_pBtn = new QPushButton();
    // dialogtools_->createPushButtonSize(reload_pBtn,"重载",listVmapButtons);
    // connect(reload_pBtn, SIGNAL(clicked()), this, SLOT(reloadpcdClicked()));
    delete_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(delete_pBtn,"删除",listVmapButtons);
    connect(delete_pBtn, SIGNAL(clicked()), this, SLOT(deletemapClicked()));
    delete_pBtn->setShortcut(tr("ctrl+d"));
    newtype_pBtn = new QPushButton(); 
    dialogtools_->createPushButtonSize(newtype_pBtn,"新建",listVmapButtons);
    connect(newtype_pBtn, SIGNAL(clicked()), this, SLOT(creatnewlaneClicked()));
    choosepoint_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(choosepoint_pBtn,"点选",listVmapButtons);
    connect(choosepoint_pBtn, SIGNAL(clicked()), this, SLOT(choosPointClicked()));
    bind_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(bind_pBtn,"粘合",listVmapButtons);
    connect(bind_pBtn, SIGNAL(clicked()), this, SLOT(bindClicked()));
    breakLine_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(breakLine_pBtn,"断线",listVmapButtons);
    connect(breakLine_pBtn, SIGNAL(clicked()), this, SLOT(breaklaneClicked()));
    forward_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(forward_pBtn,"向前",listVmapButtons);
    connect(forward_pBtn, SIGNAL(clicked()), this, SLOT(forwardClicked()));
    backward_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(backward_pBtn,"向后",listVmapButtons);
    connect(backward_pBtn, SIGNAL(clicked()), this, SLOT(backwardClicked()));
    forwardInsert_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(forwardInsert_pBtn,"前插",listVmapButtons);
    connect(forwardInsert_pBtn, SIGNAL(clicked()), this, SLOT(forwardInsertClicked()));
    backwardInsert_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(backwardInsert_pBtn,"后插",listVmapButtons);
    connect(backwardInsert_pBtn, SIGNAL(clicked()), this, SLOT(backwarInsertdClicked()));
    gidSet_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(gidSet_pBtn,"门禁",listVmapButtons);
    connect(gidSet_pBtn, SIGNAL(clicked()), this, SLOT(gidSet_pBtn_Clicked()));
    choosevmaps_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(choosevmaps_pBtn,"多选(偏移)",listVmapButtons);
    connect(choosevmaps_pBtn, SIGNAL(clicked()), this, SLOT(chooseVmaps_pBtn_Clicked()));
//  lanelet2
    chooselanelettypes_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(chooselanelettypes_pBtn,"选择",listVmapButtons);
    connect(chooselanelettypes_pBtn, SIGNAL(clicked()), this, SLOT(chooselanelettypes_pBtn_clicked()));
    addlanelet_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(addlanelet_pBtn,"添加",listVmapButtons);
    connect(addlanelet_pBtn, SIGNAL(clicked()), this, SLOT(addlanelet_pBtn_clicked()));
    jointlanelet_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(jointlanelet_pBtn,"路线连接",listVmapButtons);
    connect(jointlanelet_pBtn, SIGNAL(clicked()), this, SLOT(jointlanelet_pBtn_clicked()));
    inverlanelet_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(inverlanelet_pBtn,"方向取反",listVmapButtons);
    connect(inverlanelet_pBtn, SIGNAL(clicked()), this, SLOT(inverlanelet_pBtn_clicked()));
    extendLinestring_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(extendLinestring_pBtn,"路线延伸",listVmapButtons);
    connect(extendLinestring_pBtn, SIGNAL(clicked()), this, SLOT(extendLinestring_pBtn_clicked()));
    addregular_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(addregular_pBtn,"添加规则",listVmapButtons);
    connect(addregular_pBtn, SIGNAL(clicked()), this, SLOT(addregular_pBtn_clicked()));
    editregular_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(editregular_pBtn,"编辑规则",listVmapButtons);
    connect(editregular_pBtn, SIGNAL(clicked()), this, SLOT(editregular_pBtn_clicked()));
    deletregular_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(deletregular_pBtn,"删除规则",listVmapButtons);
    connect(deletregular_pBtn, SIGNAL(clicked()), this, SLOT(deletregular_pBtn_clicked()));
    addtags_pBtn = new QPushButton();
    dialogtools_->createPushButtonSize(addtags_pBtn,"添加标签",listVmapButtons);
    connect(addtags_pBtn, SIGNAL(clicked()), this, SLOT(addtags_pBtn_clicked())); 
    
// 显示栏
    mainWindowMenu = menuBar()->addMenu(tr("显示"));

    colorViewmenu = mainWindowMenu->addMenu(tr("点云颜色"));
    QActionGroup *colorgroup = new QActionGroup(this);
    colorgroup->setExclusive(true);
    colorViewmenu->setIcon(QIcon(":/res/images/platte.png"));
    aixcoloract = new QAction("AixColor", this);
    aixcoloract->setIcon(QIcon(":/res/images/aixcolor.png"));
    connect(aixcoloract, SIGNAL(triggered(bool)), this, SLOT(aixColor_Action()));
    aixcoloract->setShortcut(tr("ctrl+x"));
    intensityact = new QAction("强度信息", this);
    intensityact->setIcon(QIcon(":/res/images/intensity.png"));
    connect(intensityact, SIGNAL(triggered(bool)), this, SLOT(intensity_Action()));

    dialogtools_->setGroupAction(aixcoloract,colorViewmenu,colorgroup,actionLists);
    dialogtools_->setGroupAction(intensityact,colorViewmenu,colorgroup,actionLists);
    // aixcoloract->setChecked(true);

    sizesetmenu = mainWindowMenu->addMenu(tr("点云大小"));
    sizesetmenu->setIcon(QIcon(":/res/images/pointsize.png"));
    QAction *mappcdsize = new QAction("地图点云大小", this);
    dialogtools_->setupMenuAddAction(mappcdsize, sizesetmenu,actionLists);
    QAction *fpcsize = new QAction("图像地图点云大小", this);
    dialogtools_->setupMenuAddAction(fpcsize, sizesetmenu,actionLists);
    connect(mappcdsize, SIGNAL(triggered(bool)), this, SLOT(mapPcdSize_Action()));

    gridparamact = new QAction("网格参数",this);
    dialogtools_->setupMenuAddAction(gridparamact,":/res/images/gridsize.png", mainWindowMenu,actionLists);
    connect(gridparamact, SIGNAL(triggered(bool)), this, SLOT(gridParam_Action()));

    refreshZeroAct = new QAction("回到零点", this);
    dialogtools_->setupMenuAddAction(refreshZeroAct,":/res/images/refresh.png", mainWindowMenu,actionLists);
    connect(refreshZeroAct, SIGNAL(triggered(bool)), this, SLOT(refreshZero_Action()));
    
    mainWindowMenu->addSeparator();
    view2d3dact = new QAction("2D/3D/top3D", this);
    dialogtools_->setupMenuAddAction(view2d3dact,":/res/images/2d3dview.png", mainWindowMenu,actionLists);
    view2d3dact->setShortcut(tr("ctrl+v"));
    connect(view2d3dact, SIGNAL(triggered(bool)), this, SLOT(view2D3D_Action()));
    QActionGroup *viewgroup = new QActionGroup(this);
    view2d3dmenu = new QMenu(tr("视图选择"), this);
    twoDimenact = new QAction("2D", this);
    threeDimenact = new QAction("3D", this);
    topThreeDimenact = new QAction("TOP-3D", this);
    connect(twoDimenact, SIGNAL(triggered(bool)), this, SLOT(twoDimenact_Action()));
    connect(threeDimenact, SIGNAL(triggered(bool)), this, SLOT(threeDimenact_Action()));
    connect(topThreeDimenact, SIGNAL(triggered(bool)), this, SLOT(topThreeDimenact_Action()));
    dialogtools_->setGroupAction(twoDimenact,view2d3dmenu,viewgroup,actionLists);
    dialogtools_->setGroupAction(threeDimenact,view2d3dmenu,viewgroup,actionLists);
    dialogtools_->setGroupAction(topThreeDimenact,view2d3dmenu,viewgroup,actionLists);
    threeDimenact->setChecked(true);

    mainWindowMenu->addSeparator();
    imageview = new QAction("图片显示", this);
    dialogtools_->setupMenuAddAction(imageview,":/res/images/imageview.png", mainWindowMenu,actionLists);
    imageview->setCheckable(true);
    connect(imageview, SIGNAL(triggered(bool)), this, SLOT(image_view_Action()));

    mapchangemenu = mainWindowMenu->addMenu(tr("地图切换"));
    changegroup = new QActionGroup(this);
    changegroup->setExclusive(true);
    mapchangemenu->setIcon(QIcon(":/res/images/changemap.png"));
    pcd_all = new QAction("所有地图", this);
    connect(pcd_all, SIGNAL(triggered(bool)), this, SLOT(pcdAll_Action()));
    dialogtools_->setGroupAction(pcd_all,mapchangemenu,changegroup,actionLists);
    pcd_all->setChecked(true);


    connect(changeMainWindow, SIGNAL(clicked()), this, SLOT(changeMainWindowClicked()));
    connect(rBtn_addNone, SIGNAL(triggered(bool)), this, SLOT(rBtn_addNone_Toggled()));
    connect(rBtn_addNewmap, SIGNAL(triggered(bool)), this, SLOT(rBtn_addNewmap_Toggled(bool)));
    connect(rBtn_pointCorrection, SIGNAL(triggered(bool)), this, SLOT(rBtn_pointCorrection_Toggled(bool)));
    connect(objectvisible_checkbox, SIGNAL(triggered(bool)), this, SLOT(objectvisible_checkbox_clicked(bool)));
    connect(mapvisible_checkbox, SIGNAL(triggered(bool)), this, SLOT(mapvisible_checkbox_clicked(bool)));
    connect(docksvisible_checkbox, SIGNAL(triggered(bool)), this, SLOT(docksvisible_checkbox_clicked(bool)));
    connect(initpose_checkbox, SIGNAL(triggered(bool)), this, SLOT(initpose_checkbox_clicked(bool)));
    connect(clear_pBtn, SIGNAL(triggered(bool)), this, SLOT(clearAllMap_checkbox_clicked()));
    connect(mapremove_pbtn, SIGNAL(triggered(bool)), this, SLOT(mapremove_checkbox_clicked()));
    connect(docksremove_pbtn, SIGNAL(triggered(bool)), this, SLOT(docksremove_checkbox_clicked()));
    connect(update_Act, SIGNAL(triggered(bool)), this, SLOT(update_Act_checkbox_clicked()));
    connect(inspect_Act, SIGNAL(triggered(bool)), this, SLOT(inspect_Act_checkbox_clicked()));
//工具&其他
    dockWidgetMenu = menuBar()->addMenu(tr("&工具"));
    otherWidgetMenu = menuBar()->addMenu(tr("&其他"));
}
void MainWindow::refrushMapSelectiveUi()
{
  std::cout << "----刷新----" << std::endl;
  dialogtools_->deleteItem(choicehoriLayout);
  if (choose_map_ != nullptr)
  {
    delete choose_map_;
  }
  if (chooseMapUi_ != nullptr)
  {
    delete chooseMapUi_;
  }
  if (unoptimize_map_ != nullptr) 
  {
    delete unoptimize_map_;
  }
  if (task_map_ != nullptr) 
  {
    delete task_map_;
  }
  chooseMapUi_ = new ChooseMapUi(this);
  unoptimize_map_ = new Unoptimized_map(chooseMapUi_,this);
  task_map_ = new Task_map(chooseMapUi_, this);
  choose_map_ = new Choose_map(task_map_, chooseMapUi_, this, mapwidget_);
  choicehoriLayout->addWidget(chooseMapUi_->getScrollArea());

  if (chooseMapUi_->getDownLoadDataSuccess() == true)
  {
    for (size_t i = 0; i < extraSplitters.size(); i++)
    {
      QSplitter *listQsplitter = (QSplitter *)extraSplitters.at(i);
      listQsplitter->hide();
    }
    splitteChoice->setVisible(true);
    chooseMapUi_->getScrollArea()->setWidgetResizable(true);
  }
  setCentralWidget(splitterMain);
  for (auto listActDock : fixDockWidgets)
  {
    QAction *listAction = (QAction *)listActDock.first;
    QDockWidget *listDockWidget = (QDockWidget *)listActDock.second;
    listDockWidget->setVisible(false);
    listAction->setChecked(false);
    r_toolButton->setArrowType(Qt::LeftArrow);
  }
  changeMainWindow->setChecked(false);
  udpateMenuActEnabled(false);
  connect(choose_map_, SIGNAL(refreshMapUi()), this, SLOT(refrushMapSelectiveUi()));
  connect(choose_map_, SIGNAL(loadMapAndVmap(QString)), this, SLOT(closeMapChooseUiAndOpenPaint(QString)));
  connect(unoptimize_map_,SIGNAL(loadOptimizeData(QString)),this,SLOT(openOptimizeToolAndUpload(QString)));
  connect(unoptimize_map_,SIGNAL(loadMultipleOptimizeData(QList<QString>)),this,SLOT(openToolAndDealMultiple(QList<QString>)));
  connect(task_map_,SIGNAL(loadTaskData(QString)),this,SLOT(closeMapAndChooseDataView(QString)));
  connect(this, SIGNAL(uploadMapInfoSignal(QString)), chooseMapUi_, SLOT(uploadProjectName(QString)));
  connect(this,SIGNAL(deleteMysqlInfo(QString)),chooseMapUi_,SLOT(deleteMysqlProject(QString)));
  connect(this,SIGNAL(modifyMysqlInfo()),chooseMapUi_,SLOT(modifyMysqlProject()));
  connect(this, SIGNAL(updateMapTool()), choose_map_, SLOT(versionUpdate()));
  getAllProjectName();
  this->update();
}

//选择项目栏项目
void MainWindow::closeMapChooseUiAndOpenPaint(QString PROJECT)
{ 
  //清除，释放指针
    isTask_ = false;
    pcdIdPaths.clear();
    listGidInfos_.clear();
    purge_arrows_.clear();
    if (qActions.size() > 0) {
      dialogtools_->deleteQActionsIter(qActions);
    }
  //加载地图，vmap,slam文件路径
    CurQString_ = PROJECT;
    ProjectName_ = CurQString_;
    QString pcdpath_ = QString::fromStdString(home_str) +"/maptool-candela/CTI_MAP_PROJECT/" + PROJECT + "/MAP/" + PROJECT + "/" + PROJECT + ".pcd";
    QString rpcdpath_ = QString::fromStdString(home_str) +"/maptool-candela/CTI_MAP_PROJECT/" + PROJECT + "/RMAP/" + PROJECT + "/" + PROJECT + "_r.pcd";
    QString slamFile = QString::fromStdString(home_str) +"/maptool-candela/CTI_MAP_PROJECT/" + PROJECT + "/DATA/keyframes.csv";
    project_vmap_ = QString::fromStdString(home_str) +"/maptool-candela/CTI_MAP_PROJECT/" + PROJECT + "/VMAP/" + PROJECT;
    // QStringList string_list = dialogtools_->projectStringDeal(home_str,PROJECT);
  //判断是否存在rpcd文件,存在则加载,不存在加载滤波点云
    map_paths_.clear();
    QFileInfo Fileinfo(rpcdpath_);
    if (Fileinfo.isFile()) //isFile:文件 isDir:目录 exists:存在
    {
      //加载地图
      readOnlyPcdFromProject(0,rpcdpath_);
      pcdpath_ = rpcdpath_;
    }
    else
    {
      qDebug() << "无rpcd文件存在，加载pcd文件";
      readOnlyPcdFromProject(0,pcdpath_);
    } 
  //加载vmap&lanelet文件
    QStringList string_list = dialogtools_->projectStringDeal(home_str,PROJECT);
    if (string_list.size() < 1) {
      mapwidget_->clearAllVmap();
    } else {
      filePath_vmap_ = mapwidget_->readAllVmapFile(string_list,listGidInfos_);
      purge_arrows_ = mapwidget_->getPurgeArrowInfo();
      showPurgeArrow();
    }
    initDisplayData();
  //判断是否存在slam文件，加载原始数据文件
    QFileInfo fileInfo(slamFile);
    if (fileInfo.isFile())
    {
      readSlamFile(slamFile);
    }
    else {
      QMessageBox::question(this, QObject::tr("提示"), QObject::tr("原始数据文件不存在，不可加载."),
                        QMessageBox::Yes);
    }
  //跳转界面
    if (rBtn_addNewmap->isChecked() || rBtn_pointCorrection->isChecked())
    {
      mapToolWidget->setVisible(true);
      r_toolButton->setArrowType(Qt::RightArrow);
    }
    setMainWidgetView(true,false);
    udpateMenuActEnabled(true);
    changeMainWindow->setChecked(true);
  //显示项目按键
    projectPcd = new QAction(ProjectName_,this);
    dialogtools_->setGroupAction(projectPcd,mapchangemenu,changegroup,actionLists);
    connect(projectPcd, SIGNAL(triggered(bool)), this, SLOT(doProjectPcdChange_Action()));
    pcdIdPaths.append(qMakePair(0,pcdpath_));
    qActions.append(projectPcd);

    // choosePullMapDocksView();
    // initPoseGet();
    // updateVehicleData(ProjectName_);
}
//选择任务栏待绘图任务
void MainWindow::closeMapAndChooseDataView(QString PROJECT) {
    isTask_ = true;
    CurQString_ = PROJECT;
    ProjectName_ = CurQString_;
    listGidInfos_.clear();

    std::cout << "加载task...  " << std::endl;
  //地图文件路径
    QString datapath = QString::fromStdString(home_str) +"/maptool-candela/CTI_MAP_PROJECT/" + PROJECT + "/DATA/" + PROJECT + "/" + PROJECT;
  //补图vmap文件路径
    QString suppleStr = QString::fromStdString(home_str) +"/maptool-candela/CTI_MAP_PROJECT/" + PROJECT + "/DATA/" + PROJECT + "/vmap/" + PROJECT;
    project_vmap_ = QString::fromStdString(home_str) +"/maptool-candela/CTI_MAP_PROJECT/" + PROJECT + "/VMAP/" + PROJECT;
  //清除上一份地图信息
    mapwidget_->clearAllVmap();
    mapwidget_->clearAllLanelet2map();
    mapwidget_->seceneMapRemove();
  //加载当前原始数据
    getSlamDataPoses(datapath);
  //拷贝一份keyframe_pose
    QString keyframeStr = datapath + "/keyframes_pose";
    std::string cmd_str = "cp " + keyframeStr.toStdString() + " " + home_str +"/maptool-candela/CTI_MAP_PROJECT/" + PROJECT.toStdString() + "/DATA";
    std::system(cmd_str.c_str());
  //判断补图优化后vmap是否存在,存在显示补图优化后vmap，不存在生成轨迹（暂时）
    qDebug() << "vmap文件路径：" << suppleStr;
    QFileInfo fileInfo(suppleStr);
    if (fileInfo.exists())    //isFile:文件 isDir:目录 exists:存在
    {
      std::cout << "存在文件。" << std::endl;
      QString suffix = "*.csv";
      QStringList string_list;  //存储文件路径，非文件名称
      bool result = dialogtools_->isFileExistance(suppleStr,suffix,string_list);
      
      if (result)
      {
        std::cout << "加载补图优化后csv文件" << std::endl;
        filePath_vmap_ = mapwidget_->readAllVmapFile(string_list,listGidInfos_);
        initDisplayData();
      }
      else if (keyframes_poses_.size() != 0)
      {
        //生成轨迹
        std::cout << "生成轨迹" << std::endl;
        createNewTrajectory(keyframes_poses_);
      }
    }
    else {
      createNewTrajectory(keyframes_poses_);
    }
  //界面切换
    if (rBtn_addNewmap->isChecked() || rBtn_pointCorrection->isChecked())
    {
      mapToolWidget->setVisible(true);
      r_toolButton->setArrowType(Qt::RightArrow);
    }
    setMainWidgetView(true,false);
    udpateMenuActEnabled(true);
    changeMainWindow->setChecked(true);
    toolbar_->exportbutton->setEnabled(true);
}
//选择任务栏待优化任务
void MainWindow::openOptimizeToolAndUpload(QString PROJECT) {
    qDebug() << "project: " << PROJECT;
    //项目路径
    QList<QString> listStr;
    QString unopStr = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_OPTIMIZE/" + PROJECT + "/DATA/lslam-map";
    listStr.push_back(unopStr);
    qDebug() << "fileinfo: " << unopStr;
    //调用优化工具
    inputProjectToOptimize(listStr);
    // QString text = QInputDialog::getText(this, tr("园区输入"),tr("请输入操作的园区名："), QLineEdit::,0, &ok);
}
//优化任务--多选
void MainWindow::openToolAndDealMultiple(QList<QString> listPROJECT) {
    QList<QString> listStr;
    //项目路径
    for (size_t i = 0; i < listPROJECT.size(); i++)
    {
      QString unopStr = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_OPTIMIZE/" + listPROJECT.at(i) + "/DATA/lslam-map";
      listStr.push_back(unopStr);
      qDebug() << "fileinfo: " << unopStr;
    }
    inputProjectToOptimize(listStr);
}
//查看工具是否存在 -> 输入项目名 -> 判断 -> 获取操作类型
void MainWindow::inputProjectToOptimize(QList<QString> &listStr) {
  //判断优化工具是否存在
    QString toolStr = QString::fromStdString(home_str) + "/map_editor/map_editor.sh";
    QFileInfo Fileinfo(toolStr);
    if (!Fileinfo.isFile())
    {
      QMessageBox::question(this, QObject::tr("提示"), QObject::tr("设备中不存在优化工具文件，不可调用！"),
                        QMessageBox::Yes);
      return;
    }

  //调用优化工具
    QString unopName;
    std::string supplyInfo;
    //获取任务实际项目名称
    QInputDialog projectNameDia(this);
    projectNameDia.setWindowTitle("园区名称输入");
    projectNameDia.setLabelText("请输入操作的园区名：");
    projectNameDia.setInputMode(QInputDialog::TextInput); //可输入其它参数 DoubleInput IntInpout
    //获取项目类型
    if (projectNameDia.exec() == QInputDialog::Accepted)
    {
      unopName = projectNameDia.textValue();
    }
    else {
      return;
    }
    
    if (!unopName.isEmpty() && allProjectNames.size() != 0)
    {
      qDebug() << "unop name: " << unopName;
      for (auto proname : allProjectNames)
      {
        if (unopName == proname)
        {
          supplyInfo = "MERGE"; //补图
          qDebug() << "存在，补图.";
          break;
        }
        else
        {
          supplyInfo = "NEW";   //新建
          // qDebug() << "不存在，新建图.";
        }
      }
    }
    else { 
      return; 
    }
    std::cout << "操作结果：" << supplyInfo << std::endl;
    std::thread thRunOptimizeTool(&MainWindow::runOptimizeTool, this, std::ref(unopName),std::ref(supplyInfo),std::ref(listStr));
    thRunOptimizeTool.detach();
}

void MainWindow::runOptimizeTool(QString &unopname, std::string supplyInfo, QList<QString> &listStr) {
    std::string cmd_str = "bash " + home_str + "/map_editor/map_editor.sh " + " " + unopname.toStdString() + " " + supplyInfo + " ";
    for (size_t i = 0; i < listStr.size(); i++)
    {
      cmd_str = cmd_str + listStr.at(i).toStdString() + " ";
    }
    std::cout << "调用指令： " << cmd_str << std::endl;
    std::system(cmd_str.c_str());
}

void MainWindow::setMainWidgetView(bool status,bool state) {
  for (size_t i = 0; i < extraSplitters.size(); i++)
  {
    QSplitter *listQsplitter = (QSplitter *)extraSplitters.at(i);
    listQsplitter->setVisible(status);
  }
  splitteChoice->setVisible(state);
}
void MainWindow::initStatusBar() {
  statusBar()->addWidget(status_label);
  statusBar()->addWidget(view_label);
  statusBar()->addWidget(camera_base_label);
  statusBar()->addWidget(pcd_z_label);
  statusBar()->addWidget(fps_label);
  statusBar()->addWidget(autosave_label);
}
void MainWindow::updateStatusBar() {
  int fps = mapwidget_->getFps();
  QString fpsstring = QString::number(fps) + "Fps";
  fps_label->setText(fpsstring);
  if (changeMainWindow->isChecked()) {
    pcd_z_label->setText(QString("*高度限制（%1 ~ %2.）")
                                 .arg(local_points_zmin_)
                                 .arg(local_points_zmax_));
    camera_base_label->setText(QString("*视角（%1,  %2,  %3.）")
                                   .arg(mapwidget_->getPitch())
                                   .arg(mapwidget_->getYaw())
                                   .arg(mapwidget_->getDistance()));
  } else {
    pcd_z_label->setText("--");
    camera_base_label->setText("--");
  }
}

/*多选模式的操作：对选中的lanelet地图元素id进行保存
*/
void MainWindow::getChooseLanelet2Ids(int type_id, Way_Point_ pose) {
  if (lanelet2_ids_.size() == 2) {
    lanelet2_ids_.clear();
    chooselanelettypes_pBtn->setEnabled(true);
    return;
  }
  switch (type_id) {
    case (int)Vm_T::Lane: {
      lanelet::Lanelet current_lanelet_3d =
          mapwidget_->searchNearstLanelet(pose);
      if (current_lanelet_3d.id() > 0)
        lanelet2_ids_.push_back(current_lanelet_3d.id());
    } break;
    case (int)Vm_T::CrossWalk: {
      lanelet::Lanelet current_lanelet_3d =
          mapwidget_->searchNearstLanelet(pose);
      if (current_lanelet_3d.id() > 0)
        lanelet2_ids_.push_back(current_lanelet_3d.id());
    } break;
    case (int)Vm_T::RoadEdge: {
      lanelet::LineString3d current_linestring_3d =
          mapwidget_->searchNearstLinestring(pose);
      if (current_linestring_3d.id() > 0)
        lanelet2_ids_.push_back(current_linestring_3d.id());
    } break;
    case (int)Vm_T::WaitLine: {
      lanelet::LineString3d current_linestring_3d =
          mapwidget_->searchNearstLinestring(pose);
      if (current_linestring_3d.id() > 0)
        lanelet2_ids_.push_back(current_linestring_3d.id());
    } break;
    case (int)Vm_T::DeceZone: {
      lanelet::LineString3d current_linestring_3d =
          mapwidget_->searchNearstLinestring(pose);
      if (current_linestring_3d.id() > 0)
        lanelet2_ids_.push_back(current_linestring_3d.id());
    } break;
    case (int)Vm_T::Junction: {
      lanelet::Polygon3d current_polygon_3d =
          mapwidget_->searchNearstPolygon(pose);
      if (current_polygon_3d.id() > 0)
        lanelet2_ids_.push_back(current_polygon_3d.id());
    } break;
    default:
      break;
  }
  if (lanelet2_ids_.size() == 2 &&
      lanelet2_ids_.front() == lanelet2_ids_.back())
    lanelet2_ids_.pop_back();
  std::cout << "lanelet2_ids_.size = " << lanelet2_ids_.size() << std::endl;
}
/*对lanelet不同地图类型，点的新增
*/
void MainWindow::createCurrentLanelet2Type(int type_id, lanelet::Point3d &point3d) {
  switch (type_id) {
    case (int)Vm_T::Lane: {
      lanelet::Lanelet current_lanelet_3d;
      if (lanelet2_ids_.size() == 0 && point3d_vec_.size() == 2) {
        current_lanelet_3d = mapwidget_->createRoadLanelet(point3d_vec_);
        // current_point3d_id_ = current_lanelet_3d.id();
        lanelet2_ids_.push_back(current_lanelet_3d.id());
        std::cout << "in new RoadLanelet current_point3d_id_ = "
                  << current_lanelet_3d.id() << " ," << current_point3d_id_
                  << std::endl;
      } else if (lanelet2_ids_.size() > 0 &&
                 (point3d_vec_.size() > 2 || point3d_vec_.size() == 1)) {
        // mapwidget_->extendLanelet(current_point3d_id_, point3d);
        mapwidget_->extendLanelet(lanelet2_ids_.back(), point3d);
      }
    } break;
    case (int)Vm_T::CrossWalk: {
      lanelet::Lanelet current_lanelet_3d;
      if (lanelet2_ids_.size() == 0 && point3d_vec_.size() == 2) {
        current_lanelet_3d = mapwidget_->createCrosswalkLanelet(point3d_vec_);
        lanelet2_ids_.push_back(current_lanelet_3d.id());
      } else if (lanelet2_ids_.size() > 0 &&
                 (point3d_vec_.size() > 2 || point3d_vec_.size() == 1)) {
        mapwidget_->extendLanelet(lanelet2_ids_.back(), point3d);
      }
    } break;
    case (int)Vm_T::RoadEdge: {
      lanelet::LineString3d current_linestring_3d;
      if (lanelet2_ids_.size() == 0 && point3d_vec_.size() == 2) {
        current_linestring_3d = mapwidget_->createRoadLinestring(point3d_vec_);
        // current_point3d_id_ = current_linestring_3d.id();
        lanelet2_ids_.push_back(current_linestring_3d.id());
        std::cout << "in new RoadEdge line current_point3d_id_ = "
                  << lanelet2_ids_.back() << std::endl;
      } else if (lanelet2_ids_.size() > 0 &&
                 (point3d_vec_.size() > 2 || point3d_vec_.size() == 1)) {
        // mapwidget_->extendLinestring(current_point3d_id_, point3d);
        mapwidget_->extendLinestring(lanelet2_ids_.back(), point3d);
      }
    } break;
    case (int)Vm_T::WaitLine: {
      lanelet::LineString3d current_linestring_3d;
      if (lanelet2_ids_.size() == 0 && point3d_vec_.size() == 2) {
        current_linestring_3d = mapwidget_->createStopline(point3d_vec_);
        lanelet2_ids_.push_back(current_linestring_3d.id());
      } else if (lanelet2_ids_.size() > 0 &&
                 (point3d_vec_.size() > 2 || point3d_vec_.size() == 1)) {
        mapwidget_->extendLinestring(lanelet2_ids_.back(), point3d);
      }
    } break;
    case (int)Vm_T::DeceZone: {
      lanelet::LineString3d current_linestring_3d;
      if (lanelet2_ids_.size() == 0 && point3d_vec_.size() == 2) {
        current_linestring_3d = mapwidget_->createBump(point3d_vec_);
        lanelet2_ids_.push_back(current_linestring_3d.id());
      } else if (lanelet2_ids_.size() > 0 &&
                 (point3d_vec_.size() > 2 || point3d_vec_.size() == 1)) {
        mapwidget_->extendLinestring(lanelet2_ids_.back(), point3d);
      }
    } break;
    case (int)Vm_T::Junction: {
      lanelet::Polygon3d current_poly_3d;
      if (lanelet2_ids_.size() == 0 && point3d_vec_.size() == 3) {
        current_poly_3d = mapwidget_->createIntersection(point3d_vec_);
        lanelet2_ids_.push_back(current_poly_3d.id());
      } else if (lanelet2_ids_.size() > 0 &&
                 (point3d_vec_.size() > 3 || point3d_vec_.size() == 1)) {
        mapwidget_->extendIntersection(lanelet2_ids_.back(), point3d);
      }
    } break;
    default:
      break;
  }
}
//数据初始化
void MainWindow::initVmapProperty(int type_id, Way_Point_ &wp) {
  switch (type_id) {
    case (int)Vm_T::Elevator: {
      wp.property_type1.P_TYPE = 0;
      char ele_level[3] = "";
      wp.property_type1.P_PASS[0] = ele_level[0];
      wp.property_type1.P_PASS[1] = ele_level[1];
      wp.property_type1.P_PASS[2] = ele_level[2];
    } break;
    default: {
      wp.property_type.P1_TYPE = 0;
      wp.property_type.P2_TRAV = 0;
      wp.property_type.P3_PASS = 0;
      wp.property_type.P4_MODE = 0;
    } break;
  }
  wp.lanelet_id = 0;
}

/*创建窗口
  创建悬浮部件*/
void MainWindow::setupotherWidgets()
{
    QAction *aboutact = new QAction("关于", this);
    aboutact->setIcon(QIcon(":/res/images/about.png"));
//    connect(aboutact, SIGNAL(triggered(bool)), this, SLOT(aboutAction()));
    otherWidgetMenu->addAction(aboutact);

    QAction *instructionact = new QAction("说明书", this);
    instructionact->setIcon(QIcon(":/res/images/infobook.png"));
//    connect(instructionact, SIGNAL(triggered(bool)), this, SLOT(instrucAction()));
    otherWidgetMenu->addAction(instructionact);
}
void MainWindow::setupDockWidgets()
{
//dockwidget
    // QPalette pal;//定义一个QPalette
    // pal.setColor(QPalette::Background, QColor(46,46,46));
    QMenu *dockWidgetListMenu = new QMenu("部件显示", this);
    disWidgetact = new QAction("&Display", this);
    imaWidgetact = new QAction("&image view", this);
    disWidgetact->setCheckable(true);
    imaWidgetact->setCheckable(true);

    mapToolWidget = new QDockWidget("Display", this);
    modeWidget = new QWidget();
    modeLayout = new QGridLayout();
    workWidget = (QWidget *)creatWorkDesk();
    // listWidgets.append(taskWidget);
    // listWidgets.append(workWidget);

    modeLayout->setContentsMargins(0,0,0,0);
    modeLayout->setSpacing(0.8);
    modeLayout->addWidget(creattasklistinfo(),0,0,1,4);
    modeLayout->addWidget(workWidget,1,0,1,4);
    modeWidget->setLayout(modeLayout);  
    modeWidget->adjustSize();
    // mapToolWidget->setAutoFillBackground(true);
    // mapToolWidget->setPalette(pal);
    mapToolWidget->setStyleSheet("QDockWidget{ background-color:rgb(46, 46, 46); }");
    mapToolWidget->setMinimumWidth(280);
    mapToolWidget->showMinimized();
    mapToolWidget->setWidget(modeWidget);
    mapToolWidget->setTitleBarWidget(NULL);
    
    imageViewWidget = new QDockWidget("image view", this);
    QWidget *imageWidget = new QWidget();
    QVBoxLayout *imageLayout = new QVBoxLayout();
    imageLayout->setContentsMargins(0,0,0,0);
    imageLayout->addWidget(creatimageInfo());
    imageWidget->setLayout(imageLayout);
    imageViewWidget->setMinimumWidth(280);
    imageViewWidget->showMinimized();
    imageViewWidget->setWidget(imageWidget);

    fixDockWidgets.append(qMakePair(disWidgetact, mapToolWidget));
    fixDockWidgets.append(qMakePair(imaWidgetact, imageViewWidget));
    for (auto listActDock : fixDockWidgets)
    {
      QDockWidget *listdockwidget = (QDockWidget *)listActDock.second;
      listdockwidget->hide();
    }

    connect(disWidgetact, SIGNAL(triggered(bool)), this, SLOT(displayWidget_Action()));
    connect(imaWidgetact, SIGNAL(triggered(bool)), this, SLOT(imageWidget_Action()));
    connect(r_toolButton, SIGNAL(clicked()), this, SLOT(right_on_btn_hide_clicked()));
    connect(l_toolButton, SIGNAL(clicked()), this, SLOT(left_on_btn_hide_clicked()));
    
    addDockWidget(Qt::RightDockWidgetArea, mapToolWidget);
    addDockWidget(Qt::RightDockWidgetArea, imageViewWidget);

    destroyDockWidgetMenu = new QMenu(tr("销毁部件"), this);
    destroyDockWidgetMenu->setEnabled(false);
    connect(destroyDockWidgetMenu, &QMenu::triggered, this, &MainWindow::destroyDockWidget);

    QAction *addwidgetAct = new QAction("添加部件...", this);
    connect(addwidgetAct, SIGNAL(triggered(bool)), this, SLOT(createNewDockWidget()));

    QMenu *mysqlDataMenu = new QMenu(tr("数据库数据"), this);
    QAction *repairMysqlAct = new QAction("修改", this);
    connect(repairMysqlAct, SIGNAL(triggered(bool)), this, SLOT(repairMysql_Act_clicked()));
    QAction *deleteMysqlAct = new QAction("删除", this);
    connect(deleteMysqlAct, SIGNAL(triggered(bool)), this, SLOT(deleteMysql_Act_clicked()));
    QAction *createMysqlAct = new QAction("创建", this);
    connect(createMysqlAct, SIGNAL(triggered(bool)), this, SLOT(createMysql_Act_clicked()));
    mysqlDataMenu->addAction(repairMysqlAct);
    mysqlDataMenu->addAction(deleteMysqlAct);
    mysqlDataMenu->addAction(createMysqlAct);

    dockWidgetListMenu->addAction(disWidgetact);
    dockWidgetListMenu->addAction(imaWidgetact);
    dockWidgetMenu->addMenu(dockWidgetListMenu);
    dockWidgetMenu->addAction(addwidgetAct);
    dockWidgetMenu->addMenu(destroyDockWidgetMenu);
    dockWidgetMenu->addSeparator();
    dockWidgetMenu->addMenu(mysqlDataMenu);
}
QFrame *MainWindow::creatimageInfo() {
    QFrame *imageFrame = new QFrame();
    imageFrame->setStyleSheet("background-image: url(:/res/images/imageframe.png)}");
    return imageFrame;
}
TaskListUi *MainWindow::creattasklistinfo() {
    TaskListUi *tasklist = new TaskListUi(this);
    return tasklist;//函数返回此QGroupBox
}
QWidget *MainWindow::creatWorkDesk() {
    QGridLayout *playLayout = new QGridLayout();
    playLayout->setContentsMargins(0,0,0,0);
    playLayout->setSpacing(0.5);
    playLayout->addWidget(newtype_pBtn,0,0,1,1);
    playLayout->addWidget(choosepoint_pBtn,0,1,1,1);
    playLayout->addWidget(breakLine_pBtn,0,2,1,1);
    playLayout->addWidget(bind_pBtn,0,3,1,1);
    playLayout->addWidget(forward_pBtn,1,0,1,1);
    playLayout->addWidget(backward_pBtn,1,1,1,1);
    playLayout->addWidget(forwardInsert_pBtn,1,2,1,1);
    playLayout->addWidget(backwardInsert_pBtn,1,3,1,1);
    playLayout->addWidget(delete_pBtn,2,0,1,1);
    playLayout->addWidget(choosevmaps_pBtn,2,1,1,1);
    playLayout->addWidget(gidSet_pBtn,2,2,1,1);

    QWidget *playWidget = new QWidget();
    playWidget->setLayout(playLayout);
    return playWidget;
}
QWidget *MainWindow::creatLaneletWorkDesk() {
    QGridLayout *playLayout = new QGridLayout();
    playLayout->setContentsMargins(0,0,0,0);
    playLayout->setSpacing(0.5);
    playLayout->addWidget(choosepoint_pBtn,0,0,1,1);
    playLayout->addWidget(chooselanelettypes_pBtn,0,1,1,1);
    playLayout->addWidget(breakLine_pBtn,0,2,1,1);
    playLayout->addWidget(addlanelet_pBtn,0,3,1,1);
    playLayout->addWidget(forward_pBtn,1,0,1,1);
    playLayout->addWidget(backward_pBtn,1,1,1,1);
    playLayout->addWidget(forwardInsert_pBtn,1,2,1,1);
    playLayout->addWidget(backwardInsert_pBtn,1,3,1,1);
    playLayout->addWidget(jointlanelet_pBtn,2,0,1,1);
    playLayout->addWidget(inverlanelet_pBtn,2,1,1,1);
    playLayout->addWidget(extendLinestring_pBtn,2,2,1,1);
    playLayout->addWidget(addtags_pBtn,2,3,1,1);
    playLayout->addWidget(addregular_pBtn,3,0,1,1);
    playLayout->addWidget(editregular_pBtn,3,1,1,1);
    playLayout->addWidget(deletregular_pBtn,3,2,1,1);

    QWidget *playWidget = new QWidget();
    playWidget->setLayout(playLayout);
    return playWidget;
}
void MainWindow::setupRightInfoWidget() {
    // std::vector<MapDockInfo> current_docks_data = mapwidget_->docksViewData();
    dockInfoWidget = new QWidget();
    dockInfoWidget->setWindowFlags(Qt::FramelessWindowHint);
    dockInfoWidget->setStyleSheet(" border-radius:5px; border-width: 1px;\
                                    background-color: rgb(200,200,200);\
                                    font-size:13px;\
                                    color: black;");
    QGridLayout * infolayout = new QGridLayout();
    dockID_label = new QLabel("--");
    dockname_label = new QLabel("--");
    docktype_label = new QLabel("--");
    docklanlon_label = new QLabel("--");
    dockpose_label = new QLabel("--");

    infolayout->setContentsMargins(0,0,0,0);
    infolayout->setSpacing(0.5);
    infolayout->addWidget(new QLabel(tr("桩点ID：")),0,0,1,1);
    infolayout->addWidget(dockID_label,0,1,1,1);
    infolayout->addWidget(new QLabel(tr("桩点名称：")),1,0,1,1);
    infolayout->addWidget(dockname_label,1,1,1,1);
    infolayout->addWidget(new QLabel(tr("桩点类型：")),2,0,1,1);
    infolayout->addWidget(docktype_label,2,1,1,1);
    infolayout->addWidget(new QLabel(tr("桩点经纬：")),3,0,1,1);
    infolayout->addWidget(docklanlon_label,3,1,1,1);
    infolayout->addWidget(new QLabel(tr("桩点位置：")),4,0,3,1);
    infolayout->addWidget(dockpose_label,4,1,3,1);

    dockInfoWidget->setLayout(infolayout);
    dockInfoWidget->setVisible(false);
}
void MainWindow::createNewDockWidget()
{ 
    // std::cout << "+++++++++++" << std::endl;
    CreateDockWidgetDialog dialog(this);
    dialog.readDialogShowPose();
    if (dialog.exec() == QDialog::Rejected) {
        dialog.setDialogClosePose();
        return;
    }

    dialog.setDialogClosePose();
    QDockWidget *dw = new QDockWidget;
    const QString name = dialog.enteredObjectName();
    dw->setObjectName(name);
    dw->setWindowTitle(name);
    dw->setWidget(new QTextEdit);

    Qt::DockWidgetArea area = dialog.location();
    switch (area) {
        case Qt::LeftDockWidgetArea:
        case Qt::RightDockWidgetArea:
        case Qt::TopDockWidgetArea:
        case Qt::BottomDockWidgetArea:
            addDockWidget(area, dw);
            break;
        default:
            if (!restoreDockWidget(dw)) {
                QMessageBox::warning(this, QString(), tr("Failed to restore dock widget"));
                delete dw;
                return;
            }
            break;
    }
    extraDockWidgets.append(dw);
    destroyDockWidgetMenu->setEnabled(true);
    destroyDockWidgetMenu->addAction(new QAction(name, this));
}
void MainWindow::destroyDockWidget(QAction *action)
{
    int index = destroyDockWidgetMenu->actions().indexOf(action);
    delete extraDockWidgets.takeAt(index);
    destroyDockWidgetMenu->removeAction(action);
    action->deleteLater();

    if (destroyDockWidgetMenu->isEmpty())
        destroyDockWidgetMenu->setEnabled(false);
}

int MainWindow::getCurrentVmapTypeID() {
    return maptypes_comboBox->currentIndex() % Type_Num;
}
int MainWindow::getCurrentVmapTypeNumID() {
    return maptype_spinBox->value();
}
void MainWindow::getFileNameIDList(const QStringList& fileNames) {
    int pcdId = 0;
    pcdIdPaths.clear();
    QStringList file_names_info_,file_name_;
    for (auto file : fileNames)
    {
      QString id = QString::number(pcdId);
      file_names_info_.clear();
      file_name_.clear();
      
      file_names_info_ = file.split("/");
      file_name_ = file_names_info_.back().split(".");
      QString name = file_name_.front();

      QAction *pcdname = new QAction(name,this);
      dialogtools_->setGroupAction(pcdname,mapchangemenu,changegroup,actionLists);
      connect(pcdname, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
      signalMapper->setMapping(pcdname, name);
      qActions.append(pcdname);
      ProjectName_ = name;
      pcdIdPaths.append(qMakePair(id,file));
      pcdId++;
    }
    //控件槽链接
    connect(signalMapper, SIGNAL(mapped(const QString &)), this, SLOT(doPcdChange_Action(const QString &)));
}

//-----triggered Action-----
void MainWindow::saveOnlyMap_Action()
{
  int type_id = getCurrentVmapTypeID();;
  mapwidget_->saveCsvMapFiles(type_id);
}
void MainWindow::saveLanelet_Action()
{
  mapwidget_->saveOsmMapFiles();
}
void MainWindow::saveAllMap_Action() {
  for (int i = 0; i < Vm_T::Type_Num; i++) {
    if (mapwidget_->getVectorType((Vm_T)i).empty()) {
      continue;
    }
    mapwidget_->saveCsvMapFiles(i);
  }
  mapwidget_->saveOsmMapFiles();
  //保存gid文件
  if (listGidInfos_.size() != 0) {
    mapwidget_->saveGidFiles(listGidInfos_);
  }
}
void MainWindow::exportCloudPcd_act_triggered() {
  // ExportCloudPcdDialog dialog(this);
  exportdialog_->readDialogShowPose();
  exportdialog_->show();
}
void MainWindow::loadPcds_Action()
{
    QTime time;
    time.start();
    pcd_paths_.clear();
    int pcdId = 0;
    //释放action指针
    dialogtools_->deleteQActionsIter(qActions);

    QStringList fileNames =
            QFileDialog::getOpenFileNames(this, QObject::tr("打开PCD点云地图文件"), filePath_pcd_,
                                          QObject::tr("pcd(*.pcd);;CTI(*.pcd.cti)"));
    if (fileNames.isEmpty()) return;

    for (auto file : fileNames)
    {
      QString id = QString::number(pcdId);
      //损坏文件禁止加载，防止工具崩溃 （access(file.toStdString().c_str()，R_OK)!=-1
      //判断文件是否可读
      bool isOk = dialogtools_->isReadOk(file);
      if(!isOk) return;

      pcd_paths_.push_back(file.toStdString());
      if (pcd_paths_.size() <= 0) return;
      //获取pcd对应名称和序列号
      // getFileNameIDList(id,file,"/",".");
      filePath_pcd_ = mapwidget_->readOnlyPcdMapFile(id,pcd_paths_);
      pcdId++;
    }
    int time_Diff = time.elapsed();
    std::cout << "time diff ============ " << (float)(time_Diff/1000.0) << "s" << std::endl;
    //界面切换
    if (changeMainWindow->isEnabled() && !changeMainWindow->isChecked())
    {  
      if (rBtn_addNewmap->isChecked() || rBtn_pointCorrection->isChecked())
      {
        mapToolWidget->setVisible(true);
        r_toolButton->setArrowType(Qt::RightArrow);
      }
      setMainWidgetView(true,false);
      udpateMenuActEnabled(true);
      changeMainWindow->setChecked(true);
      updateQWidgetEnableStatus();
    }
    std::cout << "try map init success!!" << std::endl;
    mapwidget_->getPcdmapRangeZ(global_points_zmin_, global_points_zmax_);
    local_points_zmin_ = global_points_zmin_;
    local_points_zmax_ = global_points_zmax_;

    pcd_z_label->setText(QString("*高度限制（%1 ~ %2.）")
                                 .arg(local_points_zmax_)
                                 .arg(local_points_zmin_));
    // pcdzmin_edit->setText(QString::number(local_points_zmin_));
    // pcdzmax_edit->setText(QString::number(local_points_zmax_));
    pcd_all->setChecked(true);
    mapremove_pbtn->setEnabled(true);
    getFileNameIDList(fileNames);
    choosePullMapDocksView();
    // updateVehicleData();
}
void MainWindow::readOnlyPcdFromProject(QString id,QString path) {
  if (changeMainWindow->isEnabled() && !changeMainWindow->isChecked()) {
    setMainWidgetView(true,false);
    udpateMenuActEnabled(true);
    changeMainWindow->setChecked(true);
  }
  //损坏文件禁止加载，防止工具崩溃
  struct stat sb;
  stat(path.toStdString().c_str(),&sb);
  if(sb.st_mode&S_IROTH)
  {
    std::cout<<"-----R------"<<std::endl;
  } else {
    QMessageBox::question(this, tr("提示"), tr("pcd文件不可读，请确认文件正常."),
                    QMessageBox::Yes);
    return;
  }

  map_paths_.push_back(path.toStdString());
  mapwidget_->readOnlyPcdMapFile(id,map_paths_);
  mapwidget_->getPcdmapRangeZ(global_points_zmin_, global_points_zmax_);
  local_points_zmin_ = global_points_zmin_;
  local_points_zmax_ = global_points_zmax_;

  // pcdzmin_edit->setText(QString::number(local_points_zmin_));
  // pcdzmax_edit->setText(QString::number(local_points_zmax_));
}
void MainWindow::loadBlockPcd_Action()
{
    int reply =
          QMessageBox::information(this, tr("提示"), tr("该功能暂时不支持使用!"),
                                QMessageBox::Ok);
    if (reply == QMessageBox::Ok) {
      return;
    }
    pcd_paths_.clear();
//    filePath_pcd_ = "/home/";
    QStringList fileNames =
            QFileDialog::getOpenFileNames(this, tr("打开PCD分块点云地图数据"), filePath_pcd_,
                                          tr("txt(*.txt);;ECREPT(*.txt.ecrept)"));
    for (auto file : fileNames)
    {
        pcd_paths_.push_back(file.toStdString());
        if (pcd_paths_.size() <= 0) return;
        for (auto pcd_path : pcd_paths_)
        {
            std::cout << "++++++++++++++++" << pcd_path << std::endl;
        }
    }
}
void MainWindow::readAllMap_Action()
{
//    filePath_vmap_ = "/home/";
  purge_arrows_.clear();
  listGidInfos_.clear();
  QStringList fileNames =
        QFileDialog::getOpenFileNames(this, tr("打开地图文件"), filePath_vmap_,
                                      tr("ALL(*.*);;CSV(*.csv);;OSM(*.osm)"));
  if (fileNames.isEmpty()) return;

  filePath_vmap_ = mapwidget_->readAllVmapFile(fileNames,listGidInfos_);
  // QString pathstr = filePath_vmap_ + "gid.csv";
  // mapwidget_->readGidFile(pathstr,listGidInfos_);
  std::cout << "个数： " << listGidInfos_.size() << std::endl;
  std::cout << "-----------" << filePath_vmap_.toStdString() << std::endl;
  if (filePath_vmap_ != "") {
    QStringList file_names_info = filePath_vmap_.split("/");
    ProjectName_ = file_names_info.at(file_names_info.size()-2);
    std::cout << "-----" << ProjectName_.toStdString() << std::endl;
    project_vmap_ = QString::fromStdString(home_str) +"/maptool-candela/CTI_MAP_PROJECT/" + ProjectName_ + "/VMAP/" + ProjectName_;
  }
  purge_arrows_ = mapwidget_->getPurgeArrowInfo();
  showPurgeArrow();
  initDisplayData();
}
void MainWindow::initDisplayData() {
    // init display data:
  int type_id = 0;
  int type_num_id = 0;
  current_point_index_ = 1;
  mapwidget_->getRandActiveId(type_id, type_num_id);
  // std::cout << "***type_id: " << type_id << "numd_id: " << type_num_id << "current: " << current_point_index_ << std::endl;
  maptypes_comboBox->setCurrentIndex(type_id);
  maptype_spinBox->setValue(type_num_id);
  if (changeMainWindow->isEnabled() && !changeMainWindow->isChecked())
  {
    setMainWidgetView(true,false);
    udpateMenuActEnabled(true);
    changeMainWindow->setChecked(true);
    updateQWidgetEnableStatus();
    if (rBtn_addNewmap->isChecked() || rBtn_pointCorrection->isChecked())
    {
      mapToolWidget->setVisible(true);
      r_toolButton->setArrowType(Qt::RightArrow);
    }
  }
  updateVmapcomboBoxInfo();
  updateDisplayMapElement();
}
void MainWindow::readSlamfile_act_triggered() {
  QString fileName = QFileDialog::getOpenFileName(this, tr("打开数据文件"), filePath_frame_,
                                      tr("ALL(*.*);;CSV(*.csv)"));
  if (fileName.isEmpty()) return;
  //加载数据文件
  readSlamFile(fileName);
}

void MainWindow::readSlamFile(QString &fileName) {
  keyframes_poses_.clear();
  int num;
  filePath_frame_ = mapwidget_->readKeyFramesFiles(fileName,keyframes_poses_,num);
  if (keyframes_poses_.size() != 0)
  {
    QMessageBox::question(this, QObject::tr("提示"), QObject::tr("数据加载成功."),
                        QMessageBox::Yes);
  }
  else {
    QMessageBox::question(this, QObject::tr("提示"), QObject::tr("数据加载失败，无法自动调整路线高度."),
                        QMessageBox::Yes);
  }
  qDebug() << "----------" << filePath_frame_;
}

void MainWindow::readSlamData_act_triggered() {
  GetSlamDataPathDialog dialog(this);
  dialog.readDialogShowPose();
  if (dialog.exec() == QDialog::Rejected) {
    dialog.setDialogClosePose();
    return;
  }
  
  dialog.setDialogClosePose();
  if (!dialog.enteredSlamDataPath().isEmpty()) {
    //获取路径信息
    QString filepath = dialog.enteredSlamDataPath();
    //判断建图模式
    if (dialog.getVehicleCheckBoxStatus())
    { 
      qDebug() << "车端建图";
      lidar_height_ = 1.63;
    }
    else if (dialog.getBagsCheckBoxStatus())
    {
      qDebug() << "背包建图";
      lidar_height_ = 2.0;
    }
    else 
    {
      qDebug() << "默认";
      lidar_height_ = 1.63;
    }
    
    //判断文件是否可读
    bool isOk = dialogtools_->isReadOk(filepath);
    if (isOk)
    {
      toolbar_->exportbutton->setEnabled(true);
      //删除当前路线信息
      mapwidget_->clearAllVmap();
      getSlamDataPoses(filepath);
      if (keyframes_poses_.size() != 0)
      {
        //生成轨迹
        createNewTrajectory(keyframes_poses_);
        clear_pBtn->setEnabled(true);
      }
    } else {
      return;
    }  
  }
  updateDisplayMapElement();
}
void MainWindow::aixColor_Action() {
    SetAixColorDialog dialog(mapwidget_,this);
    dialog.readDialogShowPose();
    if (dialog.exec() == QDialog::Rejected) {
        dialog.setDialogClosePose();
        return;
    }
    
    dialog.setDialogClosePose();
    if (!dialog.enteredPcdHeightMin().isEmpty() && !dialog.enteredPcdHeightMax().isEmpty())
    {
      double value_min = atof(dialog.enteredPcdHeightMin().toStdString().c_str());
      double value_max = atof(dialog.enteredPcdHeightMax().toStdString().c_str());  
      if (value_min <= global_points_zmin_)
      {
        value_min = global_points_zmin_;
      }
      if (value_max >= global_points_zmax_)
      {
        value_max = global_points_zmax_;
      }
      local_points_zmin_ = value_min;
      local_points_zmax_ = value_max;
      std::cout << "0000000000000000000000000: " <<  local_points_zmin_ << " " << local_points_zmax_ << std::endl;
      pcd_z_label->setText(QString("*高度限制（%1 ~ %2.）")
                                  .arg(local_points_zmax_)
                                  .arg(local_points_zmin_));
      // pcdzmin_edit->setText(QString::number(local_points_zmin_));
      // pcdzmax_edit->setText(QString::number(local_points_zmax_));
      // mapwidget_->setMapZmin(local_points_zmin_);
    }
    mapwidget_->setPcIntensityMode(false,local_points_zmin_,local_points_zmax_);
    mapwidget_->reloadPcdMap(local_points_zmin_, local_points_zmax_);
}
void MainWindow::intensity_Action() {
    CreatIntensityDialog dialog(this);
    dialog.readDialogShowPose();
    if (dialog.exec() == QDialog::Rejected) {
      dialog.setDialogClosePose();
      return;
    } 

    dialog.setDialogClosePose();
    double min_intensity = atof(dialog.enteredIntensityMin().toStdString().c_str());
    double max_intensity = atof(dialog.enteredIntensityMax().toStdString().c_str());

    if (min_intensity <= max_intensity)
    {
        mapwidget_->setPcIntensityMode(true,min_intensity,max_intensity);
        mapwidget_->reloadPcdMap(local_points_zmin_, local_points_zmax_);
    }
    else
    {
        QMessageBox::question(this, tr("提示"),tr("请输入规范的强度信息！"),
                                QMessageBox::Yes);
        return;
    }
}
void MainWindow::gridParam_Action() {
  // GetGridValueDialog dialog(mapwidget_,this);
  // if (dialog.exec() == QDialog::Rejected)
  //     return;
  setParamGrid();
  griddialog_->readDialogShowPose();
  griddialog_->show();
}
void MainWindow::mapPcdSize_Action() {
  bool isOk;
  QString size = QInputDialog::getText(this,tr("mpc点云大小"),tr("请输入mpc值大小"),
                                       QLineEdit::Normal,"0.5",&isOk);
  if (isOk && !size.isEmpty())
  {
    const std::string id = "0";
    float mpc_size = size.toFloat();
    mapwidget_->setPointCloudSize(id, mpc_size);
    mapwidget_->reloadPcdMap(local_points_zmin_, local_points_zmax_);
    isOk = false;
  }
  else
  {
    return;
  }  
}
void MainWindow::view2D3D_Action() {
    mode_++;
    mode_ %= 3; 
    if(mode_ == 0){
      threeDimenact->setChecked(true);
      threeDimenact_Action();
    }
    else if(mode_ == 1){
      twoDimenact->setChecked(true);
      twoDimenact_Action();
    }
    else if(mode_ == 2){
      topThreeDimenact->setChecked(true);
      topThreeDimenact_Action();
    }
    mapwidget_->reloadPcdMap(local_points_zmin_, local_points_zmax_);
}
void MainWindow::twoDimenact_Action() {
    mapwidget_->setDispalyMode(MyRenderPanel::VIEW_2D);
    view_label->setText(QString("*2D."));
    viewEnable_ = true;
    mode_ = 1;
}
void MainWindow::threeDimenact_Action() {
    mapwidget_->setDispalyMode(MyRenderPanel::VIEW_3D);
    view_label->setText(QString("*3D."));
    viewEnable_ = false;
    mode_ = 0;
}
void MainWindow::topThreeDimenact_Action() {
    mapwidget_->setDispalyMode(MyRenderPanel::VIEW_T3D);
    view_label->setText(QString("*TOP-3D."));
    viewEnable_ = false;
    mode_ = 2;
}
void MainWindow::doPcdChange_Action(const QString &actionname){
  ProjectName_ = actionname;
  QStringList paths,names;
  paths.clear();
  names.clear();
  for (auto pcd_path : pcdIdPaths)
  {
    if (pcd_path.second == "") return;
    paths = pcd_path.second.split("/");
    names = paths.back().split(".");
    QString name = names.front();
    std::cout << "name............." << name.toStdString() << std::endl;
    std::cout << "actionname......." << actionname.toStdString() << std::endl;
    if (name == actionname)
    {
      std::cout << "enter+++++++++++++++" << std::endl;
      readOnlyPcdFromProject(pcd_path.first,pcd_path.second);
      map_paths_.clear();
      break;
    }
  }
}
void MainWindow::doProjectPcdChange_Action(){
  std::cout << "-----------------" << std::endl;
  for (auto pcd_path : pcdIdPaths) {
    if (pcd_path.second == "") return;
    readOnlyPcdFromProject(pcd_path.first,pcd_path.second);
    map_paths_.clear();
    break;
  }
}
void MainWindow::pcdAll_Action(){
  if (pcdIdPaths.size() <= 0) return;
    for (auto pcd_path : pcdIdPaths)
    {
      readOnlyPcdFromProject(pcd_path.first,pcd_path.second);
    }
  map_paths_.clear();
}
void MainWindow::refreshZero_Action() {
  mapwidget_->setCameraZero();
}
void MainWindow::image_view_Action() {
  imaWidgetact->setChecked(true);
  imageWidget_Action();
}
void MainWindow::displayWidget_Action() {
  Qt::DockWidgetArea area = this->dockWidgetArea(mapToolWidget);
  if (disWidgetact->isChecked())
  {
    mapToolWidget->show();
    if (area == Qt::RightDockWidgetArea){
      r_toolButton->setArrowType(Qt::RightArrow);
    } else {
      l_toolButton->setArrowType(Qt::LeftArrow);
    }
  } else {
    mapToolWidget->hide();
    if (area == Qt::RightDockWidgetArea){
      r_toolButton->setArrowType(Qt::LeftArrow);
    } else {
      l_toolButton->setArrowType(Qt::RightArrow);
    }  
  }
}
void MainWindow::imageWidget_Action() {
  Qt::DockWidgetArea area = this->dockWidgetArea(mapToolWidget);
  if (imaWidgetact->isChecked())
  {
    imageViewWidget->show();
    if (area == Qt::RightDockWidgetArea){
      r_toolButton->setArrowType(Qt::RightArrow);
    } else {
      l_toolButton->setArrowType(Qt::LeftArrow);
    }
  } else {
    imageViewWidget->hide();
    if (area == Qt::RightDockWidgetArea){
      r_toolButton->setArrowType(Qt::LeftArrow);
    } else {
      l_toolButton->setArrowType(Qt::RightArrow);
    }  
  }
}
void MainWindow::upLoad_Action() {
  if( CurQString_ == ""  && ProjectName_ == "") {
    QMessageBox::question(this, tr("注意!"), tr("ERROR：工程名为空！！"),
                          QMessageBox::Yes);
    return;
  }

  bool exitenceflag = false;
  for (size_t i=0; i < allProjectNames.size(); i++) {
    if (ProjectName_ == allProjectNames.at(i)) {
      exitenceflag = true;
    }
  }
  QDir dir(project_vmap_);
  if (!dir.exists()) {
    QString errormessage = project_vmap_ + "不存在，请检查工程名！";
    QMessageBox::question(this, tr("提示"), errormessage,
                          QMessageBox::Yes);
    return;
    // dir.mkdir(project_vmap_);
  } //不允许主动创建工程目录

  //自动保存
  autoSaveAllProjectData();
  autoTimeSaveProjectData();
  Q_EMIT uploadMapInfoSignal(CurQString_);
}
void MainWindow::testMap_Action() {
  bool isTest = false;
  for (int i = 0; i < Vm_T::Type_Num; i++) {
    if (!mapwidget_->getVectorType((Vm_T)i).empty()) {
      isTest = true;
      break;
    }
  }
  if (isTest) {
    int reply = QMessageBox::question(this, tr("路线调试提醒"), tr("调试前请确认是否已保存最新地图文件？"),
                                QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::No) {
      QString message = "已自动保存地图文件至" + filePath_vmap_ + "！";
      std::string test_vmap = filePath_vmap_.toStdString();
      mapwidget_->writeVmapFiles(test_vmap);
      QMessageBox::question(this, tr("提示"), message,
                              QMessageBox::Yes);
    }
    std::string filepath = filePath_vmap_.toStdString();
    test_ = new CTest();
    test_->test(filepath);
    
    TestResultDialog dialog(test_,this);
    dialog.readDialogShowPose();
    if (dialog.exec() == QDialog::Rejected) {
      dialog.setDialogClosePose();
      return;
    }
    dialog.setDialogClosePose();
    if (test_) {
        delete test_;
        test_ = nullptr;
    }
  } 
}
void MainWindow::docksview_Act_triggered() {
  if(ProjectName_ == "" && dockviewdialog_->projectname_combox->isEnabled()) {
    QMessageBox::question(this, tr("项目警告"), tr("项目为空，请加载一个地图项目!"),
                          QMessageBox::Yes);
    // return;
  } else if (mapname_ != ProjectName_) {
    updateVehicleData(ProjectName_);
    mapname_ = ProjectName_;
  }
  if (robots_info_.size() == 0) {
    int reply = QMessageBox::question(this, tr("车辆警告"), tr("该项目车辆均不在线，是否继续？"),
                          QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::No) {
      return;
    }
  }
  dockviewdialog_->readDialogShowPose();
  dockviewdialog_->show();
}
void MainWindow::exit_Action() {
    int reply = QMessageBox::question(this, tr("关闭提示"), tr("确定关闭窗口吗？"),
                                QMessageBox::Yes | QMessageBox::No);
      if (reply == QMessageBox::No) {
        return;
      } else {
        mapwidget_->closeMysqlVehical();
        this->close();
      }
}
void MainWindow::create_batch_act_Action() {
  if (pubToken_.empty())
  {
    QString errormessage = "连接失败，请重新登录用户获取列表。";
    QMessageBox::question(this, tr("提示"), errormessage,
                          QMessageBox::Yes);
    return;
  }
  parkNameIdItemShow();
  pubdialog_->create_checkbox->setChecked(true);
  pubdialog_->modify_checkbox->setChecked(false);
  pubdialog_->delete_checkbox->setChecked(false);
  pubdialog_->show();
}
void MainWindow::modify_batch_act_Action() {
  if (pubToken_.empty())
  {
    QString errormessage = "连接失败，请重新登录用户获取列表。";
    QMessageBox::question(this, tr("提示"), errormessage,
                          QMessageBox::Yes);
    return;
  }
  if (listParksIdName_.size() == 0) {
    parkNameIdItemShow();
  }
  pubdialog_->create_checkbox->setChecked(false);
  pubdialog_->modify_checkbox->setChecked(true);
  pubdialog_->delete_checkbox->setChecked(false);
  pubdialog_->show();
}
void MainWindow::delete_batch_act_Action() {
  if (pubToken_.empty())
  {
    QString errormessage = "连接失败，请重新登录用户获取列表。";
    QMessageBox::question(this, tr("提示"), errormessage,
                          QMessageBox::Yes);
    return;
  }
  if (listParksIdName_.size() == 0) {
    parkNameIdItemShow();
  }
  pubdialog_->create_checkbox->setChecked(false);
  pubdialog_->modify_checkbox->setChecked(false);
  pubdialog_->delete_checkbox->setChecked(true);
  pubdialog_->show();
}
void MainWindow::signaldelete_batch_act_Action() {
  
}
void MainWindow::closeEvent(QCloseEvent *event) {
  QMessageBox::StandardButton resBtn =
      QMessageBox::question(this, tr("close"), tr("Are you sure?\n"),
                            QMessageBox::Cancel | QMessageBox::Yes);
  if (resBtn != QMessageBox::Yes) {
    event->ignore();
  } else {
    mapwidget_->closeMysqlVehical();
    event->accept();
  }
}
void MainWindow::repairMysql_Act_clicked() {
  //--修复数据库数据信息
  Q_EMIT modifyMysqlInfo();
}
void MainWindow::deleteMysql_Act_clicked() {
  //--删除单个或多个数据库数据信息
  MysqlDeleteDialog dialog(this);
  if (allProjectNames.size() != 0) {
    for (auto proname :  allProjectNames) {
      dialog.projectname_combox->addItem(proname);
    } 
  }
  dialog.readDialogShowPose();
  if (dialog.exec() == QDialog::Rejected) {
    dialog.setDialogClosePose();
    return;
  }
  dialog.setDialogClosePose();

  QString projectName = dialog.getComboBoxText();
  qDebug() << "删除的项目: " << projectName;
  Q_EMIT deleteMysqlInfo(projectName);
}
void MainWindow::createMysql_Act_clicked() {
  //创建新的数据库信息
  Q_EMIT createMysqlInfo();
}
void MainWindow::framePose_Act_triggered() {
  QString fileName = QFileDialog::getOpenFileName(this, tr("打开点数据文件"), filePath_frame_,
                                      tr("ALL(*.*);;CSV(*.json)"));
  if (fileName.isEmpty()) return;
  std::string fileStr = fileName.toStdString();
  mapwidget_->showPoint(fileStr);
  // updateDisplayMapElement();
}

//--------toggled--------
void MainWindow::rBtn_addNone_Toggled() {
  if (rBtn_addNone->isChecked()) {  //查看模式(Ctrl+R)
    mapwidget_->setWorkMode(MyRenderPanel::VIEW);
    updateQWidgetEnableStatus();
    status_label->setText(tr("查看！"));
  }
  updateDisplayMapElement();
}
void MainWindow::rBtn_addNewmap_Toggled(bool state) {
  if (state == true) {  // 新建模式(Ctrl+N)
    {
      workWidget->setParent(NULL);
      modeLayout->removeWidget(workWidget);
    }
    workWidget = (QWidget *)creatWorkDesk();
    modeLayout->addWidget(workWidget,1,0,1,4);

    mapwidget_->setWorkMode(MyRenderPanel::DRAW);  // this is necessary ,TO DO 4 ->2
    updateQWidgetEnableStatus();
    disWidgetact->setChecked(true);
    displayWidget_Action();
    status_label->setText(tr("画图！"));
  }
  
  updateDisplayMapElement();
}
void MainWindow::rBtn_pointCorrection_Toggled(bool state) {
  if (state == true) {  // 修正模式(Ctrl+E)
    if ((lanelet2map_mode_checkbox->isChecked() &&
         vmap_mode_checkbox->isChecked()) ||
        (!lanelet2map_mode_checkbox->isChecked() &&
         !vmap_mode_checkbox->isChecked())) {
      QMessageBox::question(this, tr("提示"),
                            tr("修正模式,必须并且只能对一种地图进行修改！"),
                            QMessageBox::Yes);
      rBtn_pointCorrection->setChecked(false);
      return;
    }
    if (lanelet2map_mode_checkbox->isChecked()) {
      {
        workWidget->setParent(NULL);
        modeLayout->removeWidget(workWidget);
      }
      workWidget = (QWidget *)creatLaneletWorkDesk();
    }

    modeLayout->addWidget(workWidget,1,0,1,4);
    
    mapwidget_->setWorkMode(MyRenderPanel::DRAW);  // this is necessary ,TO DO 4 ->2
    updateQWidgetEnableStatus();
    disWidgetact->setChecked(true);
    displayWidget_Action();
    choosevmaps_pBtn->setEnabled(true);
    chooselanelettypes_pBtn->setEnabled(true);
    status_label->setText(tr("修正！"));
  }
  updateDisplayMapElement();
}

//-----indexchanged-----
void MainWindow::maptypes_comboBox_IndexChanged() {
    updateVmapcomboBoxInfo();
    if (rBtn_addNewmap->isChecked()) {
      if (vmap_mode_checkbox) {
        int type_id = getCurrentVmapTypeID();
        int type_num_id = mapwidget_->getRandEmptyTypeNumId(type_id);
        maptype_spinBox->setValue(type_num_id);
      }
      if (lanelet2map_mode_checkbox->isChecked()) {
        std::vector<lanelet::Point3d>().swap(point3d_vec_);
        std::vector<int>().swap(lanelet2_ids_);
        // point3d_vec_.clear();
        // lanelet2_ids_.clear();
        current_point3d_id_ = 0;
      }  
    } 
    updateDisplayMapElement();
}
/*vmap地图，下拉插件maptypetype_comboBox，index变化时对应的不同地图元素点的属性值的更新
*/
void MainWindow::maptypetype_comboBox_IndexChanged(int index) {
    if (index >= 0) {
        int type_id = getCurrentVmapTypeID();
        int type_num_id = getCurrentVmapTypeNumID();
        Way_Point_ wp;
        if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
            switch (type_id) {
                case (int)Vm_T::Lane: {
                PRO_LANE_TEST test(wp.property_type.P1_TYPE);
                test.pro_type.TYPE1 = index;
                wp.property_type.P1_TYPE = test.waytype;
                } break;
                case (int)Vm_T::Signal: {
                PRO_SIGNAL_TEST sigtest(wp.property_type.P1_TYPE);
                sigtest.pro_type.TYPE1 = index;
                wp.property_type.P1_TYPE = sigtest.waytype;
                } break;
                case (int)Vm_T::Gate: {
                PRO_GATE_TEST gatetest(wp.property_type.P1_TYPE);
                gatetest.pro_type.TYPE1 = index;
                wp.property_type.P1_TYPE = gatetest.waytype;
                } break;
                case (int)Vm_T::StopPoint: {
                PRO_STOPPOINT_TEST stoptest(wp.property_type.P1_TYPE);
                stoptest.pro_type.TYPE1 = index;
                wp.property_type.P1_TYPE = stoptest.waytype;
                } break;
                case (int)Vm_T::DeceZone: {
                PRO_DECEZONE_TEST decetest(wp.property_type.P1_TYPE);
                decetest.pro_type.TYPE1 = index;
                wp.property_type.P1_TYPE = decetest.waytype;
                } break;
                case (int)Vm_T::ClearArea: {
                PRO_CLEARAREA_TEST cleartest(wp.property_type.P1_TYPE);
                cleartest.pro_type.TYPE1 = index;
                wp.property_type.P1_TYPE = cleartest.waytype;
                } break;
                case (int)Vm_T::AttributeArea: {
                PRO_LANE_TEST test(wp.property_type.P1_TYPE);
                test.pro_type.TYPE1 = index;
                wp.property_type.P1_TYPE = test.waytype;
                } break;
                default:
                break;
        }
        mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                            current_point_index_, wp);
    }
    updateCoordinatesProDisplay();
  }
}
/*vmap地图，下拉插件waytypetype_comboBox，index变化时对应的不同地图元素点的属性值的更新
*/
void MainWindow::waytypetype_comboBox_IndexChanged(int index) {
  if (index >= 0) {
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();
    Way_Point_ wp;
    if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
      switch (type_id) {
        case (int)Vm_T::Lane: {
          PRO_LANE_TEST test(wp.property_type.P1_TYPE);
          test.pro_type.TYPE2 = index;
          wp.property_type.P1_TYPE = test.waytype;
        } break;
        case (int)Vm_T::Signal: {
          PRO_SIGNAL_TEST sigtest(wp.property_type.P1_TYPE);
          sigtest.pro_type.TYPE2 = index;
          wp.property_type.P1_TYPE = sigtest.waytype;
        } break;
        case (int)Vm_T::Gate: {
          PRO_GATE_TEST gatetest(wp.property_type.P1_TYPE);
          gatetest.pro_type.TYPE2 = index;
          wp.property_type.P1_TYPE = gatetest.waytype;
        } break;
        case (int)Vm_T::AttributeArea: {
          PRO_LANE_TEST test(wp.property_type.P1_TYPE);
          test.pro_type.TYPE2 = index;
          wp.property_type.P1_TYPE = test.waytype;
        } break; 
        default:
          break;
      }
      mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                          current_point_index_, wp);
    }
    updateCoordinatesProDisplay();
  }
}
void MainWindow::bypasstype_comboBox_IndexChanged(int index) {
  if (index >= 0) {
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();
    Way_Point_ wp;
    if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
      switch (type_id) {
        case (int)Vm_T::Lane: {
          PASS_LANE_TEST test(wp.property_type.P3_PASS);
          test.pass_type.TYPE1 = index;
          wp.property_type.P3_PASS = test.pass;
        } break;
        case (int)Vm_T::Signal: {
          PASS_SIGNAL_TEST sigtest(wp.property_type.P3_PASS);
          sigtest.pass_type.TYPE1 = index;
          wp.property_type.P3_PASS = sigtest.pass;
        } break;
        case (int)Vm_T::Gate: {
          PASS_GATE_TEST gatetest(wp.property_type.P3_PASS);
          gatetest.pass_type.TYPE1 = index;
          wp.property_type.P3_PASS = gatetest.pass;
        } break;
        case (int)Vm_T::AttributeArea: {
          PASS_LANE_TEST test(wp.property_type.P3_PASS);
          test.pass_type.TYPE1 = index;
          wp.property_type.P3_PASS = test.pass;
        } break;
        default:
          break;
      }
      mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                          current_point_index_, wp);
    }
    updateCoordinatesProDisplay();
  }
}
void MainWindow::traveldirectiontype_comboBox_IndexChanged(int index) {
  if (index >= 0) {
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();
    Way_Point_ wp;
    if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
      switch (type_id) {
        case (int)Vm_T::Lane: {
          TRAV_LANE_TEST test(wp.property_type.P2_TRAV);
          test.travel_type.TYPE1 = index;
          wp.property_type.P2_TRAV = test.travel;
        } break;
        case (int)Vm_T::AttributeArea: {
          TRAV_LANE_TEST test(wp.property_type.P2_TRAV);
          test.travel_type.TYPE1 = index;
          wp.property_type.P2_TRAV = test.travel;
        } break;
        default:
          break;
      }
      mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                          current_point_index_, wp);
    }
    updateCoordinatesProDisplay();
  }
}
void MainWindow::traveltype_comboBox_IndexChanged(int index) {
  if (index >= 0) {
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();
    Way_Point_ wp;
    if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
      switch (type_id) {
        case (int)Vm_T::Lane: {
          TRAV_LANE_TEST test(wp.property_type.P2_TRAV);
          test.travel_type.TYPE3 = index;
          wp.property_type.P2_TRAV = test.travel;
        } break;
        case (int)Vm_T::AttributeArea: {
          TRAV_LANE_TEST test(wp.property_type.P2_TRAV);
          test.travel_type.TYPE3 = index;
          wp.property_type.P2_TRAV = test.travel;
        } break;
        default:
          break;
      }
      mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                          current_point_index_, wp);
    }
    updateCoordinatesProDisplay();
  }
}
void MainWindow::travelslopedtype_comboBox_IndexChanged(int index) {
  if (index >= 0) {
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();
    Way_Point_ wp;
    if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
      switch (type_id) {
        case (int)Vm_T::Lane: {
          TRAV_LANE_TEST test(wp.property_type.P2_TRAV);
          test.travel_type.TYPE4 = index;
          wp.property_type.P2_TRAV = test.travel;
        } break;
        case (int)Vm_T::AttributeArea: {
          TRAV_LANE_TEST test(wp.property_type.P2_TRAV);
          test.travel_type.TYPE4 = index;
          wp.property_type.P2_TRAV = test.travel;
        } break;
        default:
          break;
      }
      mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                          current_point_index_, wp);
    }
    updateCoordinatesProDisplay();
  }
}
void MainWindow::roomtype_comboBox_IndexChanged(int index) {
  if (index >= 0) {
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();
    Way_Point_ wp;
    if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
      switch (type_id) {
        case (int)Vm_T::Lane: {
          PRO_LANE_TEST test(wp.property_type.P1_TYPE);
          test.pro_type.TYPE3 = index;
          wp.property_type.P1_TYPE = test.waytype;
        } break;
        case (int)Vm_T::AttributeArea: {
          PRO_LANE_TEST test(wp.property_type.P1_TYPE);
          test.pro_type.TYPE3 = index;
          wp.property_type.P1_TYPE = test.waytype;
        } break;
        default:
          break;
      }
      mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                          current_point_index_, wp);
    }
    updateCoordinatesProDisplay();
  }
}
void MainWindow::linktype_comboBox_IndexChanged(int index) {
  if (index >= 0) {
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();
    Way_Point_ wp;
    if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
      wp.ltype_type.LINKTYPE = index;
      mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                          current_point_index_, wp);
      updateCoordinatesProDisplay();
    }
  }
}
void MainWindow::maptype_spinBox_valueChanged(int index) {
  if (!lanelet2map_mode_checkbox->isChecked()) {
    maptype_spinBox->setValue(index);
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();
    int number = mapwidget_->getCurrentTypeNumIdNumber(type_id, type_num_id);
    current_point_index_ = number <= 0 ? 0 : number - 1;
    updateDisplayMapElement();
  }
}
void MainWindow::gidnumb_spinbox_valueChanged(int index) {
  setgiddialog_->gidnumb_spinbox->setValue(index);
  for (size_t i = 0; i < listGidInfos_.size(); i++)
  {
    QMap<QString, QList<QString>> origin = listGidInfos_.at(i);
    QMap<QString, QList<QString>>::iterator it = origin.begin();
    QString numb = it.key();
    if (stoi(numb.toStdString().c_str()) == index)
    {
      QList<QString> infos = it.value();
      QStringList explaininfo = infos.at(infos.size()-1).split("-");
      if (explaininfo.at(1) == "Signal")
      {
        setgiddialog_->setEditEnabled(false,true);
        setgiddialog_->gidsignal_edit->setText(infos.at(0));
      }
      else {
        setgiddialog_->setEditEnabled(true,false);
        setgiddialog_->gidwebid_edit->setText(infos.at(0));
        setgiddialog_->gidlora_edit->setText(infos.at(1));
      }
      break;
    }
  }
}
void MainWindow::lanelet2map_mode_checkbox_clicked(bool status) {
  if (status == true) {
    if (vmap_mode_checkbox->isChecked() && rBtn_pointCorrection->isChecked()) {
      QMessageBox::question(this, tr("提示"),
                            tr("修正模式,必须并且只能对一种地图进行修改！"),
                            QMessageBox::Yes);
      lanelet2map_mode_checkbox->setChecked(false);
      return;
    }
    if (rBtn_pointCorrection->isChecked()) {
      {
        workWidget->setParent(NULL);
        modeLayout->removeWidget(workWidget);
      }
      workWidget = (QWidget *)creatLaneletWorkDesk();
      modeLayout->addWidget(workWidget,1,0,1,4);
    }
    chooselanelettypes_pBtn->setChecked(true);
    updateQWidgetEnableStatus();
  }
}
void MainWindow::vmap_mode_checkbox_clicked(bool status) {
  if (status == true) {
    if (lanelet2map_mode_checkbox->isChecked() && rBtn_pointCorrection->isChecked()) {
      QMessageBox::question(this, tr("提示"),
                            tr("修正模式,必须并且只能对一种地图进行修改！"),
                            QMessageBox::Yes);
      vmap_mode_checkbox->setChecked(false);
      return;
    }
    if (rBtn_pointCorrection->isChecked()) {
      {
        workWidget->setParent(NULL);
        modeLayout->removeWidget(workWidget);
        // workWidget = nullptr;
      }
      workWidget = (QWidget *)creatWorkDesk();
      modeLayout->addWidget(workWidget,1,0,1,4);
    }
    updateQWidgetEnableStatus();
  }
}
void MainWindow::delBackpBtn() {
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();

    mapwidget_->setTypeNumIdDelBackVmapPoint(type_id,type_num_id);
    updateDisplayMapElement();
}

//------------editFinished------------
// void MainWindow::pcdzmin_edit_Finished() {
//   double value = atof(pcdzmin_edit->text().toStdString().c_str());
//   if (value <= global_points_zmin_) {
//     value = global_points_zmin_;
//   }
//   local_points_zmin_ = value;
// }
// void MainWindow::pcdzmax_edit_Finished() {
//   double value = atof(pcdzmax_edit->text().toStdString().c_str());
//   if (value >= global_points_zmax_) {
//     value = global_points_zmax_;
//   }
//   local_points_zmax_ = value;
// }
void MainWindow::localx_edit_Finished() {
  if (lanelet2map_mode_checkbox->isChecked() && rBtn_pointCorrection->isChecked()) {
    lanelet::Point3d pt;
    if (mapwidget_->findPoint3dWithId(current_point3d_id_, pt)) {
      pt.x() = atof(localx_edit->text().toStdString().c_str());
      mapwidget_->resetPoint3d(pt);
    }
  } else {
    Way_Point_ wp;
    if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
      int type_id = getCurrentVmapTypeID();
      int type_num_id = getCurrentVmapTypeNumID();
      wp.point.x = atof(localx_edit->text().toStdString().c_str());
      mapwidget_->localCoordinatesToGps(wp);
      mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                          current_point_index_, wp);
    }
  }  
  updateDisplayMapElement();
}
void MainWindow::localy_edit_Finished() {
   if (lanelet2map_mode_checkbox->isChecked() && rBtn_pointCorrection->isChecked()) {
    lanelet::Point3d pt;
    if (mapwidget_->findPoint3dWithId(current_point3d_id_, pt)) {
      pt.y() = atof(localy_edit->text().toStdString().c_str());
      mapwidget_->resetPoint3d(pt);
    }
  } else {
    Way_Point_ wp;
    if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
      int type_id = getCurrentVmapTypeID();
      int type_num_id = getCurrentVmapTypeNumID();
      wp.point.y = atof(localy_edit->text().toStdString().c_str());
      mapwidget_->localCoordinatesToGps(wp);
      mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                          current_point_index_, wp);  
    }
  }
  updateDisplayMapElement();
}
void MainWindow::localz_edit_Finished() {
  if (lanelet2map_mode_checkbox->isChecked() &&
      rBtn_pointCorrection->isChecked()) {
    lanelet::Point3d pt;
    if (mapwidget_->findPoint3dWithId(current_point3d_id_, pt)) {
      pt.z() = atof(localz_edit->text().toStdString().c_str());
      mapwidget_->resetPoint3d(pt);
    }
  } else {
    Way_Point_ wp;
    if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
      int type_id = getCurrentVmapTypeID();
      int type_num_id = getCurrentVmapTypeNumID();
      wp.point.z = atof(localz_edit->text().toStdString().c_str());
      wp.satfix.altitude = wp.point.z;
      mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                          current_point_index_, wp);
    }
  }
  updateDisplayMapElement();
}
void MainWindow::localw_edit_Finished() {
  // if (lanelet2map_mode_checkbox->isChecked() && rBtn_pointCorrection->isChecked()) {
  //   lanelet::Point3d pt;
  //   if (mapwidget_->findPoint3dWithId(current_point3d_id_, pt)) {
  //     pt.y() = atof(localy_edit->text().toStdString().c_str());
  //     mapwidget_->resetPoint3d(pt);
  //   }
  // } else {
  //   Way_Point_ wp;
  //   if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
  //     int type_id = getCurrentVmapTypeID();
  //     int type_num_id = getCurrentVmapTypeNumID();
  //     wp.point.y = atof(localy_edit->text().toStdString().c_str());
  //     mapwidget_->localCoordinatesToGps(wp);
  //     mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
  //                                         current_point_index_, wp);  
  //   }
  // }
  // updateDisplayMapElement();
}
void MainWindow::latitude_edit_Finished() {
  if (lanelet2map_mode_checkbox->isChecked() &&
      rBtn_pointCorrection->isChecked()) {
    lanelet::Point3d pt;
    if (mapwidget_->findPoint3dWithId(current_point3d_id_, pt)) {
      LatLonInfo latlon_info;
      latlon_info.latitude =
          atof(latitude_edit->text().toStdString().c_str());
      latlon_info.longitude =
          atof(longitude_edit->text().toStdString().c_str());
      mapwidget_->gpsCoordinatesToLocal(latlon_info, pt);
    }
  } else {
    Way_Point_ wp;
    if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
      int type_id = getCurrentVmapTypeID();
      int type_num_id = getCurrentVmapTypeNumID();
      wp.satfix.latitude =
          atof(latitude_edit->text().toStdString().c_str());
      mapwidget_->gpsCoordinatesToLocal(wp);
      mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                          current_point_index_, wp);
    }
  }
  updateDisplayMapElement();
}
void MainWindow::longitude_edit_Finished() {
  Way_Point_ wp;
  if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();
    wp.satfix.longitude =
        atof(longitude_edit->text().toStdString().c_str());
    mapwidget_->gpsCoordinatesToLocal(wp);
    mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                        current_point_index_, wp);
    updateDisplayMapElement();
  }
}
void MainWindow::limitvelocity_edit_Finished() {
  Way_Point_ wp;
  if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();
    switch (type_id) {
      case (int)Vm_T::Lane: {
        wp.limit_vel =
            atof(limitvelocity_edit->text().toStdString().c_str());
      } break;
      case (int)Vm_T::Elevator: {
        int index = atoi(limitvelocity_edit->text().toStdString().c_str());
        PRO_ELEVATOR_TEST test(wp.property_type1.P_TYPE);
        test.pro_type.TYPE1 = index;
        wp.property_type1.P_TYPE = test.waytype;
      } break;
      case (int)Vm_T::AttributeArea: {
        wp.limit_vel = atof(limitvelocity_edit->text().toStdString().c_str());
      } break;
      default:
        break;
    }
    mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                        current_point_index_, wp);
    updateCoordinatesProDisplay();
  }
}
void MainWindow::linktypenum_edit_Finished() {
  Way_Point_ wp;
  if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();
    wp.ltype_type.LINKNUM =
        atof(linktypenum_edit->text().toStdString().c_str());
    mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                        current_point_index_, wp);
    updateCoordinatesProDisplay();
  }
}
void MainWindow::laneletid_edit_Finished() {
  Way_Point_ wp;
  if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();
    switch (type_id) {
      case (int)Vm_T::Lane: {
        wp.lanelet_id = atof(laneletid_edit->text().toStdString().c_str());
      } break;
      case (int)Vm_T::Elevator: {
        std::string str = laneletid_edit->text().toStdString();
        if (str.size() > 3) {
          QMessageBox::question(this, tr("提示"), tr("最多存三个字符！"),
                                QMessageBox::Yes);
          laneletid_edit->setText("");
          return;
        }
        char ele_level[3] = "";
        char str_c[str.size()];
        strcpy(str_c, str.c_str());
        // //#pragma omp parallel for
        for (int i = 0; i < str.size(); i++) {
          // std::cout<<"str:"<<str_c[i]<<std::endl;
          // if (str_c[i] == '\0') break;
          ele_level[i] = str_c[i];
        }
        wp.property_type1.P_PASS[0] = ele_level[0];
        wp.property_type1.P_PASS[1] = ele_level[1];
        wp.property_type1.P_PASS[2] = ele_level[2];
      }
      case (int)Vm_T::AttributeArea: {
          wp.lanelet_id = atof(laneletid_edit->text().toStdString().c_str());
        } break;
      default:
        break;
    }
    mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                        current_point_index_, wp);
    updateCoordinatesProDisplay();
  }
}
void MainWindow::countcell_edit_Finished() {
  uint32_t grid_size = count_edit->text().toInt();
  mapwidget_->setGridSize(grid_size);
  updateGridParamEdit();
}
void MainWindow::celllength_edit_Finished() {
  float grid_length = atof(length_edit->text().toStdString().c_str());
  mapwidget_->setGridCellLength(grid_length);
  updateGridParamEdit();
}
void MainWindow::cellwidth_edit_Finished() {
  float grid_width = atof(width_edit->text().toStdString().c_str());
  mapwidget_->setGridLineWidth(grid_width);
  updateGridParamEdit();
}
void MainWindow::regularid_edit_Finished() {
  bool isHave_regu = false;
  bool getRef1 = false;
  bool getRef2 = false;
  bool getRef3 = false;
  int regul_id = atoi(regularid_edit->text().toStdString().c_str());
  regulardialog_->refid_edit_1->setText(QString(""));
  regulardialog_->refidrole_edit_1->setText(QString(""));
  regulardialog_->refid_edit_2->setText(QString(""));
  regulardialog_->refidrole_edit_2->setText(QString(""));
  regulardialog_->refid_edit_3->setText(QString(""));
  regulardialog_->refidrole_edit_3->setText(QString(""));
  regulardialog_->refid_edit_4->setText(QString(""));
  regulardialog_->refidrole_edit_4->setText(QString(""));
  regulardialog_->refid_edit_5->setText(QString(""));
  regulardialog_->refidrole_edit_5->setText(QString(""));
  std::vector<reguinfo_list> regularlists = mapwidget_->getRegularInfoList();
  ////#pragma omp parallel for
  for (auto reg : regularlists) {
    if (reg.reglists.Regelement_id == regul_id) {
      isHave_regu = true;
      deletregular_pBtn->setEnabled(true);
      editregular_pBtn->setEnabled(true);
      regulardialog_->refid_edit_1->setText(
          QString::number(reg.regmemberid));
      regulardialog_->refidrole_edit_1->setText(
          QString::fromStdString(reg.Subtype));
      // for show maneuver, element
      int index = setCurrentRegularComboxIndex(reg.reglists.maneuver, 0);
      if (index != -1) {
        regulardialog_->maneuver_comboBox->setCurrentIndex(
            index % RegulatoryManeuverNUM);
      }
      index = setCurrentRegularComboxIndex(reg.reglists.element, 1);
      if (index != -1) {
        regulardialog_->element_comboBox->setCurrentIndex(
            index % RegulatoryElementsNUM);
      }
      // for refid2-5
      ////#pragma omp parallel for
      for (auto reflist : reg.reglists.refers_list) {
        if (!getRef1 && reflist.refer_id > 0) {
          getRef1 = true;
          regulardialog_->refid_edit_2->setText(
              QString::number(reflist.refer_id));
          regulardialog_->refidrole_edit_2->setText(
              QString::fromStdString(reflist.referName));
        } else if (getRef1 && !getRef2 && reflist.refer_id > 0) {
          getRef2 = true;
          regulardialog_->refid_edit_3->setText(
              QString::number(reflist.refer_id));
          regulardialog_->refidrole_edit_3->setText(
              QString::fromStdString(reflist.referName));
        } else if (getRef1 && getRef2 && !getRef3 && reflist.refer_id > 0) {
          regulardialog_->refid_edit_4->setText(
              QString::number(reflist.refer_id));
          regulardialog_->refidrole_edit_4->setText(
              QString::fromStdString(reflist.referName));
        } else if (getRef1 && getRef2 && getRef3 && reflist.refer_id > 0) {
          regulardialog_->refid_edit_5->setText(
              QString::number(reflist.refer_id));
          regulardialog_->refidrole_edit_5->setText(
              QString::fromStdString(reflist.referName));
        }
      }
      return;
    }
  }
  if (!isHave_regu) {
    QMessageBox::question(this, tr("提示"),
                          tr("当前交规不存在,不能添加交规信息!"),
                          QMessageBox::Yes);
    regularid_edit->setText(QString("--"));
    regularid_edit->setEnabled(false);
    return;
  }
}
void MainWindow::addregularinfo() {
  int regul_id = atoi(regularid_edit->text().toStdString().c_str());
  ////#pragma omp parallel for
  std::vector<reguinfo_list> regularlists = mapwidget_->getRegularInfoList();
  for (auto &reg : regularlists) {
    if (reg.reglists.Regelement_id == regul_id) {
      reg.reglists.refers_list.clear();
      regulardialog_->refidrole_edit_2->setText(QString(""));
      regulardialog_->refidrole_edit_3->setText(QString(""));
      regulardialog_->refidrole_edit_4->setText(QString(""));
      regulardialog_->refidrole_edit_5->setText(QString(""));
      refer_list reflist;
      std::string ref_role;
      int ref_id2 =
          atoi(regulardialog_->refid_edit_2->text().toStdString().c_str());
      std::cout << "id2: " << ref_id2 << std::endl;
      if (ref_id2 > 0) {
        if (!mapwidget_->haveCurrentRefIdRole(ref_id2, ref_role)) {
          regulardialog_->refid_edit_2->setText(QString(""));
        } else {
          regulardialog_->refidrole_edit_2->setText(
              QString::fromStdString(ref_role));
          reflist.refer_id = ref_id2;
          reflist.referName = ref_role;
          reg.reglists.refers_list.push_back(reflist);
        }
      }
      int ref_id3 =
          atoi(regulardialog_->refid_edit_3->text().toStdString().c_str());
      if (ref_id3 > 0) {
        if (!mapwidget_->haveCurrentRefIdRole(ref_id3, ref_role)) {
          regulardialog_->refid_edit_3->setText(QString(""));
        } else {
          regulardialog_->refidrole_edit_3->setText(
              QString::fromStdString(ref_role));
          reflist.refer_id = ref_id3;
          reflist.referName = ref_role;
          reg.reglists.refers_list.push_back(reflist);
        }
      }
      int ref_id4 =
          atoi(regulardialog_->refid_edit_4->text().toStdString().c_str());
      if (ref_id4 > 0) {
        if (!mapwidget_->haveCurrentRefIdRole(ref_id4, ref_role)) {
          regulardialog_->refid_edit_4->setText(QString(""));
        } else {
          regulardialog_->refidrole_edit_4->setText(
              QString::fromStdString(ref_role));
          reflist.refer_id = ref_id4;
          reflist.referName = ref_role;
          reg.reglists.refers_list.push_back(reflist);
        }
      }
      int ref_id5 =
          atoi(regulardialog_->refid_edit_5->text().toStdString().c_str());
      if (ref_id5 > 0) {
        if (!mapwidget_->haveCurrentRefIdRole(ref_id5, ref_role)) {
          regulardialog_->refid_edit_5->setText(QString(""));
        } else {
          regulardialog_->refidrole_edit_5->setText(
              QString::fromStdString(ref_role));
          reflist.refer_id = ref_id5;
          reflist.referName = ref_role;
          reg.reglists.refers_list.push_back(reflist);
        }
      }
      reg.reglists.maneuver =
          regulardialog_->maneuver_comboBox->currentText().toStdString();
      reg.reglists.element =
          regulardialog_->element_comboBox->currentText().toStdString();
      mapwidget_->createNewRegularInfoList(regularlists);
      return;
    }
  }
}
void MainWindow::tags_edit_editingFinished() {
  lanelet::LineString3d ls;
  lanelet::Lanelet ll;
  if (mapwidget_->findLinestringWithId(lanelet2_ids_.back(), ls) ||
      mapwidget_->findLaneletWithId(lanelet2_ids_.back(), ll)) {
    int id = ll.id() > 0 ? ll.id() : ls.id();
    tag_list taglist;
    tag tag;
    std::string none;
    std::vector<tag_list> taginfolists = mapwidget_->getTagInfoList();
    tag.k = taginfodialog_->key_edit_1->text().toStdString();
    tag.v = taginfodialog_->value_edit_1->text().toStdString();
    if (tag.k != none && tag.v != none) {
      taglist.tags.push_back(tag);
    }
    tag.k = taginfodialog_->key_edit_2->text().toStdString();
    tag.v = taginfodialog_->value_edit_2->text().toStdString();
    if (tag.k != none && tag.v != none) {
      taglist.tags.push_back(tag);
    }
    bool isbeing = false;
    if (taglist.tags.size() > 0) {
      ////#pragma omp parallel for
      for (auto &taglist_value : taginfolists) {
        if (taglist_value.id == id) {
          taglist_value.tags.clear();
          taglist_value.tags.assign(taglist.tags.begin(), taglist.tags.end());
          isbeing = true;
          return;
        }
      }
      if (!isbeing) {
        taglist.id = id;
        taginfolists.push_back(taglist);
        mapwidget_->createNewTagInfoList(taginfolists);
      }
    }
  }
}
//选择下拉地图桩点信息
void MainWindow::choosePullMapDocksView() {
  if (ProjectName_ == "") {
    return;
  }
  mapwidget_->seceneDocksRemove();
  std::string pullname = ProjectName_.toStdString();
  bool dockresult = mapwidget_->getdockViewInfo(pullname);
  if (dockresult) {
    mapwidget_->udpateShowDocksData();
  }
  docksremove_pbtn->setEnabled(true);
}
//补图更新上传桩点数据信息
void MainWindow::pushNewestDockInfo(QString project) {
    if (project == "") {
      return;
    }
    QString path_str = QString::fromStdString(HOME_STR) + "/maptool-candela/CTI_MAP_PROJECT/" + \
                      project + "/DATA/" + project + "/dock/api.csv";
    std::vector<std::pair<std::string, std::vector<double>>> uploadDockInfos;
    // bool uploadRsult;
    QFileInfo fileINfo(path_str);
    //文件是否存在
    if (fileINfo.isFile())
    {
      //读取桩点文件
      mapwidget_->readNewestDockFile(path_str,uploadDockInfos);
      //判断是否上传成功
      if(mapwidget_->uploadDockById(uploadDockInfos)) {
          qDebug() << "桩点上传成功";
      }
    }
    else {
      qDebug() << "桩点文件不存在";
      // return;
    }
    //清除dock文件
    std::string str = "rm -rf " + HOME_STR + "/maptool-candela/CTI_MAP_PROJECT/" + \
                      project.toStdString() + "/DATA/" + project.toStdString() + "/dock";
    if (!std::system(str.c_str())) qDebug() << "桩点文件删除成功";
}
//更新车辆数据
void MainWindow::getAllProjectName() {
  allProjectNames.clear();
  allProjectNames = choose_map_->getProjectName();
  if (allProjectNames.size() != 0) {
    for (auto proname :  allProjectNames)
    {
      dockviewdialog_->projectname_combox->addItem(proname);
    } 
  }
}
void MainWindow::updateVehicleData(QString mapname) {
  if (mapname_ != mapname) {
    mapname_ = mapname;
    map_name = mapname;
  }
  
  robots_info_.clear();
  robots_info_.insert(std::make_pair(mapname.toStdString(),mapwidget_->getCurrentRobotId(mapname)));
  dockviewdialog_->vehicleinfo_combox->clear();
  dockviewdialog_->vehicleinfo_combox->addItem(tr("全部"));
  if (robots_info_.size() != 0) {
      for (auto robot_info : robots_info_) {
        for (auto robot_i : robot_info.second) {
            // QString robotname = QString::fromStdString(robot_i);
          dockviewdialog_->vehicleinfo_combox->addItem(robot_i.c_str());
        }
        for (size_t i = 0; i < allProjectNames.size(); i++) {
          QString proname = chooseProjectNameIndex(i);
          if (proname == QString::fromStdString(robot_info.first))
          {
            dockviewdialog_->projectname_combox->setCurrentIndex(i);
            dockviewdialog_->projectname_combox->setEnabled(false);
          } 
        }  
      }
  }
}
   
QString MainWindow::chooseUpdateVehicleView() {
  return dockviewdialog_->vehicleinfo_combox->currentText();
  // currentIndex() % (robots_info_.size() + 1);
}
QString MainWindow::chooseProjectName() {
  return dockviewdialog_->projectname_combox->currentText();
}
QString MainWindow::chooseProjectNameIndex(int index) {
  return dockviewdialog_->projectname_combox->itemText(index);
}
void MainWindow::getVehicleViewInfo() {
  if (dockviewdialog_->projectname_combox->isEnabled()) {
     map_name = chooseProjectName();
     updateVehicleData(map_name);
  } else if (ProjectName_ == "" && map_name == "") {
    return;
  }
  
  QString current_robot_id;
  bool isPull_ = false;
  std::set<std::string> currentrobot;
  // std::vector<std::string> currentrobotID;
  std::set<std::string>::iterator iter;

  if (robots_info_.size() != 0) {
    current_robot_id = chooseUpdateVehicleView();
  }
  //获取车辆信息
  // QDateTime current_date_time = QDateTime::currentDateTime();
  // QString current_date =current_date_time.toString("yyyy-MM-dd hh:mm:ss");
  for (auto robot_info : robots_info_) {
    if (!current_robot_id.isEmpty() && QString::fromStdString(robot_info.first) == map_name) {
      if (current_robot_id == "全部") {
        for (auto robot_i : robot_info.second) {
          currentrobot.insert(robot_i);
        }
      } else {
        currentrobot.insert(current_robot_id.toStdString());
      }
      break;
    }
  }
  isPull_ = mapwidget_->getVehicleViewInfo(map_name,currentrobot);
  if (isPull_) {
    mapwidget_->updateShowVehicleData(currentrobot);
    isPull_ = false;
  }
}
 
//-----clicked-----
void MainWindow::onMouseEventClicked(float x,float y,float w)
{
    int type_id = 0;
    // int type_id = getCurrentVmapTypeID();
    int type_num_id = 0;
    printf("x=%f,y=%f,w=%f\n",x,y,w);
    Way_Point_ pose;
    pose.point.x = x;
    pose.point.y = y;
    pose.point.quate_w = w;
    //--是否自动判断高度
    if (key_frames_.size() != 0 && keyframes_poses_.size() != 0)
    {
      double height = mapwidget_->findCurrentNearPointHeight(pose,100,key_frames_) - lidar_height_;
      if (height == std::numeric_limits<double>::max() && rBtn_addNewmap->isChecked())
      {
        QMessageBox::question(this, tr("提示"), tr("该点附近无法自动识别高度，已调整为点云高度."),
                        QMessageBox::Yes);
        pose.point.z = local_points_zmin_;
      } else {
        pose.point.z = height;
      }
    } else if (keyframes_poses_.size() != 0)
    {
      double height = mapwidget_->findCurrentNearPointHeight(pose,100,keyframes_poses_) - lidar_height_;
      if (height == std::numeric_limits<double>::max() && rBtn_addNewmap->isChecked())
      {
        QMessageBox::question(this, tr("提示"), tr("该点附近无法自动识别高度，已调整为点云高度."),
                        QMessageBox::Yes);
        pose.point.z = local_points_zmin_;
      } else {
        pose.point.z = height;
      }
    } else {
      pose.point.z = local_points_zmin_;
    }
    //--
    wp_point_ = pose;
    updateFromClickPoint(wp_point_);
    updateDisplayMapElement();
    updateStatusBar();
    dockInfoWidget->setVisible(false);
    if (rBtn_addNewmap->isChecked() && viewEnable_ == false)
    {
      int reply = QMessageBox::question(this, tr("切换到“2D模式“?"),
                            tr("No：保持当前模式,Yes：切换到2D模式。"),
                            QMessageBox::Yes | QMessageBox::No);
      if (reply = QMessageBox::Yes) {
        twoDimenact->setChecked(true);
        twoDimenact_Action();
      } else {
        return;
      } 
    } 
}
void MainWindow::onrightMouseEventClicked(float x,float y,float w,int win_x,int win_y) {
  std::cout << "右键" << std::endl;
  std::vector<MapDockInfo> current_docks_data = mapwidget_->docksViewData();
  dockInfoWidget->setVisible(false);
  if (current_docks_data.size() == 0) {
    return;
  }
  //获取鼠标位置
  point_ point;
  point.x = x;
  point.y = y;
  point.z = local_points_zmin_;
  printf("x=%f,y=%f,w=%f\n",x,y,w);
  int index = mapwidget_->findCurrentNearDocksNumber(point,20);
  if (index >= 0 ) {
    QString currentlanlon = QString::number(current_docks_data[index].longitude, 'f', 9) + ", " +
                            QString::number(current_docks_data[index].altitude, 'f', 9);
    QString currentpose = QString::number(current_docks_data[index].post[0], 'f', 4) + ", " +
                          QString::number(current_docks_data[index].post[1], 'f', 4) + ", " +
                          QString::number(current_docks_data[index].post[2], 'f', 4) + ".";
    dockID_label->setText(tr(current_docks_data[index].id.c_str()));
    dockname_label->setText(tr(current_docks_data[index].name.c_str()));
    docktype_label->setText(tr(current_docks_data[index].type.c_str()));
    docklanlon_label->setText(currentlanlon);
    dockpose_label->setText(currentpose);
    //设置信息栏位置
    int dockInfoWidth = (dockInfoWidget->geometry().width())/2;
    int dockInfoHeight = (dockInfoWidget->geometry().height())/2;
    int double_x = win_x + dockInfoWidth - 40;
    int double_y = win_y + dockInfoHeight;
    dockInfoWidget->move(double_x,double_y);
    dockInfoWidget->setVisible(true);
  } else {
    return;
  }
  // printf("x=%f,y=%f,w=%f\n",x,y,w);
}
void MainWindow::changeMainWindowClicked() {
  if (changeMainWindow->isChecked())
  {
    if (rBtn_addNewmap->isChecked() || rBtn_pointCorrection->isChecked())
    {
      mapToolWidget->setVisible(true);
      r_toolButton->setArrowType(Qt::RightArrow);
    }
    setMainWidgetView(true,false);
    udpateMenuActEnabled(true);
    updateQWidgetEnableStatus();
  } else {
    for (auto listActDock : fixDockWidgets)
    {
      QAction *listAction = (QAction *)listActDock.first;
      QDockWidget *listDockWidget = (QDockWidget *)listActDock.second;
      listDockWidget->setVisible(false);
      listAction->setChecked(false);
      r_toolButton->setArrowType(Qt::LeftArrow);
    }
    setMainWidgetView(false,true);
    udpateMenuActEnabled(false);
  }
  updateStatusBar();
}
//刷新pcd高度信息
void MainWindow::refreshPcdClicked() {
  CreatMapHeightDialog dialog(mapwidget_,this);
  dialog.readDialogShowPose();
  if (dialog.exec() == QDialog::Rejected) {
    dialog.setDialogClosePose();
    return;
  }

  dialog.setDialogClosePose();
  if (!dialog.enteredPcdSzie().isEmpty())
  {
    const std::string id = "0";
    float mpc_size = dialog.enteredPcdSzie().toFloat();
    mapwidget_->setPointCloudSize(id, mpc_size);
  }
  if (!dialog.enteredPcdHeightMin().isEmpty() && !dialog.enteredPcdHeightMax().isEmpty())
  {
    double value_min = atof(dialog.enteredPcdHeightMin().toStdString().c_str());
    double value_max = atof(dialog.enteredPcdHeightMax().toStdString().c_str());  
    if (value_min <= global_points_zmin_)
    {
      value_min = global_points_zmin_;
    }
    if (value_max >= global_points_zmax_)
    {
      value_max = global_points_zmax_;
    }
    local_points_zmin_ = value_min;
    local_points_zmax_ = value_max;
    pcd_z_label->setText(QString("*高度限制（%1 ~ %2.）")
                                 .arg(local_points_zmax_)
                                 .arg(local_points_zmin_));
    // pcdzmin_edit->setText(QString::number(local_points_zmin_));
    // pcdzmax_edit->setText(QString::number(local_points_zmax_));
    if (aixcoloract->isChecked() && !intensityact->isChecked()) {
      mapwidget_->setPcIntensityMode(false,local_points_zmin_,local_points_zmax_);
    }
    mapwidget_->reloadPcdMap(local_points_zmin_, local_points_zmax_);
    // mapwidget_->setMapZmin(local_points_zmin_);
  }
  bool ischeck = dialog.isCheckBoxChecked();
  if (ischeck)
  {
    mapwidget_->setCameraZero();
  }
}
// void MainWindow::reloadpcdClicked() {
//   pcd_z_label->setText(QString("*高度限制（%1 ~ %2.）")
//                                  .arg(local_points_zmax_)
//                                  .arg(local_points_zmin_));
//   // pcdzmin_edit->setText(QString::number(local_points_zmin_));
//   if (aixcoloract->isChecked() && !intensityact->isChecked()) {
//     mapwidget_->setPcIntensityMode(false,local_points_zmin_,local_points_zmax_);
//   }
//   mapwidget_->reloadPcdMap(local_points_zmin_, local_points_zmax_);
// }
void MainWindow::right_on_btn_hide_clicked() {
  Qt::ArrowType arrowtype = r_toolButton->arrowType();
  for (auto listActDock : fixDockWidgets)
  {
    QAction *listAction = (QAction *)listActDock.first;
    QDockWidget *listDockWidget = (QDockWidget *)listActDock.second;
    Qt::DockWidgetArea area = this->dockWidgetArea(listDockWidget);
    if (arrowtype == Qt::LeftArrow && area == Qt::RightDockWidgetArea)
    {
      listDockWidget->show();
      r_toolButton->setArrowType(Qt::RightArrow);
      listAction->setChecked(true);
    }
    else if (arrowtype == Qt::RightArrow && area == Qt::RightDockWidgetArea)
    {
      listDockWidget->hide();
      r_toolButton->setArrowType(Qt::LeftArrow);
      listAction->setChecked(false);
    } 
  }
  updateDisplayMapElement();
}
void MainWindow::left_on_btn_hide_clicked() {
  Qt::ArrowType arrowtype = l_toolButton->arrowType();
  for (auto listActDock : fixDockWidgets)
  {
    QAction *listAction = (QAction *)listActDock.first;
    QDockWidget *listDockWidget = (QDockWidget *)listActDock.second;
    Qt::DockWidgetArea area = this->dockWidgetArea(listDockWidget);
    if (arrowtype == Qt::RightArrow && area == Qt::LeftDockWidgetArea)
    {
      listDockWidget->show();
      l_toolButton->setArrowType(Qt::LeftArrow);
      listAction->setChecked(true);
    }
    else if (arrowtype == Qt::LeftArrow && area == Qt::LeftDockWidgetArea)
    {
      listDockWidget->hide();
      l_toolButton->setArrowType(Qt::RightArrow);
      listAction->setChecked(false);
    } 
  }
  updateDisplayMapElement();
}
void MainWindow::objectvisible_checkbox_clicked(bool state) {
  mapwidget_->seceneObjectVisible(state);
}
void MainWindow::mapvisible_checkbox_clicked(bool state) { 
  mapwidget_->seceneMapVisible(state);
}
void MainWindow::docksvisible_checkbox_clicked(bool state) {
  //选中情况下下拉框选择对应环境桩点信息
  QStringList items; //ComboBox 列表的内容
  items <<"生产环境"<<"预发布环境";
  QString dlgTitle="环境选择框";
  QString txtLabel="请选择桩点所在环境";
  int curIndex=0; //初始选择项
  bool editable=true; //ComboBox是否可编辑
  bool ok=false;
  if (state)
  {
    QString text = QInputDialog::getItem(this, dlgTitle,txtLabel,items,curIndex,editable,&ok);
    if (ok && !text.isEmpty()) {
      if (text == "生产环境")
      {
        std::string domain = "api.ctirobot.com";
        mapwidget_->setDockRequestDomain(domain);
      }
      else if (text == "预发布环境")
      {
        std::string domain = "staging.api.ctirobot.com";
        mapwidget_->setDockRequestDomain(domain);
      }
    }
    choosePullMapDocksView();
    initPoseGet();
  }
  mapwidget_->seceneDocksVisible(state);
}
void MainWindow::initpose_checkbox_clicked(bool state) {
  // mapwidget_->seceneDocksVisible(state);
}
void MainWindow::mapremove_checkbox_clicked() {
  int reply = QMessageBox::question(this, tr("是否移除pcd地图文件?"),
                              tr("No：否,   Yes：是。"),
                              QMessageBox::Yes | QMessageBox::No);
  if (reply = QMessageBox::Yes) {
    mapwidget_->seceneMapRemove();
  } else {
    return;
  }
}
void MainWindow::docksremove_checkbox_clicked() {
  int reply = QMessageBox::question(this, tr("是否移除地图桩点信息?"),
                              tr("No：否,   Yes：是。"),
                              QMessageBox::Yes | QMessageBox::No);
  if (reply = QMessageBox::Yes) {
    mapwidget_->seceneDocksRemove();
  } else {
    return;
  }
}
void MainWindow::bindClicked() {
  if (lanelet2map_mode_checkbox->isChecked()) {
    if (mapwidget_->getLaneletCurrentTypeBind()) {
      bind_pBtn->setEnabled(false);
      chooselanelettypes_pBtn->setEnabled(true);
    }
    updateDisplayMapElement();
    return;
  }
  bind_pBtn->setEnabled(false);
}
void MainWindow::choosPointClicked() {
  if (rBtn_pointCorrection->isChecked()) {
    if ((lanelet2map_mode_checkbox->isChecked() &&
         vmap_mode_checkbox->isChecked()) ||
        (!lanelet2map_mode_checkbox->isChecked() &&
         !vmap_mode_checkbox->isChecked())) {
      QMessageBox::question(this, tr("提示"),
                            tr("修正模式,必须并且只能对一种地图进行修改！"),
                            QMessageBox::Yes);
      return;
    }
  }
  chooselanelettypes_pBtn->setEnabled(true);
  choosepoint_pBtn->setEnabled(false);
  if (lanelet2_ids_.size() == 2) {
    lanelet2_ids_.clear();
    updateDisplayMapElement();
  }
  choosepoint_pBtn->setEnabled(false);
}
void MainWindow::creatnewlaneClicked() {
    updateVmapcomboBoxInfo();
    if (viewEnable_ == false)
    {
      int reply = QMessageBox::question(this, tr("切换到“2D模式“?"),
                              tr("No：保持当前模式,Yes：切换到2D模式。"),
                              QMessageBox::Yes | QMessageBox::No);
      if (reply = QMessageBox::Yes) {
        twoDimenact->setChecked(true);
        twoDimenact_Action();
      } else {
        return;
      }
    }  
    if (!rBtn_addNewmap->isChecked()) {
      int reply =
          QMessageBox::question(this, tr("切换到“画图模式“?"),
                              tr("No：保持当前模式,Yes：切到画图模式。"),
                              QMessageBox::Yes | QMessageBox::No);
      if (reply = QMessageBox::Yes) {
          rBtn_addNewmap_Toggled(true);
      } else {
          return;
      }
    }
    if (rBtn_addNewmap->isChecked()) {  // TO DO this "if" is necessary?
      if ((!vmap_mode_checkbox->isChecked()) &&
          (!lanelet2map_mode_checkbox->isChecked())) {
        QMessageBox::question(
            this, tr("提示"),
            tr("请至少选择一种待新建的地图, Lanelet2map? Vmap?"),
            QMessageBox::Yes);
        return;
      }
      if (vmap_mode_checkbox->isChecked()) {
        int type_id = getCurrentVmapTypeID();
        //若绘制类型为门 信号灯 电梯，弹出gid设置框
        int type_num_id = mapwidget_->getRandEmptyTypeNumId(type_id);
        if (type_id == Vm_T::Signal || type_id == Vm_T::Gate || type_id == Vm_T::Elevator)
        {
          showGidDIalog(type_id);
        }
        std::cout << "线的创建： " << type_id << " " << type_num_id << std::endl;
        maptype_spinBox->setValue(type_num_id);
      }
      if (lanelet2map_mode_checkbox->isChecked()) {
        std::vector<lanelet::Point3d>().swap(point3d_vec_);
        std::vector<int>().swap(lanelet2_ids_);
        // point3d_vec_.clear();
        // lanelet2_ids_.clear();
        current_point3d_id_ = 0;
      }
      updateDisplayMapElement();
    }
}
void MainWindow::clearAllMap_checkbox_clicked() {
  if ((lanelet2map_mode_checkbox->isChecked() &&
       vmap_mode_checkbox->isChecked()) ||
      (!lanelet2map_mode_checkbox->isChecked() &&
       !vmap_mode_checkbox->isChecked())) {
    QMessageBox::question(this, tr("提示"),
                          tr("一次只能删除一种类型地图，请确定您的删除类型！"),
                          QMessageBox::Yes);
    return;
  }
  if (lanelet2map_mode_checkbox->isChecked()) {
    int reply = QMessageBox::question(this, tr("确定要清空所有的矢量地图吗？"),
                                      tr("No：返回,Yes：删除所有数据。"),
                                      QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::Yes) {
      mapwidget_->clearAllLanelet2map();
    }
  } else if (vmap_mode_checkbox->isChecked()) {
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();
    int reply = QMessageBox::question(
        this, tr("注意选择删除方式："),
        tr("No：删除当前类型编号数据,Yes：删除所有类型数据，Cancel:取消。"),
        QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
    if (reply == QMessageBox::Yes) {
      mapwidget_->clearAllVmap();
      //清空gid参数
      purge_arrows_.clear();
      mapwidget_->seceneDocksRemove();
      listGidInfos_.clear();
    } else if (reply == QMessageBox::No) {
      mapwidget_->setTypeNumIdDelVmapPoints(type_id, type_num_id);
    } else {
      return;
    }
  }
  updateDisplayMapElement();
}
void MainWindow::deletemapClicked() {
  if (vmap_mode_checkbox->isChecked()) {
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();
    bool ok = mapwidget_->delCurrentTypeNumIdPoint(type_id, type_num_id);
    if (ok) {
      current_point_index_ =
          current_point_index_ <= 0 ? 0 : current_point_index_ - 1;
      updateDisplayMapElement();
    }
  } else if (lanelet2map_mode_checkbox->isChecked()) {
    if (!chooselanelettypes_pBtn->isEnabled()) {
      if (!lanelet2_ids_.empty()) {
        mapwidget_->rmChooseLanelet2Ids();
        lanelet2_ids_.clear();
      }
      chooselanelettypes_pBtn->setEnabled(true);
      updateDisplayMapElement();
    } else {
      lanelet::Point3d point3d;
      if (mapwidget_->findPoint3dWithId(current_point3d_id_, point3d)) {
        mapwidget_->rmPoint3d(point3d);
        updateDisplayMapElement();
      }
    }
  }
}
void MainWindow::updatevmap_tBtn_Clicked() {
  if (CurQString_ == ""  && ProjectName_ == "") {
    QMessageBox::question(this, tr("注意!"), tr("ERROR：工程名为空！！"),
                          QMessageBox::Yes);
    return;
  }
  bool exitenceflag = false;
  for (size_t i=0; i < allProjectNames.size(); i++) {
    if (ProjectName_ == allProjectNames.at(i)) {
      exitenceflag = true;
    }
  }
  QDir dir(project_vmap_);
  if (!dir.exists()) {
    QString errormessage = project_vmap_ + "不存在，请检查工程名！";
    QMessageBox::question(this, tr("提示"), errormessage,
                          QMessageBox::Yes);
    return;
  }
}
void MainWindow::postureEcho_tBtn_clicked() {

}
void MainWindow::rulerDis_tBtn_clicked() {

}
void MainWindow::userLogin_tBtn_clicked() {
  logindialog_->show();
}
void MainWindow::breaklaneClicked() {
  if (vmap_mode_checkbox->isChecked()) {
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();
    mapwidget_->getNewlineFromCurrentPoint(type_id, type_num_id);
  } else if (lanelet2map_mode_checkbox->isChecked()) {
    if (current_point3d_id_ >= 0) {
      mapwidget_->getNewlineFromCurrentPoint();
    }
  }
  updateDisplayMapElement();
}
void MainWindow::forwardClicked() {
  if (vmap_mode_checkbox->isChecked())
  {
    Way_Point_ wp;
    if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
      int type_id = getCurrentVmapTypeID();
      int type_num_id = getCurrentVmapTypeNumID();
      int num = mapwidget_->getCurrentTypeNumIdNumber(type_id, type_num_id);
      if (num > 0) {
        current_point_index_ = (current_point_index_ + 1) % num;
      }
    }
  } else if (lanelet2map_mode_checkbox->isChecked()) {
    lanelet::Point3d pt, new_pt;
    lanelet::LineString3d ll;
    lanelet::Polygon3d pl;
    if (mapwidget_->findPoint3dWithId(current_point3d_id_, pt)) {
      if (mapwidget_->findLinestringWithId(lanelet2_ids_.back(), ll)) {
        if (mapwidget_->isPoint3dinLinestring(current_point3d_id_, ll)) {
          mapwidget_->moveFrontPoint3d(pt, new_pt, ll);
          current_point3d_id_ = new_pt.id();
        }
      } else if (mapwidget_->findPoly3dWithId(lanelet2_ids_.back(), pl)) {
        if (mapwidget_->isPoint3dinPoly(current_point3d_id_, pl)) {
          mapwidget_->moveFrontPoint3d(pt, new_pt, pl);
          current_point3d_id_ = new_pt.id();
        }
      }
    }
  } 
  updateDisplayMapElement();
}
void MainWindow::backwardClicked() {
  if (vmap_mode_checkbox->isChecked()) {
    Way_Point_ wp;
    if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
      int type_id = getCurrentVmapTypeID();
      int type_num_id = getCurrentVmapTypeNumID();
      int num = mapwidget_->getCurrentTypeNumIdNumber(type_id, type_num_id);
      if (num > 0) {
        current_point_index_--;
        current_point_index_ = current_point_index_ < 0
                                   ? num + current_point_index_
                                   : current_point_index_;
      }
    }
  } else if (lanelet2map_mode_checkbox->isChecked()) {
    lanelet::Point3d pt, new_pt;
    lanelet::LineString3d ll;
    lanelet::Polygon3d pl;
    if (mapwidget_->findPoint3dWithId(current_point3d_id_, pt)) {
      if (mapwidget_->findLinestringWithId(lanelet2_ids_.back(), ll)) {
        if (mapwidget_->isPoint3dinLinestring(current_point3d_id_, ll)) {
          mapwidget_->moveBackPoint3d(pt, new_pt, ll);
          current_point3d_id_ = new_pt.id();
        }
      } else if (mapwidget_->findPoly3dWithId(lanelet2_ids_.back(), pl)) {
        if (mapwidget_->isPoint3dinPoly(current_point3d_id_, pl)) {
          mapwidget_->moveBackPoint3d(pt, new_pt, pl);
          current_point3d_id_ = new_pt.id();
        }
      }
    }
  }
  updateDisplayMapElement();
}
void MainWindow::forwardInsertClicked() {
  if (vmap_mode_checkbox->isChecked()) {
    Way_Point_ wp;
    if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
      int type_id = getCurrentVmapTypeID();
      int type_num_id = getCurrentVmapTypeNumID();
      wp.point.x += 0.5;
      wp.point.y += 0.5;
      mapwidget_->localCoordinatesToGps(wp);
      mapwidget_->insertFrontTypeNumIdPoint(type_id, type_num_id, wp);
      current_point_index_++;
    }
  } else if (lanelet2map_mode_checkbox->isChecked()) {
    lanelet::Point3d pt, new_pt;
    lanelet::LineString3d ll;
    lanelet::Polygon3d pl;
    if (mapwidget_->findPoint3dWithId(current_point3d_id_, pt)) {
      new_pt.x() = pt.x() + 0.5;
      new_pt.y() = pt.y() + 0.5;
      new_pt.z() = pt.z();
      if (mapwidget_->findLinestringWithId(lanelet2_ids_.back(), ll)) {
        if (mapwidget_->isPoint3dinLinestring(current_point3d_id_, ll)) {
          mapwidget_->insertFrontPoint3d(pt, ll, new_pt);
          current_point3d_id_ = new_pt.id();
        }
      } else if (mapwidget_->findPoly3dWithId(lanelet2_ids_.back(), pl)) {
        if (mapwidget_->isPoint3dinPoly(current_point3d_id_, pl)) {
          mapwidget_->insertFrontPoint3d(pt, pl, new_pt);
          current_point3d_id_ = new_pt.id();
        }
      }
    }
  }
  updateDisplayMapElement();
}
void MainWindow::backwarInsertdClicked() {
  if (vmap_mode_checkbox->isChecked()) {
    Way_Point_ wp;
    if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
      int type_id = getCurrentVmapTypeID();
      int type_num_id = getCurrentVmapTypeNumID();
      wp.point.x += 0.5;
      wp.point.y += 0.5;
      mapwidget_->localCoordinatesToGps(wp);
      mapwidget_->insertBackTypeNumIdPoint(type_id, type_num_id, wp);
      current_point_index_;
    }
  } else if (lanelet2map_mode_checkbox->isChecked()) {
    lanelet::Point3d pt, new_pt;
    lanelet::LineString3d ll;
    lanelet::Polygon3d pl;
    if (mapwidget_->findPoint3dWithId(current_point3d_id_, pt)) {
      new_pt.x() = pt.x() + 0.5;
      new_pt.y() = pt.y() + 0.5;
      new_pt.z() = pt.z();
      if (mapwidget_->findLinestringWithId(lanelet2_ids_.back(), ll)) {
        if (mapwidget_->isPoint3dinLinestring(current_point3d_id_, ll)) {
          mapwidget_->insertBackPoint3d(pt, ll, new_pt);
          current_point3d_id_ = new_pt.id();
        }
      } else if (mapwidget_->findPoly3dWithId(lanelet2_ids_.back(), pl)) {
        if (mapwidget_->isPoint3dinPoly(current_point3d_id_, pl)) {
          mapwidget_->insertBackPoint3d(pt, pl, new_pt);
          current_point3d_id_ = new_pt.id();
        }
      }
    }
  }
  updateDisplayMapElement();
}
void MainWindow::gidSet_pBtn_Clicked() {

}
void MainWindow::chooseVmaps_pBtn_Clicked() {
    choosevmaps_pBtn->setEnabled(false);
    maptypes_comboBox->setEnabled(false);
    maptype_spinBox->setEnabled(false);
    updateDisplayMapElement();
}
//-----lanelet clicked-----
void MainWindow::chooselanelettypes_pBtn_clicked() {
  lanelet2_ids_.clear();
  updateDisplayMapElement();
  chooselanelettypes_pBtn->setEnabled(false);
}
void MainWindow::addlanelet_pBtn_clicked() {
  if (!chooselanelettypes_pBtn->isEnabled()) {
    if (!lanelet2_ids_.empty()) {
      int lanelet_id = mapwidget_->addLanelet();
      lanelet2_ids_.clear();
      lanelet2_ids_.push_back(lanelet_id);
    }
    chooselanelettypes_pBtn->setEnabled(true);
    updateDisplayMapElement();
  }
}
void MainWindow::jointlanelet_pBtn_clicked() {
  mapwidget_->jointLanelet();
  lanelet2_ids_.erase(lanelet2_ids_.begin());
  chooselanelettypes_pBtn->setEnabled(true);
  updateDisplayMapElement();
}
void MainWindow::inverlanelet_pBtn_clicked() {
  mapwidget_->invertLanelet();
  lanelet2_ids_.clear();
  updateDisplayMapElement();
}
void MainWindow::extendLinestring_pBtn_clicked() {
  lanelet::LineString3d ls;
  int linestring_id = lanelet2_ids_.back(); 
  lanelet::Point3d pt;
  if ((mapwidget_->findPoint3dWithId(current_point3d_id_, pt) && 
       mapwidget_->findLinestringWithId(linestring_id, ls)))
  {
    mapwidget_->extendLinestring(linestring_id, pt);
  }
  chooselanelettypes_pBtn->setEnabled(true);
  updateDisplayMapElement();
}
void MainWindow::addregular_pBtn_clicked() {
  int regular_id = 0;
  reguinfo_list reg_list;
  if (mapwidget_->addregular(regular_id, reg_list)) {
    mapwidget_->addRegularInfo(reg_list);
    lanelet2_ids_.clear();
  } else {
    lanelet2_ids_.clear();
    QMessageBox::question(
        this, tr("提示"),
        tr("添加交规失败，请先确认所选类型无误，再重新添加！"),
        QMessageBox::Yes);
  }
  regularid_edit->setText(QString::number(regular_id));
  chooselanelettypes_pBtn->setEnabled(true);
  updateDisplayMapElement();
}
void MainWindow::editregular_pBtn_clicked() {
  regulardialog_->readDialogShowPose();
  regulardialog_->show();
}
void MainWindow::deletregular_pBtn_clicked() {
  int reply = QMessageBox::question(this, tr("提示"), tr("确定删除当前交规?"),
                                    QMessageBox::Yes | QMessageBox::No);
  if (reply == QMessageBox::Yes) {
    int regul_id = atoi(regularid_edit->text().toStdString().c_str());
    // //#pragma omp parallel for
    mapwidget_->deletRegularInfo(regul_id);
    regularid_edit->setText(QString("--"));
    regularid_edit->setEnabled(false);
  }
  updateDisplayMapElement();
}
void MainWindow::addtags_pBtn_clicked() {
  taginfodialog_->readDialogShowPose();
  taginfodialog_->show();
  updateCurrentTypeTags();
}
//导出rpcd文件
void MainWindow::originalpcdsave_pbtn_clicked() {
  // 导出原始点云rpcd文件
  bool rmapExportStatu;
  QString filepath_r = QString::fromStdString(HOME_STR) + "/maptool-candela/CTI_MAP_PROJECT/" + ProjectName_ + "/RMAP/" + ProjectName_ + "/";
  QString suffixName_r = ProjectName_ + "_r.pcd";
  if (map_zero_gnss_)
  {
    {
     //线程使用的每个引用都将引用同一个对象,通过使用带有线程的引用包装器来摆脱它,传递真正的引用需要使用std::ref,传递指针使用std::move
      std::promise<bool> promiseObj;
      std::future<bool> futureObj = promiseObj.get_future();
      std::thread thExportRCloudMap(&MainWindow::savePointCloudPcdFile, this, std::ref(ProjectName_),std::ref(key_frames_),std::ref(map_zero_gnss_),false,0.2,true,std::ref(promiseObj));
      rmapExportStatu = futureObj.get();
      thExportRCloudMap.detach();
    }
    if (rmapExportStatu)
    {
      QString path_str = filepath_r + suffixName_r;
      QString info = "文件已成功存入：\n" + path_str;
      QMessageBox::information(this, tr("成功！"), info, QMessageBox::Yes);
    } else {
      QString path_str = filepath_r + suffixName_r;
      QString info = "文件存入失败！！！请检查路径信息：\n" + path_str;
      QMessageBox::information(this, tr("失败！"), info, QMessageBox::Yes);
    }
    // savePointCloudPcdFile(ProjectName_,key_frames_,map_zero_gnss_,false,0.2,true);
  }
  
}
//导出pcd文件
void MainWindow::filterpcdsave_pbtn_clicked() {
  // 导出滤波后的车端点云pcd文件
  double downsample_filter_num = exportdialog_->getCurrentFilterNum();
  bool mapExportStatu;
  QString filepath = QString::fromStdString(HOME_STR) + "/maptool-candela/CTI_MAP_PROJECT/" + ProjectName_ + "/MAP/" + ProjectName_ + "/";
  QString suffixName = ProjectName_ + ".pcd";
  if (map_zero_gnss_)
  {
    {
      std::cout << "-------------value: " << downsample_filter_num << std::endl;
      std::promise<bool> promiseObj;
      std::future<bool> futureObj = promiseObj.get_future();
      std::thread thExportCloudMap(&MainWindow::savePointCloudPcdFile, this, std::ref(ProjectName_),std::ref(key_frames_),std::ref(map_zero_gnss_),true,downsample_filter_num,true,std::ref(promiseObj));
      mapExportStatu = futureObj.get();
      thExportCloudMap.detach();
    }
    if (mapExportStatu)
    {
      QString path_str = filepath + suffixName;
      QString info = "文件已成功存入：\n" + path_str;
      QMessageBox::information(this, tr("成功！"), info, QMessageBox::Yes);
    } else {
      QString path_str = filepath + suffixName;
      QString info = "文件存入失败！！！请检查路径信息：\n" + path_str;
      QMessageBox::information(this, tr("失败！"), info, QMessageBox::Yes);
    }
    // savePointCloudPcdFile(ProjectName_,key_frames_,map_zero_gnss_,true,downsample_filter_num,true);
  }
}
void MainWindow::datafilesave_pbtn_clicked() {
  //导出原始数据内帧数据
  if (keyframes_poses_.size() != 0)
  {
    QString frames_path = QString::fromStdString(home_str) +"/maptool-candela/CTI_MAP_PROJECT/" + ProjectName_ + "/DATA/";
    qDebug() << "导出原始数据帧";
    bool result = mapwidget_->writeKeyFramesFiles(frames_path,keyframes_poses_);
    
    if (result) {
      QString message = "文件已成功存入： \n" + frames_path + "keyframes.csv";
      QMessageBox::information(this, tr("成功！"), message,
                            QMessageBox::Yes);
    } else {
      QString message = "文件存入失败！！！请检查存入路径：\n" + frames_path;
      QMessageBox::information(this, tr("失败！"), message,
                            QMessageBox::Yes);
    }
  }
}
//版本更新
void MainWindow::update_Act_checkbox_clicked() {
  Q_EMIT updateMapTool();
}
//检查版本
void MainWindow::inspect_Act_checkbox_clicked() {
  QString localVer = choose_map_->getMapToolLocalVer();
  QString remoteVer = choose_map_->getMapToolRemoteVer();

  QString message = "当前软件版本为：\n" + localVer + "\n" +
                    "软件最新版本为：\n" + remoteVer;
  QMessageBox::information(this, tr("软件版本信息"), message,
                            QMessageBox::Yes);
}
//--dialog--
void MainWindow::regulinfo_dialog_okpBtn_clicked() {
  addregularinfo();
  regulardialog_->accept();
  regulardialog_->setDialogClosePose();
  deletregular_pBtn->setEnabled(false);
  editregular_pBtn->setEnabled(false);
}
void MainWindow::regulinfo_dialog_cancelpBtn_clicked() {
  regulardialog_->reject();
  regulardialog_->setDialogClosePose();
  deletregular_pBtn->setEnabled(false);
  editregular_pBtn->setEnabled(false);
}
void MainWindow::taginfo_dialog_okpBtn_clicked() {
  tags_edit_editingFinished();
  taginfodialog_->accept();
  taginfodialog_->setDialogClosePose();
}
void MainWindow::taginfo_dialog_cancelpBtn_clicked() {
  taginfodialog_->reject();
  taginfodialog_->setDialogClosePose();
}
void MainWindow::docksview_dialog_okpBtn_clicked() {
  dockviewdialog_->accept();
  getVehicleViewInfo();
  dockviewdialog_->setDialogClosePose();
}
void MainWindow::docksview_dialog_cancelpBtn_clicked() {
  dockviewdialog_->reject();
  dockviewdialog_->setDialogClosePose();
}
void MainWindow::gridsize_dialog_okpBtn_clicked() {
  // gridParamSet();
  griddialog_->accept();
  gridParamSet();
  griddialog_->setDialogClosePose();
}
void MainWindow::gridsize_dialog_cancelpBtn_clicked() {
  griddialog_->reject();
  griddialog_->setDialogClosePose();
}
void MainWindow::exportclod_dialog_okpBtn_clicked() {
  exportdialog_->accept();
  exportdialog_->setDialogClosePose();
}
void MainWindow::exportclod_dialog_cancelpBtn_clicked() {
  exportdialog_->reject();
  exportdialog_->setDialogClosePose();
}
void MainWindow::loginui_dialog_okpBtn_clicked() {
  logindialog_->accept();
  logindialog_->setRemberedPassword();
  getAccessToken();
}
void MainWindow::loginui_dialog_cancelpBtn_clicked() {
  logindialog_->reject();
}
void MainWindow::cloudpub_dialog_cancelpBtn_clicked() {
  pubdialog_->reject();
}
void MainWindow::cloudpub_dialog_okpBtn_clicked() {
  pubdialog_->accept();
  std::string map_id = pubdialog_->getParkIdText().toStdString();
  std::string map_name = getCurrentParkText().toStdString();
  int typeIndex = pubdialog_->getDataTypeIndex();
  if (pubdialog_->create_checkbox->isChecked()) {
    std::cout << "创建" << std::endl;
    pubInitPoseAndAreaInfo(map_name,map_id,typeIndex);
  } else if (pubdialog_->modify_checkbox->isChecked()) {
    //修改区域数据
    std::cout << "修改" << std::endl;
    modifyOrPubFullData(false,map_name,map_id,typeIndex);
  } else if (pubdialog_->delete_checkbox->isChecked()) {
    std::cout << "删除" << std::endl;
    deleteAllData(map_id,typeIndex);
  } else if (signaldelete_batch_act->isChecked()) {

  } 
}
void MainWindow::setgid_dialog_cancelpBtn_clicked() {
  setgiddialog_->reject();
}
void MainWindow::setgid_dialog_okpBtn_clicked() {
  setgiddialog_->accept();
  getGidInfo();
}


/*不同编辑状态（分别新建，查看，修正,首页模式）
主界面小插件的状态
*/
void MainWindow::updateQWidgetEnableStatus() {
  if (rBtn_addNewmap->isChecked()) {
    setQWidgetStatus(true, false);
    regularid_edit->setEnabled(false);
    newtype_pBtn->setEnabled(true);
    breakLine_pBtn->setEnabled(false);
    choosepoint_pBtn->setEnabled(false);
    clear_pBtn->setEnabled(false);
    chooselanelettypes_pBtn->setEnabled(false);
    gidSet_pBtn->setEnabled(false);
    choosevmaps_pBtn->setEnabled(false);
    if (lanelet2map_mode_checkbox->isChecked() &&
        !vmap_mode_checkbox->isChecked()) {
      undoact->setEnabled(false);
      delete_pBtn->setEnabled(false);
      setVmapProQWidgetStatus(false);
    } else if (vmap_mode_checkbox->isChecked()) {
      addtags_pBtn->setEnabled(false);
    }
  } 
  else if (rBtn_addNone->isChecked()) {
    choosevmaps_pBtn->setEnabled(false);
    regularid_edit->setEnabled(false);
    chooselanelettypes_pBtn->setEnabled(false);
    setQWidgetStatus(false, false);
  } 
  else if (rBtn_pointCorrection->isChecked()) {
    setQWidgetStatus(true, false);
    if (vmap_mode_checkbox->isChecked()) {
      regularid_edit->setEnabled(false);
      forward_pBtn->setEnabled(true);
      backward_pBtn->setEnabled(true);
      forwardInsert_pBtn->setEnabled(true);
      backwardInsert_pBtn->setEnabled(true);
      gidSet_pBtn->setEnabled(true);
      chooselanelettypes_pBtn->setEnabled(false);
      addtags_pBtn->setEnabled(false);
    } 
    else if (lanelet2map_mode_checkbox->isChecked()) {
      // laneletid_search_edit->setEnabled(true);
      // linestringid_search_edit->setEnabled(true);
      // findLinestring_pBtn->setEnabled(true);
      // findLanelet_pBtn->setEnabled(true);
      bind_pBtn->setEnabled(false);
      setVmapProQWidgetStatus(false);
      getLanelet2mapFunQWidgetStatus();
    }
  }
}
void MainWindow::udpateMenuActEnabled(bool status) {
  static bool enable = false;
  if (enable == false || !changeMainWindow->isChecked()) {
    undoact->setEnabled(status);
    // clear_pBtn->setEnabled(status);
    // delete_pBtn->setEnabled(status);
    enable = true;
  }
  modemenu->setEnabled(status);
  colorViewmenu->setEnabled(status);
  sizesetmenu->setEnabled(status);
  mapchangemenu->setEnabled(status);
  gridparamact->setEnabled(status);
  view2d3dmenu->setEnabled(status);

  toolbar_->savebutton->setEnabled(status);
  toolbar_->modebutton->setEnabled(status);
  toolbar_->slamfilebutton->setEnabled(status);
  toolbar_->databutton->setEnabled(status);
  toolbar_->exportbutton->setEnabled(false);
  toolbar_->isviewbutton->setEnabled(status);
  toolbar_->colorbutton->setEnabled(status);
  toolbar_->sizebutton->setEnabled(status);
  toolbar_->mapchangebutton->setEnabled(status);
  toolbar_->gridbutton->setEnabled(status);
  toolbar_->view2d3dbutton->setEnabled(status);
  toolbar_->imagebutton->setEnabled(status);
  // toolbar_->updatebutton->setEnabled(status);
  toolbar_->uploadbutton->setEnabled(status);
  toolbar_->testmapbutton->setEnabled(status);
  toolbar_->docksviewbutton->setEnabled(status);
  toolbar_->clearbutton->setEnabled(status);
  toolbar_->zerobutton->setEnabled(status);
  toolbar_->versionbutton->setEnabled(status);
  toolbar_->cloudLinkbutton->setEnabled(status);

  allmapSave->setEnabled(status);
  onlymapSave->setEnabled(status);
  view2d3dact->setEnabled(status);
  disWidgetact->setEnabled(status);
  imaWidgetact->setEnabled(status);
  refreshZeroAct->setEnabled(status);
  refreshPcdAct->setEnabled(status);
  imageview->setEnabled(status);
  uploadAct->setEnabled(status);
  testMapAct->setEnabled(status);
  updatevmap_tBtn->setEnabled(status);
  // postureEcho_tBtn->setEnabled(status);
  // rulerDis_tBtn->setEnabled(status);
  userLogin_tBtn->setEnabled(status);
}
void MainWindow::setQWidgetStatus(bool editStatus, bool editStatus1) {
  setVmapProQWidgetStatus(editStatus);
  choosepoint_pBtn->setEnabled(editStatus);
  undoact->setEnabled(editStatus);
  clear_pBtn->setEnabled(editStatus);
  delete_pBtn->setEnabled(editStatus);
  breakLine_pBtn->setEnabled(editStatus);
  bind_pBtn->setEnabled(editStatus);
  localx_edit->setEnabled(editStatus);
  localy_edit->setEnabled(editStatus);
  localz_edit->setEnabled(editStatus);
  latitude_edit->setEnabled(editStatus);
  longitude_edit->setEnabled(editStatus);
  maptypes_comboBox->setEnabled(editStatus);
  addtags_pBtn->setEnabled(editStatus);

  addregular_pBtn->setEnabled(editStatus1);
  jointlanelet_pBtn->setEnabled(editStatus1);
  extendLinestring_pBtn->setEnabled(editStatus1);
  inverlanelet_pBtn->setEnabled(editStatus1);
  deletregular_pBtn->setEnabled(editStatus1);
  editregular_pBtn->setEnabled(editStatus1);
  addlanelet_pBtn->setEnabled(editStatus1);
  forward_pBtn->setEnabled(editStatus1);
  backward_pBtn->setEnabled(editStatus1);
  forwardInsert_pBtn->setEnabled(editStatus1);
  backwardInsert_pBtn->setEnabled(editStatus1);
  newtype_pBtn->setEnabled(editStatus1);
  gidSet_pBtn->setEnabled(editStatus1);
  // laneletid_search_edit->setEnabled(editStatus1);
  // linestringid_search_edit->setEnabled(editStatus1);
  // findLinestring_pBtn->setEnabled(editStatus1);
  // findLanelet_pBtn->setEnabled(editStatus1);
}
void MainWindow::setVmapProQWidgetStatus(bool editStatus) {
  linktypenum_edit->setEnabled(editStatus);
  limitvelocity_edit->setEnabled(editStatus);
  laneletid_edit->setEnabled(editStatus);
  maptypetype_comboBox->setEnabled(editStatus);
  bypasstype_comboBox->setEnabled(editStatus);
  waytypetype_comboBox->setEnabled(editStatus);
  traveldirectiontype_comboBox->setEnabled(editStatus);
  traveltype_comboBox->setEnabled(editStatus);
  travelslopedtype_comboBox->setEnabled(editStatus);
  linktype_comboBox->setEnabled(editStatus);
  roomtype_comboBox->setEnabled(editStatus);
}
void MainWindow::getLanelet2mapFunQWidgetStatus() {
  if (lanelet2_ids_.size() == 1) {
    int id2 = lanelet2_ids_.back();
    lanelet::Lanelet ll;
    if (mapwidget_->findLaneletWithId(id2, ll)) {
      inverlanelet_pBtn->setEnabled(true);
    }
  } else {
    addtags_pBtn->setEnabled(false);
  }
  if (lanelet2_ids_.size() > 0 && current_point3d_id_ > 0) {
    int id2 = lanelet2_ids_.back();
    lanelet::LineString3d ls;
    lanelet::Polygon3d pl;
    lanelet::Point3d pt;
    if (mapwidget_->findLinestringWithId(id2, ls) &&
        mapwidget_->findPoint3dWithId(current_point3d_id_, pt)) {
      // bind_pBtn->setEnabled(true);
      forwardInsert_pBtn->setEnabled(true);
      backwardInsert_pBtn->setEnabled(true);
      forward_pBtn->setEnabled(true);
      backward_pBtn->setEnabled(true);
      extendLinestring_pBtn->setEnabled(true);
    } else if (mapwidget_->findPoly3dWithId(id2, pl) &&
               mapwidget_->findPoint3dWithId(current_point3d_id_, pt)) {
      forwardInsert_pBtn->setEnabled(true);
      backwardInsert_pBtn->setEnabled(true);
      forward_pBtn->setEnabled(true);
      backward_pBtn->setEnabled(true);
    }
  }
  if (lanelet2_ids_.size() == 2) {
    int id1 = lanelet2_ids_.front();
    int id2 = lanelet2_ids_.back();
    lanelet::LineString3d ls;
    lanelet::Lanelet ll;
    lanelet::Point3d pt;
    if (mapwidget_->findLinestringWithId(id1, ls) &&
        mapwidget_->findLinestringWithId(id2, ls)) {
      addlanelet_pBtn->setEnabled(true);
    } else if (mapwidget_->findLaneletWithId(id1, ll) &&
               mapwidget_->findLaneletWithId(id2, ll)) {
      jointlanelet_pBtn->setEnabled(true);
      // join lanelet true:2 lanle
    } else if (mapwidget_->findLaneletWithId(id1, ll) ||
               mapwidget_->findLaneletWithId(id2, ll)) {
      addregular_pBtn->setEnabled(true);
    }  
  }
}

void MainWindow::updateFromClickPoint(Way_Point_ pose) {
    int type_id = getCurrentVmapTypeID();
    int type_num_id = getCurrentVmapTypeNumID();
    if (rBtn_addNone->isChecked()) {  // look modle
      return;
    }
    else if (rBtn_pointCorrection->isChecked()) {
      if (!choosepoint_pBtn->isEnabled())   {
        if (vmap_mode_checkbox->isChecked()) {
          if (choiceVmapNearPoint(type_id, pose)) {  //选择clickpoint最近的点
            choosepoint_pBtn->setEnabled(true);
          }
          return;
        } else if (lanelet2map_mode_checkbox->isChecked()) {
          //选择clickpoint最近的点
          lanelet::Point3d point3d = mapwidget_->searchNearstPoint3d(pose);
          current_point3d_id_ = point3d.id();
          std::cout << "search current point.id()= " << current_point3d_id_
                    << std::endl;
          if (mapwidget_->findPoint3dWithId(current_point3d_id_, point3d)) {
            choosepoint_pBtn->setEnabled(true);
            // lanelet当前点所在线的投影到图像
          }
          return;
        }    
      }
      if (vmap_mode_checkbox->isChecked()) {
        if (!choosevmaps_pBtn->isEnabled()) { //--选择线段偏移
          if (pugre_vmap_ids_.size() == 2) { //--第三个点为方向点
            bool isOK;
            QString text = QInputDialog::getText(NULL, "Input Purge",
                                                "请输入偏移量（m为单位）",
                                                QLineEdit::Normal,
                                                "",
                                                &isOK);
            if (isOK && !text.isEmpty()) {
              //获取角度
              double arrowdire;
              Ogre::Quaternion ogrequater;
              std::tie(ogrequater, arrowdire) = mapwidget_->angleToOuaterGetDire(pose.point.quate_w);
              std::vector<int> allIndexs;
              std::vector<Way_Point_> allPurgePoints;
              int start_index = pugre_vmap_ids_.front() < pugre_vmap_ids_.back() ? pugre_vmap_ids_.front() : pugre_vmap_ids_.back();
              int end_index = pugre_vmap_ids_.front() > pugre_vmap_ids_.back() ? pugre_vmap_ids_.front() : pugre_vmap_ids_.back();
              //获取所有点的索引值
              for (size_t i = start_index; i <= end_index; i++) {
                allIndexs.push_back(i);
              }
              //获取所有点坐标信息
              for (size_t i = 0; i < allIndexs.size(); i++) {
                Way_Point_ pp;
                bool isSet = mapwidget_->getCurrentTypeNumIdPoint(type_id,type_num_id,allIndexs[i],pp);
                if (isSet) allPurgePoints.push_back(pp);
              }
              //获取所有点偏移后的点数据
              std::vector<Way_Point_> purgeTargetPoses = mapwidget_->getDeviationPose(arrowdire,allPurgePoints,text.toDouble());
              //修改
              if (purgeTargetPoses.size() != 0 &&
                  purgeTargetPoses.size() == allPurgePoints.size()) {
                    std::cout << "enter size:" << purgeTargetPoses.size() << std::endl;
                for (size_t i = 0; i < purgeTargetPoses.size(); i++)
                {
                  createCurrentWayPoint(type_id, purgeTargetPoses[i]);
                  mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                                       allIndexs[i], purgeTargetPoses[i]);
                  current_point_index_ = allIndexs[i];
                }
              }
              pugre_vmap_ids_.clear();
              choosevmaps_pBtn->setEnabled(true);
              maptypes_comboBox->setEnabled(true);
              maptype_spinBox->setEnabled(true);
              return;
            } else {
              int reply = QMessageBox::question(this, tr("是否继续设置偏移方向以及偏移量？"), tr("No：放弃,  Yes：继续."),
                                QMessageBox::Yes | QMessageBox::No);
              if (reply == QMessageBox::Yes) {
                return;
              } else {
                pugre_vmap_ids_.clear();
                choosevmaps_pBtn->setEnabled(true);
                maptypes_comboBox->setEnabled(true);
                maptype_spinBox->setEnabled(true);
                return;
              } 
            }
          }
          getChooseVmapPurgeIds(type_id,pose);
          return;
        }
        Way_Point_ wp;
        if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
          //渲染窗口的操作，点的位置修改更新
          wp.point.x = pose.point.x;
          wp.point.y = pose.point.y;
          wp.point.quate_w = pose.point.quate_w;
          // for bind_pBtn 粘合
          point_ point;
          if (!bind_pBtn->isEnabled() &&
              mapwidget_->findNearPoint(type_id, wp.point, point, 2)) {
            if (type_id == Vm_T::ClearArea)
            {
              QMessageBox::question(this, tr("提示"), tr("清扫区不可进行粘合处理！"),
                                  QMessageBox::Yes);
              return;
            } else {
              wp.point.x = point.x;
              wp.point.y = point.y;
              wp.point.z = point.z;         
            }
            bind_pBtn->setEnabled(true);
          }
          //对界面插件的修改，进行更新
          createCurrentWayPoint(type_id, wp);
          // currentTypeMaptoImage(point);
          mapwidget_->setCurrentTypeNumIdPoint(type_id, type_num_id,
                                              current_point_index_, wp);
          if (type_id == Vm_T::ClearArea)
          {
            if (mapwidget_->setClearareaIntersect(type_id, type_num_id, wp))
            {
              QMessageBox::question(this, tr("提示"), tr("修改后的清扫区存在相交！！"),
                                  QMessageBox::Yes);
            }
          }
        }
      }
      else if (lanelet2map_mode_checkbox->isChecked()) {
        //多选模式
        if (!chooselanelettypes_pBtn->isEnabled()) {  //&&
          // bind_pBtn->isEnabled()) {
          std::cout << " do lanelet choose..." << std::endl;
          getChooseLanelet2Ids(type_id, pose);
          return;
        }
        //对选中的点的位置修改
        lanelet::Point3d point3d;
        if (mapwidget_->findPoint3dWithId(current_point3d_id_, point3d)) {
          point3d.x() = pose.point.x;
          point3d.y() = pose.point.y;
          // point3d.z() = point.z;
          mapwidget_->resetPoint3d(point3d);
          // updateCoordinatesTagDisplay();
        }
      }
    }
    else if (rBtn_addNewmap->isChecked()) {
      if (vmap_mode_checkbox->isChecked()) {
        if (isLimitVmapWayPoint((int)Vm_T::StopPoint, 1) ||
            isLimitVmapWayPoint((int)Vm_T::Elevator, 1) ||
            isLimitVmapWayPoint((int)Vm_T::ConvergePoint, 1) ||
            isLimitVmapWayPoint((int)Vm_T::NodePoint, 1)) {
          QMessageBox::question(
              this, tr("提示"),
              tr("当前类型编号已经被限制数量，请增加新编号操作！"),
              QMessageBox::Yes);
          return;
        }
        //vmap点的创建
        Way_Point_ wp;
        VectorMap_ vectormap;
        wp.point.x = pose.point.x;
        wp.point.y = pose.point.y;
        wp.point.z = pose.point.z;
        wp.point.quate_w = pose.point.quate_w;
        point_ point;
        if (!bind_pBtn->isEnabled() &&
            mapwidget_->findNearPoint(type_id, wp.point, point, 2)) {
          wp.point.x = point.x;
          wp.point.y = point.y;
          wp.point.z = point.z;
          bind_pBtn->setEnabled(true);
        }
        createCurrentWayPoint(type_id, wp);
        vectormap.wp.push_back(wp);  // push_back对vector_map填数据
        vectormap.type_id = type_id;  //矢量地图对应的类型，如lane,roadedge,sigald等
        vectormap.type_num_id = type_num_id;  //编号
        std::cout << "点的创建： " << type_id << " " << type_num_id << std::endl;
        mapwidget_->setCurrentVectorMap(type_id, type_num_id, vectormap);
        int index = mapwidget_->getCurrentTypeNumIdNumber(type_id, type_num_id);
        std::cout << "参数： " << type_id << " " << type_num_id << std::endl;
        if ((type_id == Vm_T::ClearArea) && 
            (maptypetype_comboBox->currentIndex() == TYPE_P1_TYPE_CLEARAREA::PURGEBOUNDARY_TYPE))
        {
          std::cout << "获取边界线方向" << std::endl;
          mapwidget_->setCurrentPurgeArrow(type_id, type_num_id, purge_arrows_);
          showPurgeArrow();
        }
        if (type_id == Vm_T::ClearArea)
        {
          if (mapwidget_->setClearareaIntersect(type_id, type_num_id))
          {
            int reply = QMessageBox::question(this, tr("清扫区存在相交点，是否继续？"),
                                      tr("No：取消,Yes：继续"),
                                      QMessageBox::Yes | QMessageBox::No);
            if (reply == QMessageBox::No) {
              mapwidget_->setTypeNumIdDelBackVmapPoint(type_id, type_num_id);
            }
          }  
        }
        current_point_index_ = index <= 0 ? 0 : index - 1;
      }
      if (lanelet2map_mode_checkbox->isChecked()) {
        if (type_id == (int)Vm_T::Signal || type_id == (int)Vm_T::StopPoint ||
            type_id == (int)Vm_T::SprayArea || type_id == (int)Vm_T::ClearArea ||
            type_id == (int)Vm_T::Gate || type_id == (int)Vm_T::Elevator ||
            type_id == (int)Vm_T::ConvergePoint ||
            type_id == (int)Vm_T::NodePoint || 
            type_id == (int)Vm_T::AttributeArea){
          QMessageBox::question(this, tr("提示"),
                                tr("lanelet地图暂时不支持当前类型！"),
                                QMessageBox::Yes);
          return;
        }
        std::cout << "in new laneletmap " << std::endl;
        lanelet::Point3d point3d;
        point3d.x() = pose.point.x;
        point3d.y() = pose.point.y;
        point3d.z() = pose.point.z;
        // llet2data_.createPoint(point3d,init_origin_pos);
        point3d_vec_.push_back(point3d);
        std::cout << "in new lanelet point3d_vec_.size()="
                  << point3d_vec_.size() << std::endl;
        createCurrentLanelet2Type(type_id, point3d);
      }
    }      
}

/*对vmap点Way_Point_进行更新
更新内容来源与界面插件，属性信息，坐标的修改
*/
void MainWindow::createCurrentWayPoint(int type_id, Way_Point_ &wp) {
  initVmapProperty(type_id, wp);
  switch (type_id) {
    case (int)Vm_T::Lane: {
      wp.property_type.P1_TYPE =
          PRO_LANE_TEST::getProway(maptypetype_comboBox->currentIndex(),
                                   waytypetype_comboBox->currentIndex(),
                                   roomtype_comboBox->currentIndex());
      wp.property_type.P2_TRAV = TRAV_LANE_TEST::getTravel(
          traveldirectiontype_comboBox->currentIndex(), 0,
          traveltype_comboBox->currentIndex(),
          travelslopedtype_comboBox->currentIndex());
      PASS_LANE_TEST test(wp.property_type.P3_PASS);
      test.pass_type.TYPE1 = bypasstype_comboBox->currentIndex();
      wp.property_type.P3_PASS = test.pass;
      if (vmap_mode_checkbox->isChecked() &&
          lanelet2map_mode_checkbox->isChecked()) {
        int laneletid = lanelet2_ids_.size() == 0 ? 0 : lanelet2_ids_.back();
        laneletid_edit->setText(QString::number(laneletid));
      }
      wp.lanelet_id = atof(laneletid_edit->text().toStdString().c_str());
    } break;
    case (int)Vm_T::Signal: {
      wp.property_type.P1_TYPE =
          PRO_SIGNAL_TEST::getProway(maptypetype_comboBox->currentIndex(),
                                     waytypetype_comboBox->currentIndex());
      PASS_SIGNAL_TEST sigtest(wp.property_type.P3_PASS);
      sigtest.pass_type.TYPE1 = bypasstype_comboBox->currentIndex();
      wp.property_type.P3_PASS = sigtest.pass;
    } break;
    case (int)Vm_T::StopPoint: {
      wp.property_type.P1_TYPE = PRO_STOPPOINT_TEST::getProway(
          maptypetype_comboBox->currentIndex());
    } break;
    case (int)Vm_T::DeceZone: {
      wp.property_type.P1_TYPE = PRO_DECEZONE_TEST::getProway(
          maptypetype_comboBox->currentIndex());
    } break;
    case (int)Vm_T::ClearArea: {
      wp.property_type.P1_TYPE = PRO_CLEARAREA_TEST::getProway(
          maptypetype_comboBox->currentIndex());
    } break;
    case (int)Vm_T::Gate: {
      wp.property_type.P1_TYPE =
          PRO_GATE_TEST::getProway(maptypetype_comboBox->currentIndex(),
                                   waytypetype_comboBox->currentIndex());
      PASS_GATE_TEST gatetest(wp.property_type.P3_PASS);
      gatetest.pass_type.TYPE1 = bypasstype_comboBox->currentIndex();
      wp.property_type.P3_PASS = gatetest.pass;
    } break;
    case (int)Vm_T::Elevator: {
      wp.property_type1.P_TYPE = PRO_ELEVATOR_TEST::getProway(
          atoi(limitvelocity_edit->text().toStdString().c_str()));
      std::string str = laneletid_edit->text().toStdString();
      if (str.size() > 3) {
        QMessageBox::question(this, tr("提示"), tr("最多存三个字符！"),
                              QMessageBox::Yes);
        laneletid_edit->setText("");
        return;
      }
      char ele_level[3] = "";
      char str_c[str.size()];
      strcpy(str_c, str.c_str());
      // //#pragma omp parallel for
      for (int i = 0; i < str.size(); i++) {
        // if (str_c[i] == '\0') break;
        ele_level[i] = str_c[i];
      }
      wp.property_type1.P_PASS[0] = ele_level[0];
      wp.property_type1.P_PASS[1] = ele_level[1];
      wp.property_type1.P_PASS[2] = ele_level[2];
    } break;
    case (int)Vm_T::AttributeArea: {
      wp.property_type.P1_TYPE =
          PRO_LANE_TEST::getProway(maptypetype_comboBox->currentIndex(),
                                   waytypetype_comboBox->currentIndex(),
                                   roomtype_comboBox->currentIndex());
      wp.property_type.P2_TRAV = TRAV_LANE_TEST::getTravel(
          traveldirectiontype_comboBox->currentIndex(), 0,
          traveltype_comboBox->currentIndex(),
          travelslopedtype_comboBox->currentIndex());
      PASS_LANE_TEST test(wp.property_type.P3_PASS);
      test.pass_type.TYPE1 = bypasstype_comboBox->currentIndex();
      wp.property_type.P3_PASS = test.pass;
      if (vmap_mode_checkbox->isChecked() &&
          lanelet2map_mode_checkbox->isChecked()) {
        int laneletid = lanelet2_ids_.size() == 0 ? 0 : lanelet2_ids_.back();
        laneletid_edit->setText(QString::number(laneletid));
      }
      wp.lanelet_id = atof(laneletid_edit->text().toStdString().c_str());
    } break;
    default:
      wp.property_type.P1_TYPE = 0;
      break;
  }
  if (type_id == (int)Vm_T::Elevator) {
    wp.limit_vel = 5.0;
  } else {
    wp.limit_vel =
        limitvelocity_edit->text() == "--"
            ? 5.0
            : atof(limitvelocity_edit->text().toStdString().c_str());
  }
  wp.ltype_type.LINKTYPE = linktype_comboBox->currentIndex();
  wp.ltype_type.LINKNUM =
      atoi(linktypenum_edit->text().toStdString().c_str());
  mapwidget_->localCoordinatesToGps(wp);
  wp.satfix.altitude = wp.point.z;
}

bool MainWindow::choiceVmapNearPoint(int type_id, Way_Point_ wp) {
  bool ret = false;
  int index = -1;
  int typeNumId = -1;
  point_ point;
  point.x = wp.point.x;
  point.y = wp.point.y;
  point.z = wp.point.z;
  std::tie(index, typeNumId) =
      mapwidget_->findCurrentNearPointNumber(type_id, point);
  if (typeNumId >= 0) {
    maptype_spinBox->setValue(typeNumId);  //会改变current_point_index
    //注意:后设置
    current_point_index_ = index <= 0 ? 0 : index - 1;
    ret = true;
  }
  return ret;
}
//网格信息刷新显示
void MainWindow::updateGridParamEdit() {
  count_edit->setText(QString::number(mapwidget_->getGridSize()));
  length_edit->setText(QString("%1").arg(mapwidget_->getGridCellLength()));
  width_edit->setText(QString("%1").arg(mapwidget_->getGridLineWidth()));

  count_edit->setEnabled(true);
  length_edit->setEnabled(true);
  width_edit->setEnabled(true);
}
void MainWindow::setQWidgetRegularStatus() {
  regulardialog_->refid_edit_1->setEnabled(false);
  regulardialog_->refidrole_edit_1->setEnabled(false);
  regulardialog_->refidrole_edit_2->setEnabled(false);
  regulardialog_->refidrole_edit_3->setEnabled(false);
  regulardialog_->refidrole_edit_4->setEnabled(false);
  regulardialog_->refidrole_edit_5->setEnabled(false);
  dialogtools_->updateComboBox(regulardialog_->element_comboBox, regulatory_elements_str,
                 RegulatoryElements::RegulatoryElementsNUM, true);
  dialogtools_->updateComboBox(regulardialog_->maneuver_comboBox, regulatory_maneuver_str,
                 RegulatoryManeuver::RegulatoryManeuverNUM, true);
}

int MainWindow::setCurrentRegularComboxIndex(std::string value_str, int num) {
  int index = -1;
  if (num == 0) {
    // //#pragma omp parallel for
    for (int i = 0; i < RegulatoryManeuverNUM; i++) {
      std::string str = regulatory_maneuver_str[i];
      if (str == value_str) {
        index = i;
      }
    }
  } else if (num == 1) {
    // //#pragma omp parallel for
    for (int i = 0; i < RegulatoryElementsNUM; i++) {
      std::string str = regulatory_elements_str[i];
      if (str == value_str) {
        index = i;
      }
    }
  } else if (num = -1) {
  }  // TO DO role1_combox
  return index;
}
/*界面刷新显示
vmap. lanelet地图的刷新显示
消息栏显示:vmap. lanelet当期状态的显示
坐标栏的刷新显示
插件的状态刷新
lanelet，search栏的清空
*/
void MainWindow::updateDisplayMapElement(void) {
  int type_id = getCurrentVmapTypeID();
  int type_num_id = getCurrentVmapTypeNumID();
  // static bool view_falg;
  std::cout << "type_id: " << type_id << "numd_id: " << type_num_id << "current: " << current_point_index_ << std::endl;
  if (rBtn_addNone->isChecked()) {
    mapwidget_->updateShowVMapData(type_id, type_num_id, current_point_index_, false);
  } else {
    mapwidget_->updateShowVMapData(type_id, type_num_id, current_point_index_, true);
  }
  mapwidget_->updateShowLaneletMapData(current_point3d_id_, lanelet2_ids_);
  if (vmap_mode_checkbox->isChecked()) {
    VectorMap_ vectormap;
    bool flag = mapwidget_->getCurrentVectorMap(type_id, type_num_id, vectormap);
    int num = vectormap.wp.size();
    vmapnum_label->setText(QString("*类型%1的编号%2\n坐标总个数为%3.")
                                    .arg(type_name_str[type_id].c_str())
                                    .arg(type_num_id)
                                    .arg(num));
    updateCoordinatesProDisplay();
  }
  if (lanelet2map_mode_checkbox->isChecked()) {
    int num;
    std::string regularid_str;
    std::string nextlaneletid_str;
    if (lanelet2_ids_.empty()) {
      num = 0;
    } else {
      num = lanelet2_ids_.back();
    }
    if (type_id == (int)Vm_T::Lane || type_id == (int)Vm_T::RoadEdge) {
      regularid_str = updateCurrentTypeRegular();
      nextlaneletid_str = getNextLaneletId();
      if (regularid_str == "") regularid_str = "NULL";
      if (nextlaneletid_str == "") nextlaneletid_str = "NULL";
      lanelet2id_label->setText(
          QString("*类型%1的ID为%2\n(nextid:%3,regids:%4).")
              .arg(type_name_str[type_id].c_str())
              .arg(num)
              .arg(nextlaneletid_str.c_str())
              .arg(regularid_str.c_str()));
    } else {
      lanelet2id_label->setText(QString("*类型%1的ID\n编号为%2.")
                                        .arg(type_name_str[type_id].c_str())
                                        .arg(num));
    }
    updateCoordinatesTagDisplay();
  }
  if (!rBtn_pointCorrection->isChecked() ||
      !lanelet2map_mode_checkbox->isChecked()) {
    // laneletid_search_edit->setText("");
    // linestrings_searched_label->setText(QString("--"));
    // linestringid_search_edit->setText("");
    // points_searched_label->setText(QString("--"));
  }
  updateQWidgetEnableStatus();
}
//在界面插件，显示当前vmap点的属性和坐标
void MainWindow::updateCoordinatesProDisplay() {
  Way_Point_ wp;
  if (mapwidget_->getCurrentTypeNumIdPoint(wp)) {
    mapwidget_->setMapZmin(wp.point.z);
    int type_id = getCurrentVmapTypeID();
    localx_edit->setText(QString::number(wp.point.x, 'f', 2));
    localy_edit->setText(QString::number(wp.point.y, 'f', 2));
    localz_edit->setText(QString::number(wp.point.z, 'f', 2));
    latitude_edit->setText(QString::number(wp.satfix.latitude, 'f', 8));
    longitude_edit->setText(QString::number(wp.satfix.longitude, 'f', 8));
    switch (type_id) {
      case (int)Vm_T::Lane: {
        PRO_LANE_TEST testpro(wp.property_type.P1_TYPE);
        maptypetype_comboBox->setCurrentIndex(testpro.pro_type.TYPE1);
        waytypetype_comboBox->setCurrentIndex(testpro.pro_type.TYPE2);
        roomtype_comboBox->setCurrentIndex(testpro.pro_type.TYPE3);
        PASS_LANE_TEST testpass(wp.property_type.P3_PASS);
        bypasstype_comboBox->setCurrentIndex(testpass.pass_type.TYPE1);
        TRAV_LANE_TEST testtra(wp.property_type.P2_TRAV);
        traveldirectiontype_comboBox->setCurrentIndex(testtra.travel_type.TYPE1);
        traveltype_comboBox->setCurrentIndex(testtra.travel_type.TYPE3);
        travelslopedtype_comboBox->setCurrentIndex(testtra.travel_type.TYPE4);
        laneletid_edit->setText(QString("%1").arg(wp.lanelet_id));
      } break;
      case (int)Vm_T::AttributeArea: {
        PRO_LANE_TEST testpro(wp.property_type.P1_TYPE);
        maptypetype_comboBox->setCurrentIndex(testpro.pro_type.TYPE1);
        waytypetype_comboBox->setCurrentIndex(testpro.pro_type.TYPE2);
        roomtype_comboBox->setCurrentIndex(testpro.pro_type.TYPE3);
        PASS_LANE_TEST testpass(wp.property_type.P3_PASS);
        bypasstype_comboBox->setCurrentIndex(testpass.pass_type.TYPE1);
        TRAV_LANE_TEST testtra(wp.property_type.P2_TRAV);
        traveldirectiontype_comboBox->setCurrentIndex(testtra.travel_type.TYPE1);
        traveltype_comboBox->setCurrentIndex(testtra.travel_type.TYPE3);
        travelslopedtype_comboBox->setCurrentIndex(testtra.travel_type.TYPE4);
        laneletid_edit->setText(QString("%1").arg(wp.lanelet_id));
      } break;
      case (int)Vm_T::Signal: {
        PRO_SIGNAL_TEST testpro(wp.property_type.P1_TYPE);
        maptypetype_comboBox->setCurrentIndex(testpro.pro_type.TYPE1);
        waytypetype_comboBox->setCurrentIndex(testpro.pro_type.TYPE2);
        PASS_SIGNAL_TEST testpass(wp.property_type.P3_PASS);
        bypasstype_comboBox->setCurrentIndex(testpass.pass_type.TYPE1);
      } break;
      case (int)Vm_T::StopPoint: {
        PRO_STOPPOINT_TEST testpro(wp.property_type.P1_TYPE);
        maptypetype_comboBox->setCurrentIndex(testpro.pro_type.TYPE1);
      } break;
      case (int)Vm_T::DeceZone: {
        PRO_DECEZONE_TEST testpro(wp.property_type.P1_TYPE);
        maptypetype_comboBox->setCurrentIndex(testpro.pro_type.TYPE1);
      } break;
      case (int)Vm_T::ClearArea: {
        PRO_CLEARAREA_TEST testpro(wp.property_type.P1_TYPE);
        maptypetype_comboBox->setCurrentIndex(testpro.pro_type.TYPE1);
      } break;
      case (int)Vm_T::Gate: {
        PRO_GATE_TEST testpro(wp.property_type.P1_TYPE);
        maptypetype_comboBox->setCurrentIndex(testpro.pro_type.TYPE1);
        waytypetype_comboBox->setCurrentIndex(testpro.pro_type.TYPE2);
        PASS_GATE_TEST testpass(wp.property_type.P3_PASS);
        bypasstype_comboBox->setCurrentIndex(testpass.pass_type.TYPE1);
      } break;
      case (int)Vm_T::Elevator: {
        PRO_ELEVATOR_TEST testpro(wp.property_type1.P_TYPE);
        limitvelocity_edit->setText(QString("%1").arg(testpro.pro_type.TYPE1));
        PASS_ELEVATOR_TEST testpass(wp.property_type1.P_PASS);
        std::string ele_level = testpass.getPass(testpass.pass[0], testpass.pass[1], 
                                                 testpass.pass[2]);
        laneletid_edit->setText(QString::fromStdString(ele_level));
      } break;
      default: {
        maptypetype_comboBox->setCurrentIndex(0);
      } break;
    }
    linktype_comboBox->setCurrentIndex(wp.ltype_type.LINKTYPE);
    linktypenum_edit->setText(QString("%1").arg(wp.ltype_type.LINKNUM));
    if (type_id != (int)Vm_T::Elevator)
      limitvelocity_edit->setText(QString("%1").arg(wp.limit_vel));
  } else {
    maptypetype_comboBox->setCurrentIndex(0);
    waytypetype_comboBox->setCurrentIndex(0);
    bypasstype_comboBox->setCurrentIndex(0);
    traveldirectiontype_comboBox->setCurrentIndex(0);
    traveltype_comboBox->setCurrentIndex(0);
    travelslopedtype_comboBox->setCurrentIndex(0);
    linktype_comboBox->setCurrentIndex(0);
    roomtype_comboBox->setCurrentIndex(0);
    linktypenum_edit->setText("--");
    limitvelocity_edit->setText("--");
    laneletid_edit->setText("--");
  }
}
//在界面插件，显示当前lanelet点坐标
void MainWindow::updateCoordinatesTagDisplay() {
  lanelet::Point3d pt;
  if (current_point3d_id_ > 0 &&
      mapwidget_->findPoint3dWithId(current_point3d_id_, pt)) {
    mapwidget_->setMapZmin(pt.z());
    localx_edit->setText(QString::number(pt.x(), 'f', 2));
    localy_edit->setText(QString::number(pt.y(), 'f', 2));
    localz_edit->setText(QString::number(pt.z(), 'f', 2));
    ////#pragma omp parallel for
    for (auto p : pt.attributes()) {
      std::string k = p.first;
      std::string v = p.second.value();
      if (k == "gps_lat") {
        double lat = atof(v.c_str());
        latitude_edit->setText(QString::number(lat, 'f', 8));
      }
      if (k == "gps_lon") {
        double lon = atof(v.c_str());
        longitude_edit->setText(QString::number(lon, 'f', 8));
      }
    }
  } else {
    localx_edit->setText("--");
    localy_edit->setText("--");
    localz_edit->setText("--");
    latitude_edit->setText("--");
    longitude_edit->setText("--");
  }
}

std::string MainWindow::updateCurrentTypeRegular() {
  // for show current lanelet's regular
  std::string str;
  lanelet::Lanelet ll;
  regularid_edit->setEnabled(false);
  regularid_edit->setText(QString("--"));
  int type_id = getCurrentVmapTypeID();
  if (lanelet2_ids_.size() == 0 ||
      (type_id != int(Vm_T::Lane) && type_id != int(Vm_T::RoadEdge)))
    return str;
  if (!mapwidget_->findLaneletWithId(lanelet2_ids_.back(), ll)) return str;
  bool get_first = false;
  ////#pragma omp parallel for
  std::vector<reguinfo_list> regularinfo_lists = mapwidget_->getRegularInfoList();
  for (auto &regularlist : regularinfo_lists) {
    if (regularlist.laneletid == lanelet2_ids_.back()) {
      if (!get_first) {
        regularid_edit->setEnabled(true);
        regularid_edit->setText(
            QString::number(regularlist.reglists.Regelement_id));
        get_first = true;
      }
      str = str + std::to_string(regularlist.reglists.Regelement_id) + " ";
    }
  }
  return str;
}
std::string MainWindow::getNextLaneletId() {
  std::string str;
  lanelet::Lanelet ll;
  int type_id = getCurrentVmapTypeID();
  if (lanelet2_ids_.size() == 0 ||
      (type_id != int(Vm_T::Lane) && type_id != int(Vm_T::RoadEdge)))
    return str;
  if (!mapwidget_->findLaneletWithId(lanelet2_ids_.back(), ll)) return str;
  if (ll.leftBound().size() == 0 || ll.rightBound().size() == 0) return str;
  std::vector<int> ids = mapwidget_->findNextLanelet(ll);
  ////#pragma omp parallel for
  for (auto &id : ids) {
    str = str + std::to_string(id) + " ";
  }
  return str;
}
/*tag dialog - 对话框中小插件状态的设置
显示当前机动车道元素，被添加的tag信息
*/
bool MainWindow::updateCurrentTypeTags() {
  ////for show current lanelet or linestring tags
  bool havetags = false;
  taginfodialog_->key_edit_1->setText(QString(""));
  taginfodialog_->value_edit_1->setText(QString(""));
  taginfodialog_->key_edit_2->setText(QString(""));
  taginfodialog_->value_edit_2->setText(QString(""));
  lanelet::Lanelet ll;
  lanelet::LineString3d ls;
  std::vector<tag_list> taginfolists = mapwidget_->getTagInfoList();
  if (lanelet2_ids_.size() == 0) return havetags;
  if (mapwidget_->findLaneletWithId(lanelet2_ids_.back(), ll) ||
      mapwidget_->findLinestringWithId(lanelet2_ids_.back(), ls)) {
    taginfodialog_->idtag_label->setText(
        QString("*linestring/lanelet id:%1").arg(lanelet2_ids_.back()));
    ////#pragma omp parallel for
    for (auto taglist : taginfolists) {
      if (taglist.id == lanelet2_ids_.back()) {
        // bool key1get = false;
        for (auto tag : taglist.tags) {
          if (!havetags) {
            taginfodialog_->key_edit_1->setText(QString::fromStdString(tag.k));
            taginfodialog_->value_edit_1->setText(QString::fromStdString(tag.v));
            // std::cout << "yes do " << tag.v << std::endl;
            havetags = true;
            continue;
          }
          taginfodialog_->key_edit_2->setText(QString::fromStdString(tag.k));
          taginfodialog_->value_edit_2->setText(QString::fromStdString(tag.v));
          // std::cout << "yes do .." << tag.k << std::endl;
        }
      }
    }
  }
  return havetags;
}

/*园区地图vmap:
不同地图元素，关注的属性不同,
对vmap属性栏小插件是否显示进行设置.
*/
void MainWindow::updateVmapcomboBoxInfo() {  
  int type_id = getCurrentVmapTypeID();
  std::cout << "init type_id = " << type_id << std::endl;
  // int typetype_id = getCurrentVmapTypeTypeID();
  switch (type_id) {
    case (int)Vm_T::Lane: {
      dialogtools_->updateComboBox(maptypetype_comboBox, TYPE_P1_TYPE_LANE_STR,
                     TYPE_P1_TYPE_LANE::TYPE_P1_TYPE_LANE_NUM, true);
      dialogtools_->updateComboBox(waytypetype_comboBox, TYPE_P1_TYPE1_LANE_STR,
                     TYPE_P1_TYPE1_LANE::TYPE_P1_TYPE1_LANE_NUM, true);
      dialogtools_->updateComboBox(bypasstype_comboBox, TYPE_P3_PASS_LANE_STR,
                     TYPE_P3_PASS_LANE::TYPE_P3_PASS_LANE_NUM, true);
      dialogtools_->updateComboBox(traveldirectiontype_comboBox, TYPE_P2_TRAV1_LANE_STR,
                     TYPE_P2_TRAV1_LANE::TYPE_P2_TRAV1_LANE_NUM, true);
      dialogtools_->updateComboBox(traveltype_comboBox, TYPE_P2_TRAV3_LANE_STR,
                     TYPE_P2_TRAV3_LANE::TYPE_P2_TRAV3_LANE_NUM, true);
      dialogtools_->updateComboBox(travelslopedtype_comboBox, TYPE_P2_TRAV4_LANE_STR,
                     TYPE_P2_TRAV4_LANE::TYPE_P2_TRAV4_LANE_NUM, true);
      dialogtools_->updateComboBox(roomtype_comboBox, TYPE_P1_TYPE2_LANE_STR,
                     TYPE_P1_TYPE2_LANE::TYPE_P1_TYPE2_LANE_NUM, true);
      dialogtools_->updateQLineEdit(laneletid_edit, true);
      dialogtools_->updateQLineEdit(limitvelocity_edit, true);
    } break;
    case (int)Vm_T::Signal: {
      dialogtools_->updateComboBox(maptypetype_comboBox, TYPE_P1_TYPE_SIGNAL_STR,
                     TYPE_P1_TYPE_SIGNAL::TYPE_P1_TYPE_SIGNAL_NUM, true);
      dialogtools_->updateComboBox(waytypetype_comboBox, TYPE_P1_TYPE1_SIGNAL_STR,
                     TYPE_P1_TYPE1_SIGNAL::TYPE_P1_TYPE1_SIGNAL_NUM, true);
      dialogtools_->updateComboBox(bypasstype_comboBox, TYPE_P3_PASS_SIGNAL_STR,
                     TYPE_P3_PASS_SIGNAL::TYPE_P3_PASS_SIGNAL_NUM, true);
      dialogtools_->updateComboBox(traveldirectiontype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(traveltype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(travelslopedtype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(roomtype_comboBox, TYPE_P1_TYPE2_LANE_STR,
                     TYPE_P1_TYPE2_LANE::TYPE_P1_TYPE2_LANE_NUM, false);
      dialogtools_->updateQLineEdit(laneletid_edit, false);
      dialogtools_->updateQLineEdit(limitvelocity_edit, false);
    } break;
    case (int)Vm_T::Gate: {
      dialogtools_->updateComboBox(maptypetype_comboBox, TYPE_P1_TYPE_GATE_STR,
                     TYPE_P1_TYPE_GATE::TYPE_P1_TYPE_GATE_NUM, true);
      dialogtools_->updateComboBox(waytypetype_comboBox, TYPE_P1_TYPE1_GATE_STR,
                     TYPE_P1_TYPE1_GATE::TYPE_P1_TYPE1_GATE_NUM, true);
      dialogtools_->updateComboBox(bypasstype_comboBox, TYPE_P3_PASS_GATE_STR,
                     TYPE_P3_PASS_GATE::TYPE_P3_PASS_GATE_NUM, true);
      dialogtools_->updateComboBox(traveldirectiontype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(traveltype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(travelslopedtype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(roomtype_comboBox, TYPE_P1_TYPE2_LANE_STR,
                     TYPE_P1_TYPE2_LANE::TYPE_P1_TYPE2_LANE_NUM, false);
      dialogtools_->updateQLineEdit(laneletid_edit, false);
      dialogtools_->updateQLineEdit(limitvelocity_edit, false);
    } break;
    case (int)Vm_T::StopPoint: {
      dialogtools_->updateComboBox(maptypetype_comboBox, TYPE_P1_TYPE_STOPPOINT_STR,
                     TYPE_P1_TYPE_STOPPOINT::TYPE_P1_TYPE_STOPPOINT_NUM, true);
      dialogtools_->updateComboBox(waytypetype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(bypasstype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(traveldirectiontype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(traveltype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(travelslopedtype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(roomtype_comboBox, TYPE_P1_TYPE2_LANE_STR,
                     TYPE_P1_TYPE2_LANE::TYPE_P1_TYPE2_LANE_NUM, false);
      dialogtools_->updateQLineEdit(laneletid_edit, false);
      dialogtools_->updateQLineEdit(limitvelocity_edit, false);
    } break;
    case (int)Vm_T::DeceZone: {
      dialogtools_->updateComboBox(maptypetype_comboBox, TYPE_P1_TYPE_DECEZONE_STR,
                     TYPE_P1_TYPE_DECEZONE::TYPE_P1_TYPE_DECEZONE_NUM, true);
      dialogtools_->updateComboBox(waytypetype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(bypasstype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(traveldirectiontype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(traveltype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(travelslopedtype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(roomtype_comboBox, TYPE_P1_TYPE2_LANE_STR,
                     TYPE_P1_TYPE2_LANE::TYPE_P1_TYPE2_LANE_NUM, false);
      dialogtools_->updateQLineEdit(laneletid_edit, false);
      dialogtools_->updateQLineEdit(limitvelocity_edit, false);
    } break;
    case (int)Vm_T::ClearArea: {
      dialogtools_->updateComboBox(maptypetype_comboBox, TYPE_P1_TYPE_CLEARAREA_STR,
                     TYPE_P1_TYPE_CLEARAREA::TYPE_P1_TYPE_CLEARAREA_NUM, true);
      dialogtools_->updateComboBox(waytypetype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(bypasstype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(traveldirectiontype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(traveltype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(travelslopedtype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(roomtype_comboBox, TYPE_P1_TYPE2_LANE_STR,
                     TYPE_P1_TYPE2_LANE::TYPE_P1_TYPE2_LANE_NUM, false);
      dialogtools_->updateQLineEdit(laneletid_edit, false);
      dialogtools_->updateQLineEdit(limitvelocity_edit, false);
    } break;
    case (int)Vm_T::Elevator: {
      dialogtools_->updateComboBox(maptypetype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(waytypetype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(bypasstype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(traveldirectiontype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(traveltype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(travelslopedtype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(roomtype_comboBox, TYPE_P1_TYPE2_LANE_STR,
                     TYPE_P1_TYPE2_LANE::TYPE_P1_TYPE2_LANE_NUM, false);
      dialogtools_->updateQLineEdit(laneletid_edit, true);
      dialogtools_->updateQLineEdit(limitvelocity_edit, true);
    } break;
    case (int)Vm_T::AttributeArea: {
      dialogtools_->updateComboBox(maptypetype_comboBox, TYPE_P1_TYPE_LANE_STR,
                     TYPE_P1_TYPE_LANE::TYPE_P1_TYPE_LANE_NUM, true);
      dialogtools_->updateComboBox(waytypetype_comboBox, TYPE_P1_TYPE1_LANE_STR,
                     TYPE_P1_TYPE1_LANE::TYPE_P1_TYPE1_LANE_NUM, true);
      dialogtools_->updateComboBox(bypasstype_comboBox, TYPE_P3_PASS_LANE_STR,
                     TYPE_P3_PASS_LANE::TYPE_P3_PASS_LANE_NUM, true);
      dialogtools_->updateComboBox(traveldirectiontype_comboBox, TYPE_P2_TRAV1_LANE_STR,
                     TYPE_P2_TRAV1_LANE::TYPE_P2_TRAV1_LANE_NUM, true);
      dialogtools_->updateComboBox(traveltype_comboBox, TYPE_P2_TRAV3_LANE_STR,
                     TYPE_P2_TRAV3_LANE::TYPE_P2_TRAV3_LANE_NUM, true);
      dialogtools_->updateComboBox(travelslopedtype_comboBox, TYPE_P2_TRAV4_LANE_STR,
                     TYPE_P2_TRAV4_LANE::TYPE_P2_TRAV4_LANE_NUM, true);
      dialogtools_->updateComboBox(roomtype_comboBox, TYPE_P1_TYPE2_LANE_STR,
                     TYPE_P1_TYPE2_LANE::TYPE_P1_TYPE2_LANE_NUM, true);
      dialogtools_->updateQLineEdit(laneletid_edit, true);
      dialogtools_->updateQLineEdit(limitvelocity_edit, true);
    } break;
    default: {
      dialogtools_->updateComboBox(maptypetype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, true);
      dialogtools_->updateComboBox(waytypetype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(bypasstype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(traveldirectiontype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(traveltype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(travelslopedtype_comboBox, TYPE_NULLDATA_STR,
                     TYPE_NULLDATA::TYPE_NULLDATA_NUM, false);
      dialogtools_->updateComboBox(roomtype_comboBox, TYPE_P1_TYPE2_LANE_STR,
                     TYPE_P1_TYPE2_LANE::TYPE_P1_TYPE2_LANE_NUM, false);
      dialogtools_->updateQLineEdit(laneletid_edit, false);
      dialogtools_->updateQLineEdit(limitvelocity_edit, false);
    } break;
  }
}
//点击上传，总会保存一份当前时刻的vmap数据，根据项目名称，到本地的AutoSaveVmap文件夹内
void MainWindow::autoTimeSaveProjectData() {
  QDateTime current_date_time =QDateTime::currentDateTime();
  QString current_date =current_date_time.toString("yyyy.MM.dd-hh:mm:ss");
  std::string auto_time_save = home_str + "/AutoSaveVmap";
  QDir dir(QString::fromStdString(auto_time_save));
  if (!dir.exists()) {
    dir.mkdir(QString::fromStdString(auto_time_save));
  }
  {
    if (ProjectName_ == "") {
      auto_time_save = auto_time_save + "/auto" + current_date.toStdString();
    } else {
      auto_time_save = auto_time_save + "/" + ProjectName_.toStdString();
      QDir dirs(QString::fromStdString(auto_time_save));
      if (!dirs.exists()) {
        dirs.mkdir(QString::fromStdString(auto_time_save));
      }
      auto_time_save = auto_time_save + "/" + current_date.toStdString();
    }
    QDir dirs(QString::fromStdString(auto_time_save));
    if (!dirs.exists()) {
      dirs.mkdir(QString::fromStdString(auto_time_save));
    }
  }
  mapwidget_->writeVmapFiles(auto_time_save);
  {
    auto_time_save = auto_time_save + "/hdmap.osm";
    if (!mapwidget_->laneletmapEmpty()) {
      mapwidget_->autoSaveFile(auto_time_save);
      std::cout << "auto save filished ..." << std::endl;
    }
  }
}

//点击上传，总会自动保存所有数据到project文件内
void MainWindow::autoSaveAllProjectData() {
  //导入原始数据（导入任务栏），上传自动导出pcd&_r文件保存，否则不导出pcd文件
  bool mapExportStatu,rmapExportStatu;
  //文件路径
  QString suffixName,suffixName_r,filepath,filepath_r;
  filepath = QString::fromStdString(HOME_STR) + "/maptool-candela/CTI_MAP_PROJECT/" + ProjectName_ + "/MAP/" + ProjectName_ + "/";
  suffixName = ProjectName_ + ".pcd";

  filepath_r = QString::fromStdString(HOME_STR) + "/maptool-candela/CTI_MAP_PROJECT/" + ProjectName_ + "/RMAP/" + ProjectName_ + "/";
  suffixName_r = ProjectName_ + "_r.pcd";

  if (isTask_)
  {
    std::cout << "777777777777" << std::endl;
    //pcd-cloud
    {
      std::promise<bool> promiseObj;
      std::future<bool> futureObj = promiseObj.get_future();
      std::thread thExportCloudMap(&MainWindow::savePointCloudPcdFile, this, std::ref(ProjectName_),std::ref(key_frames_),std::ref(map_zero_gnss_),true,0.2,true,std::ref(promiseObj));
      mapExportStatu = futureObj.get();
      thExportCloudMap.detach();
    }
    if (mapExportStatu)
    {
      QString path_str = filepath + suffixName;
      QString info = "文件已成功存入：\n" + path_str;
      QMessageBox::information(this, tr("成功！"), info, QMessageBox::Yes);
    } else {
      QString path_str = filepath + suffixName;
      QString info = "文件存入失败！！！请检查路径信息：\n" + path_str;
      QMessageBox::information(this, tr("失败！"), info, QMessageBox::Yes);
    }
    // savePointCloudPcdFile(ProjectName_,key_frames_,map_zero_gnss_,true,0.2,true);

    // rpcd-cloud
    {
      std::promise<bool> promiseObj;
      std::future<bool> futureObj = promiseObj.get_future();
      std::thread thExportRCloudMap(&MainWindow::savePointCloudPcdFile, this, std::ref(ProjectName_),std::ref(key_frames_),std::ref(map_zero_gnss_),false,0.2,true,std::ref(promiseObj));
      rmapExportStatu = futureObj.get();
      thExportRCloudMap.detach();
    }
    if (rmapExportStatu)
    {
      QString path_str = filepath_r + suffixName_r;
      QString info = "文件已成功存入：\n" + path_str;
      QMessageBox::information(this, tr("成功！"), info, QMessageBox::Yes);
    } else {
      QString path_str = filepath_r + suffixName_r;
      QString info = "文件存入失败！！！请检查路径信息：\n" + path_str;
      QMessageBox::information(this, tr("失败！"), info, QMessageBox::Yes);
    }
    // savePointCloudPcdFile(ProjectName_,key_frames_,map_zero_gnss_,false,0.2,true);

    //导出/更新原始数据文件
    datafilesave_pbtn_clicked();
    //导出/更新vmap文件
    QString message = "已保存地图文件至 " + project_vmap_ + " ！";
    std::string project_vmap = project_vmap_.toStdString();
    mapwidget_->writeVmapFiles(project_vmap);
    QMessageBox::question(this, tr("提示"), message,
                            QMessageBox::Yes);
    isTask_ = false;
  }
  else //导入原始文件和pcd文件（导入项目栏），上传自动保存vmap文件
  {
    //导出/更新vmap文件
    std::cout << "888888888888888" << std::endl;
    QString message = "已保存地图文件至 " + project_vmap_ + " ！";
    std::string project_vmap = project_vmap_.toStdString();
    mapwidget_->writeVmapFiles(project_vmap);
    QMessageBox::question(this, tr("提示"), message,
                            QMessageBox::Yes);
  }
}

//save pcd/rpcd
void MainWindow::savePointCloudPcdFile(QString &project_name,
                                      std::deque<KeyFrame> &key_frames,
                                      boost::optional<Eigen::Vector3d> &map_zero_gnss,
                                      bool downsample_flag,
                                      double downsample_resolution, 
                                      bool saveCompressedPCD,
                                      std::promise<bool> &promiseObj) {
  bool result = mapwidget_->savePointCloudPcdFile(project_name,key_frames,map_zero_gnss,downsample_flag,downsample_resolution,saveCompressedPCD);
  promiseObj.set_value(result);
}
//grid-color
void MainWindow::getGridValidColor() {
  float color_rgba_a;
  Ogre::ColourValue color;
  QColor cell_color = QColorDialog::getColor(Qt::white, this);
  if (!cell_color.isValid()) {
    color = mapwidget_->getGridColor();
    QString color_string = QString::number(color.r*255) + ":" 
                         + QString::number(color.g*255) + ":" 
                         + QString::number(color.b*255); 
    griddialog_->color_info->setText(color_string);
    // Q_EMIT gridColorSignal(color);
    return;
  }
  float color_rgba_r = QString::number(cell_color.red()).toFloat() / 255;
  float color_rgba_g = QString::number(cell_color.green()).toFloat() / 255;
  float color_rgba_b = QString::number(cell_color.blue()).toFloat() / 255;
  color = Ogre::ColourValue(color_rgba_r, color_rgba_g, color_rgba_b, 0.5f);
  QString color_string = QString::number(color.r*255) + ":" 
                         + QString::number(color.g*255) + ":" 
                         + QString::number(color.b*255); 
  griddialog_->color_info->setText(color_string);
  // Q_EMIT gridColorSignal(color);
}
void MainWindow::setParamGrid() {
    Ogre::ColourValue color = mapwidget_->getGridColor();
    QString color_string = QString::number(color.r*255) + ":" 
                         + QString::number(color.g*255) + ":" 
                         + QString::number(color.b*255); 
    griddialog_->color_info->setText(color_string);
    griddialog_->count_edit->setText(QString::number(mapwidget_->getGridSize()));
    griddialog_->length_edit->setText(QString("%1").arg(mapwidget_->getGridCellLength()));
    griddialog_->width_edit->setText(QString("%1").arg(mapwidget_->getGridLineWidth()));
}
void MainWindow::gridParamSet() {
  QString color_string = griddialog_->enteredCellColor();
  if (color_string != "")
  {
    QStringList color_strings = color_string.split(":");
    float color_rgba_r = color_strings.at(0).toFloat() / 255;
    float color_rgba_g = color_strings.at(1).toFloat() / 255;
    float color_rgba_b = color_strings.back().toFloat() / 255;
    mapwidget_->setGridColor(Ogre::ColourValue(color_rgba_r, color_rgba_g, color_rgba_b, 0.5f));
  }
  uint32_t grid_size = griddialog_->enteredCellCount().toInt();
  float grid_length = atof(griddialog_->enteredCellLength().toStdString().c_str());
  float grid_width = atof(griddialog_->enteredCellWidth().toStdString().c_str());

  mapwidget_->setGridSize(grid_size);
  mapwidget_->setGridCellLength(grid_length);
  mapwidget_->setGridLineWidth(grid_width);
}
//--
void MainWindow::getSlamDataPoses(QString &folderPath) {
  QDir dir(folderPath);
  if(!dir.exists()) {
    //判断文件路径是否存在
      QMessageBox::critical(this,tr("错误"),tr("找不到该文件夹,请检查路径!"));
      return;
  }
  //获取文件下的原始数据信息
  if (!mapwidget_->get_keyframes(folderPath.toStdString(),key_frames_))
  {
    std::cerr << "failed to get empty key_frames_" << std::endl;
    return;
  }

  QStringList folderinfos = folderPath.split("/");
  ProjectName_ = folderinfos.at(folderinfos.size()-2);

  Eigen::Vector4f origin = Eigen::Vector4f::Zero();
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
  pcl::PointCloud<PointType>::Ptr accumulated(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
  // std::vector<Way_Point_> poses;
  //获取每帧点云的位姿，转换
  keyframes_poses_.clear();
  for(const auto &key_frame : key_frames_)
  {
      // std::cout << "doing: " << key_frame.stamp << std::endl;
      Eigen::Vector3d vector3d = key_frame.estimate->translation();
      Way_Point_ pt;
      pt.point.x = vector3d[0];
      pt.point.y = vector3d[1];
      pt.point.z = vector3d[2];
      pt.point.quate_x = 0;
      pt.point.quate_y = 0;
      pt.point.quate_z = 1;
      keyframes_poses_.push_back(pt);
      pcl::io::loadPCDFile(key_frame.cloud_file, *cloud);
      pcl::transformPointCloud(*cloud, *cloud, key_frame.estimate->cast<float>());
      std::copy(cloud->begin(), cloud->end(), std::back_inserter(accumulated->points));
  }
  std::cout << "over###" << accumulated->size() << std::endl;
  //每帧cloud拼接成完整cloud
  accumulated->header.frame_id = "map";
  accumulated->is_dense = false;
  accumulated->width = accumulated->size();
  accumulated->height = 1;
  //从zero_gnss中获取cloud的origin&orientation信息
  mapwidget_->get_QuaterAndVector(folderPath,map_zero_gnss_);
  if (map_zero_gnss_) {
        origin.x() = int(map_zero_gnss_->x() * 1e3); //
        origin.y() = int(map_zero_gnss_->y() * 1e3);
        origin.z() = int(map_zero_gnss_->z() * 1e3);
        orientation.w() = 1.0;
        orientation.x() = map_zero_gnss_->x() * 1e3 - origin.x();
        orientation.y() = map_zero_gnss_->y() * 1e3 - origin.y();
        orientation.z() = map_zero_gnss_->z() * 1e3 - origin.z();
  }
  //初始化cloud，加载cloud
  bool pcd_flag = mapwidget_->initMapOriginPostion(accumulated,origin,orientation);
  if (!pcd_flag)
  {
    int reply = QMessageBox::warning(
        this, QObject::tr("提示"),
        QObject::tr("初始经纬度为0, 您要继续使用吗!!"),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    if (reply == QMessageBox::No) {
      return;
    } else {
      mapwidget_->loadPcdMap("0");
    }
  } else {
    mapwidget_->loadPcdMap("0");
  }
  std::cout << "try map init success!!" << std::endl;
  //赋值本地高度
  mapwidget_->getPcdmapRangeZ(global_points_zmin_, global_points_zmax_);
  local_points_zmin_ = global_points_zmin_;
  local_points_zmax_ = global_points_zmax_;

  pcd_z_label->setText(QString("*高度限制（%1 ~ %2.）")
                                .arg(local_points_zmax_)
                                .arg(local_points_zmin_));

  // 生成轨迹
  // createNewTrajectory(keyframes_poses_);

}
void MainWindow::createNewTrajectory(std::vector<Way_Point_> &poses) {
  int type_id = 0;
  int type_num_id = mapwidget_->getRandEmptyTypeNumId(type_id);
  std::cout << "生成行驶轨迹: " << type_id << " " << type_num_id << std::endl;
  maptype_spinBox->setValue(type_num_id);
  for (size_t i = 0; i < poses.size(); i++)
  {
    //点的创建生成
    Way_Point_ wp;
    VectorMap_ vectormap;
    wp.point.x = poses[i].point.x;
    wp.point.y = poses[i].point.y;
    wp.point.z = poses[i].point.z - lidar_height_;
    createCurrentWayPoint(type_id, wp);
    vectormap.wp.push_back(wp);  // push_back对vector_map填数据
    vectormap.type_id = type_id;  //矢量地图对应的类型，如lane,roadedge,sigald等
    vectormap.type_num_id = type_num_id;
    mapwidget_->setCurrentVectorMap(type_id, type_num_id, vectormap);
    int index = mapwidget_->getCurrentTypeNumIdNumber(type_id, type_num_id);
    current_point_index_ = index <= 0 ? 0 : index - 1;
  }
  //显示轨迹
  updateDisplayMapElement();
}

bool MainWindow::isLimitVmapWayPoint(const int limit_type,
                                     const int limit_num) {
  bool ret = false;
  int type_id = getCurrentVmapTypeID();
  int type_num_id = getCurrentVmapTypeNumID();
  if (type_id == limit_type) {
    VectorMap_ vectormap;
    bool flag = mapwidget_->getCurrentVectorMap(type_id, type_num_id, vectormap);
    int num = vectormap.wp.size();
    if (num >= limit_num) {  //判断当前id矢量数据的个数与限制的num大小关系
      ret = true;
    }
  }
  return ret;
}

////////////////////////////////////////////////////////////////////////////// showpark
void MainWindow::parkNameIdItemShow() {
  //获取环境下所有的园区信息
  listParksIdName_.clear();
  listParksIdName_ = mapwidget_->pullByToken(pubToken_);
  if (listParksIdName_.size() == 0) return;

  for (size_t i = 0; i < listParksIdName_.size(); i++)
  {
    QMap<QString, QString> origin = listParksIdName_.at(i);
    QMap<QString, QString>::iterator it = origin.begin();
    std::string parkName = it.key().toStdString();
    //设置环境下所有园区下拉框选项
    pubdialog_->parkName_combox->addItem(parkName.c_str());
  }
}
QString MainWindow::getCurrentParkText() {
  //获取下拉框内文字信息
  return pubdialog_->parkName_combox->currentText();
    // return pubdialog_->parkName_combox->currentIndex() % listParksIdName_.size();
}
int MainWindow::getCurrentParkTypeId() {
  //获取下拉框索引值
    return pubdialog_->parkName_combox->currentIndex() % listParksIdName_.size();
}
void MainWindow::parkName_comboBox_IndexChanged() {
  QString currentParkName = getCurrentParkText();
  int type_id = getCurrentParkTypeId();
  //根据正确的园区名和索引值 获取对应园区id
  QString currentParkId = listParksIdName_.at(type_id).find(currentParkName).value();
  //设置显示
  pubdialog_->parkId_edit->setText(currentParkId);
}

////////////////////////////////////////////////////////////////////////////// login
void MainWindow::getAccessToken() {
    std::string domain;
    std::string authorizationToken;
    //设置平台环境 只显示当前发布环境
    if (logindialog_->prerelease_checkbox->isChecked()) {
      domain = "staging.api.ctirobot.com";
      pubdialog_->prerelease_checkbox->setChecked(true);
      pubdialog_->produce_checkbox->setChecked(false);
    }
    else if (logindialog_->produce_checkbox->isChecked()) {
      domain = "api.ctirobot.com";
      pubdialog_->produce_checkbox->setChecked(true);
      pubdialog_->prerelease_checkbox->setChecked(false);
    }
    else {
      domain = "staging.api.ctirobot.com";
      pubdialog_->produce_checkbox->setChecked(false);
      pubdialog_->prerelease_checkbox->setChecked(false);
    }
    mapwidget_->setPlatformRequestDomain(domain);
    //设置用户权限->token
    if (logindialog_->ordinary_checkbox->isChecked()) {
      authorizationToken = authorization_oridinary_token_; //普通用户
    }
    else if (logindialog_->deploy_checkbox->isChecked() && logindialog_->prerelease_checkbox->isChecked()) {
      authorizationToken = authorization_deploy_staging_token_;  //部署用户--预发布
    }
    else if (logindialog_->deploy_checkbox->isChecked() && logindialog_->produce_checkbox->isChecked()) {
      authorizationToken = authorization_deploy_api_token_;  //部署用户--生产
    }
    else {
      authorizationToken = authorization_oridinary_token_; //普通用户
    }
    mapwidget_->setPlatformRequesauthorizationToken(authorizationToken);
    //登录连接-获取当前环境所有园区信息
    std::string userName = logindialog_->getUserName().toStdString();
    std::string passWord = logindialog_->getPassWord().toStdString();
    std::string keyDir = qApp->applicationDirPath().toStdString() + "public_key.pem";
    pubToken_ = mapwidget_->pullParkToken(userName,passWord,keyDir);
    if (pubToken_.empty()) {
      QString errormessage = "连接失败,请检查用户帐号密码或网络服务器状态是否有误。";
      QMessageBox::question(this, tr("提示"), errormessage,
                          QMessageBox::Yes);
    } else {
      if (pubdialog_->disConnectToClear()) {
        pubdialog_->parkName_combox->clear();
        pubdialog_->connectToClear();
        listParksIdName_.clear();
      }
      QString message = "连接成功!";
      QMessageBox::question(this, tr("提示"), message,
                          QMessageBox::Yes);
    }
    
    // std::cout << "token = " << pubToken_ << std::endl;
}
void MainWindow::pubInitPoseAndAreaInfo(std::string map_name, std::string map_id,
                                        int typeIndex) {
    //--当前上传发布园区名称-id号-发布环境
    std::cout << "pub..." << std::endl;
    if (pubdialog_->mapdata_checkbox->isChecked()) {
      //--单独发布区域->平台
      modifyOrPubFullData(true,map_name,map_id,typeIndex);
    } else if (pubdialog_->initpose_checkbox->isChecked()) {
      //--单独初始定位点处理
      dealInitPose(map_id);
    } else if (pubdialog_->alldata_checkbox->isChecked()) {
      modifyOrPubFullData(true,map_name,map_id,typeIndex);
      dealInitPose(map_id);
    }   
}
void MainWindow::dealInitPose(std::string map_id) {
    QString suppleStr = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + ProjectName_ + "/DATA/" + "keyframes_pose";
    QFile file(suppleStr);

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    QTextStream in(&file);
    QString line = in.readLine();
    qDebug() << "文件初始点： " << line;
      //获取初始定位点文本信息
    std::vector<std::string> DockInfo = dialogtools_->spaceSplit(line.toStdString());
      //转换格式
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    double timestamp;
    istringstream isa_str(line.toStdString());
    isa_str >> timestamp >> t.x() >> t.y() >> t.z() >> q.w() >> q.x() >> q.y() >> q.z();
//     ifs >> timestamp >> t.x() >> t.y() >> t.z() >> q.w() >> q.x() >> q.y() >>q.z();

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = q.matrix();
    pose(0,3) = t.x();
    pose(1,3) = t.y();
    pose(2,3) = t.z();
      //发布初始定位点->平台
    mapwidget_->post_initia_dock(map_id,pose,pubToken_);
}
void MainWindow::initPoseGet() {
    QString suppleStr = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + ProjectName_ + "/DATA/" + "keyframes_pose";
    QFile file(suppleStr);

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;

    QTextStream in(&file);
    QString line = in.readLine();
    // qDebug() << "文件初始点： " << line;
    std::vector<std::string> DockInfo = dialogtools_->spaceSplit(line.toStdString());
    //--显示
    mapwidget_->udpateShowInitData(DockInfo);
}
void MainWindow::modifyOrPubFullData(bool state, std::string map_name,
                                     std::string map_id,
                                     int typeIndex) {   
    std::cout << "pub..." << std::endl;
    //--发布区域->平台    
    switch (typeIndex) {
      case 0:
        for (int i = 0; i < Vm_T::Type_Num; i++) {
          if (mapwidget_->getVectorType((Vm_T)i).empty()) {
            continue;
          }
          if (i == (int)Vm_T::ClearArea)
          {
            //生成json文件
            std::string uploadArea = mapwidget_->saveCsvJsonFiles(map_name,i,map_id);
            //上传
            if (state) {
              std::cout << "新建--cleareare" << std::endl;
              mapwidget_->postUploadAllData(pubToken_,uploadArea);
            } else {
              std::cout << "修改--cleareare" << std::endl;
              mapwidget_->odinaryAllData(pubToken_,uploadArea,map_id);
            }
            break;
          }
        }
        break;
      case 1:
        for (int i = 0; i < Vm_T::Type_Num; i++) {
          if (mapwidget_->getVectorType((Vm_T)i).empty()) {
            continue;
          }
          if (i == (int)Vm_T::RoadEdge)
          {
            //生成json文件
            std::string uploadArea = mapwidget_->saveCsvJsonFiles(map_name,i,map_id);
            //上传
            if (state) {
              std::cout << "新建--roadedge" << std::endl;
              mapwidget_->postUploadAllData(pubToken_,uploadArea);
            } else {
              std::cout << "修改--roadedge" << std::endl;
              mapwidget_->odinaryAllData(pubToken_,uploadArea,map_id);
            }
            break;
          }
        }
        break;
      case 2:
        for (int i = 0; i < Vm_T::Type_Num; i++) {
          if (mapwidget_->getVectorType((Vm_T)i).empty()) {
            continue;
          }
          //生成json文件
          std::string uploadArea = mapwidget_->saveCsvJsonFiles(map_name,i,map_id);
          //上传
          if (state) {
            std::cout << "新建--all" << std::endl;
            mapwidget_->postUploadAllData(pubToken_,uploadArea);
          } else {
            std::cout << "修改--all" << std::endl;
            mapwidget_->odinaryAllData(pubToken_,uploadArea,map_id);
          }
        }
        break;
      default:
        break;
    }
}
void MainWindow::deleteAllData(std::string map_id, int typeIndex) {
  std::string dataType;
  std::string dataType2;
  switch (typeIndex) {
    case 0:
      dataType = "CLEAN";
      mapwidget_->deleteAllData(pubToken_,dataType,map_id);
      break;
    case 1:
      dataType = "DRIVE";
      mapwidget_->deleteAllData(pubToken_,dataType,map_id);
      break;
    case 2:
      dataType = "CLEAN";
      dataType2 = "DRIVE";
      mapwidget_->deleteAllData(pubToken_,dataType,map_id);
      mapwidget_->deleteAllData(pubToken_,dataType2,map_id);
      break;
    default:
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// gid
void MainWindow::showGidDIalog(int type_id) {
  //初始化numb
  for (size_t i = 0; i < listGidInfos_.size(); i++)
  {
    QMap<QString, QList<QString>> origin = listGidInfos_.at(i);
    QMap<QString, QList<QString>>::iterator it = origin.begin();
    gidnum_.push_back(atoi(it.key().toStdString().c_str()));
  }

  //不允许重复编号
  int gid_numb = setgiddialog_->getCurrentGidNumb();
  if (gidnum_.size() != 0)
  {
    for (auto numb : gidnum_)
    {
      if (gid_numb == numb)
      {
        gid_numb = gid_numb+1;
        setgiddialog_->gidnumb_spinbox->setValue(gid_numb);
      }
    }
  }
  //根据类型设置可填写参数
  if (type_id == Vm_T::Signal)
  {
    setgiddialog_->setEditEnabled(false,true);
  }
  if (type_id == Vm_T::Gate || type_id == Vm_T::Elevator)
  {
    setgiddialog_->setEditEnabled(true,false);
  }
  setgiddialog_->show();
}
void MainWindow::getGidInfo() {
  //获取当前地图的gid数据
  QList<QString> infos;
  QMap<QString,QList<QString>> signalinfos;
  
  int type_id = getCurrentVmapTypeID();
  int gid_numb = setgiddialog_->getCurrentGidNumb();
  gidnum_.push_back(gid_numb);
  if (type_id == Vm_T::Signal)
  {
    //设置默认链接自己
    linktype_comboBox->setCurrentIndex(Vm_T::Signal);
    QString signalvalue = setgiddialog_->enteredGidSignal();
    QString explaininfo = ProjectName_ + "-" + "Signal-id";
    // infos.push_back(QString::number(gid_numb));
    infos.push_back(signalvalue);
    infos.push_back(explaininfo);

    signalinfos.insert(QString::number(gid_numb),infos);
  }
  if (type_id == Vm_T::Gate || type_id == Vm_T::Elevator)
  {
    QString explaininfo;
    // infos.push_back(QString::number(gid_numb));
    infos.push_back(setgiddialog_->enteredGidWebId());
    infos.push_back(setgiddialog_->enteredGidLoar());
    if (type_id == Vm_T::Gate)
    {
      linktype_comboBox->setCurrentIndex(Vm_T::Gate);
      explaininfo = ProjectName_ + "-" + "Gate-id";
    }
    else if (type_id == Vm_T::Elevator) {
      linktype_comboBox->setCurrentIndex(Vm_T::Elevator);
      explaininfo = ProjectName_ + "-" + "Elevator-id";
    }
    infos.push_back(explaininfo);
    signalinfos.insert(QString::number(gid_numb),infos);
  }
  //设置默认关联gid数据
  linktypenum_edit->setText(QString::number(gid_numb));
  listGidInfos_.push_back(signalinfos);
}

//////////////////////////////////////////////////////////////////////////////////////////
//--边界线显示
void MainWindow::showPurgeArrow() {
  double dire;
  std::map<int, VectorMap_>::const_iterator pugrearrow;
  int i = 0;
  for (pugrearrow = purge_arrows_.begin(); pugrearrow != purge_arrows_.end(); pugrearrow++)
  {
    // double quate_w = pugrearrow->second.wp[0].point.quate_w;
    // dire = mapwidget_->angleToOuaterGetDire(quate_w);
    std::vector<Way_Point_> wp = pugrearrow->second.wp;
    mapwidget_->getAndShowDeviationPose(wp,1,pugrearrow->first);
    i = i++;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
//--选点偏移
/*多选模式的操作：对选中的vmapt地图元素id进行保存->偏移
  --选取起始点以及终点
  --中间点全部高亮储存
*/
void MainWindow::getChooseVmapPurgeIds(int type_id, Way_Point_ pose) {
  if (pugre_vmap_ids_.size() == 2) {
    // pugre_vmap_ids_.clear();
    // choosevmaps_pBtn->setEnabled(true);
    return;
  }
  switch (type_id) {
    case (int)Vm_T::ClearArea: {
      bool isGet = choiceVmapNearPoint(type_id,pose);
      if (isGet) {
        pugre_vmap_ids_.push_back(current_point_index_);
      }   
    } break;
    case (int)Vm_T::Lane: {
      bool isGet = choiceVmapNearPoint(type_id,pose);
      if (isGet) {
        pugre_vmap_ids_.push_back(current_point_index_);
      }   
    } break;
    case (int)Vm_T::RoadEdge: {
      bool isGet = choiceVmapNearPoint(type_id,pose);
      if (isGet) {
        pugre_vmap_ids_.push_back(current_point_index_);
      }   
    } break;
    default:
      break;
  }
  if (pugre_vmap_ids_.size() == 2 &&
      pugre_vmap_ids_.front() == pugre_vmap_ids_.back())
    pugre_vmap_ids_.pop_back();
  std::cout << "选中偏移的点：" << pugre_vmap_ids_.size() << std::endl;
}