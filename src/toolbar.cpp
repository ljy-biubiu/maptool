#include "toolbar.h"

#include <QMainWindow>
#include <QMenu>
#include <QPainter>
#include <QPainterPath>
#include <QSpinBox>
#include <QLabel>
#include <QToolTip>
#include <QToolButton>
#include <QActionGroup>

#include <stdlib.h>

static QPixmap genIcon(const QSize &iconSize, const QString &, const QColor &color)
{
    int w = iconSize.width();
    int h = iconSize.height();

    QImage image(w, h, QImage::Format_ARGB32_Premultiplied);
    image.fill(0);

    QPainter p(&image);

    extern void render_qt_text(QPainter *, int, int, const QColor &);
    render_qt_text(&p, w, h, color);

    return QPixmap::fromImage(image, Qt::DiffuseDither | Qt::DiffuseAlphaDither);
}

static QPixmap genIcon(const QSize &iconSize, int number, const QColor &color)
{ 
    return genIcon(iconSize, QString::number(number), color); 
}

ToolBar::ToolBar(const QString &title, MainWindow *mainwindow_, QWidget *parent)
    : QToolBar(parent)
    , mainwindow(mainwindow_)
{
    setWindowTitle(title);
    setObjectName(title);

    setIconSize(QSize(22, 22));


    filebutton = creatQToolButtonMenu();
    filebutton->setText(tr("点云"));
    filebutton->setIcon(QIcon(":/res/images/pcd_file.png"));
    connect(filebutton, SIGNAL(clicked()), mainwindow_, SLOT(loadPcds_Action()));

    mapbutton = creatQToolButtonMenu();
    mapbutton->setText(tr("地图"));
    mapbutton->setIcon(QIcon(":/res/images/files.png"));
    connect(mapbutton, SIGNAL(clicked()), mainwindow_, SLOT(readAllMap_Action()));

    slamfilebutton = creatQToolButtonMenu();
    slamfilebutton->setText(tr("文件"));
    slamfilebutton->setIcon(QIcon(":/res/images/file_frame.png"));
    connect(slamfilebutton, SIGNAL(clicked()), mainwindow_, SLOT(readSlamfile_act_triggered()));

    databutton = creatQToolButtonMenu();
    databutton->setText(tr("数据"));
    databutton->setIcon(QIcon(":/res/images/slamdata.png"));
    connect(databutton, SIGNAL(clicked()), mainwindow_, SLOT(readSlamData_act_triggered()));

    savebutton = creatQToolButtonMenu();
    savebutton->setText(tr("保存"));
    savebutton->setIcon(QIcon(":/res/images/save.png"));
    connect(savebutton, SIGNAL(clicked()), mainwindow_, SLOT(saveAllMap_Action()));

    exportbutton = creatQToolButtonMenu();
    exportbutton->setText(tr("导出"));
    exportbutton->setIcon(QIcon(":/res/images/export_pcd.png"));
    connect(exportbutton, SIGNAL(clicked()), mainwindow_, SLOT(exportCloudPcd_act_triggered()));

    modebutton = creatQToolButtonMenu();
    modebutton->setText(tr("模式"));
    modebutton->setIcon(QIcon(":/res/images/blocks_pcd.png"));
    modebutton->setMenu(mainwindow_->modemenu);

    isviewbutton = creatQToolButtonMenu();
    isviewbutton->setText(tr("预览"));
    isviewbutton->setIcon(QIcon(":/res/images/previewclose.png"));
    isviewbutton->setMenu(mainwindow_->isviewmenu);

    colorbutton = creatQToolButtonMenu();
    colorbutton->setText(tr("强度"));
    colorbutton->setIcon(QIcon(":/res/images/platte.png"));
    colorbutton->setMenu(mainwindow_->colorViewmenu);

    sizebutton = creatQToolButtonMenu();
    sizebutton->setText(tr("大小"));
    sizebutton->setIcon(QIcon(":/res/images/pointsize.png"));
    sizebutton->setMenu(mainwindow_->sizesetmenu);

    mapchangebutton = creatQToolButtonMenu();
    mapchangebutton->setText(tr("切换"));
    mapchangebutton->setIcon(QIcon(":/res/images/changemap.png"));
    mapchangebutton->setMenu(mainwindow_->mapchangemenu);

    gridbutton = creatQToolButtonMenu();
    gridbutton->setText(tr("网格"));
    gridbutton->setIcon(QIcon(":/res/images/gridsize.png"));
    connect(gridbutton, SIGNAL(clicked()), mainwindow_, SLOT(gridParam_Action()));

    view2d3dbutton = creatQToolButtonMenu();
    view2d3dbutton->setText(tr("2D/3D"));
    view2d3dbutton->setIcon(QIcon(":/res/images/2d3dview.png"));
    view2d3dbutton->setMenu(mainwindow_->view2d3dmenu);

    clearbutton = creatQToolButtonMenu();
    clearbutton->setText(tr("清除"));
    clearbutton->setIcon(QIcon(":/res/images/erase.png"));
    clearbutton->setMenu(mainwindow_->clearmenu);

    zerobutton = creatQToolButtonMenu();
    zerobutton->setText(tr("原点"));
    zerobutton->setIcon(QIcon(":/res/images/aimingzero.png"));
    connect(zerobutton, SIGNAL(clicked()), mainwindow_, SLOT(refreshZero_Action()));

    imagebutton = creatQToolButtonMenu();
    imagebutton->setText(tr("图像"));
    imagebutton->setIcon(QIcon(":/res/images/imageview.png"));
    connect(imagebutton, SIGNAL(clicked()), mainwindow_, SLOT(image_view_Action()));

    // updatebutton = creatQToolButtonMenu();
    // updatebutton->setText(tr("更新"));
    // updatebutton->setIcon(QIcon(":/res/images/update.png"));
    // connect(updatebutton, SIGNAL(clicked()), mainwindow_, SLOT(updatevmap_tBtn_Clicked()));

    uploadbutton = creatQToolButtonMenu();
    uploadbutton->setText(tr("上传"));
    uploadbutton->setIcon(QIcon(":/res/images/upload.png"));
    connect(uploadbutton, SIGNAL(clicked()), mainwindow_, SLOT(upLoad_Action()));

    testmapbutton = creatQToolButtonMenu();
    testmapbutton->setText(tr("验证"));
    testmapbutton->setIcon(QIcon(":/res/images/testMap.png"));
    connect(testmapbutton, SIGNAL(clicked()), mainwindow_, SLOT(testMap_Action()));

    docksviewbutton = creatQToolButtonMenu();
    docksviewbutton->setText(tr("调试"));
    docksviewbutton->setIcon(QIcon(":/res/images/truck.png"));
    connect(docksviewbutton, SIGNAL(clicked()), mainwindow_, SLOT(docksview_Act_triggered()));

    versionbutton = creatQToolButtonMenu();
    versionbutton->setText(tr("版本"));
    versionbutton->setIcon(QIcon(":/res/images/updateVersion.png"));
    versionbutton->setMenu(mainwindow_->versionmenu);

    cloudLinkbutton = creatQToolButtonMenu();
    cloudLinkbutton->setText(tr("平台"));
    cloudLinkbutton->setIcon(QIcon(":/res/images/cloudlink.png"));
    cloudLinkbutton->setMenu(mainwindow_->platformmenu);

    addWidget(mainwindow_->changeMainWindow);
    addSeparator();
    addWidget(filebutton);
    addWidget(mapbutton);
    addWidget(slamfilebutton);
    addWidget(savebutton);
    addSeparator();
    addWidget(databutton);
    addWidget(exportbutton);
    addSeparator();
    addWidget(modebutton);
    addWidget(isviewbutton);
    addWidget(clearbutton);
    addSeparator();
    addWidget(mapchangebutton);
    addWidget(colorbutton);
    addWidget(view2d3dbutton);
    addWidget(sizebutton);
    addWidget(mainwindow_->refreshPcdAct);
    addWidget(gridbutton);
    addWidget(zerobutton);
    addSeparator();
    addWidget(mainwindow_->undoact);
    // addWidget(mainwindow_->delete_pBtn);
    addSeparator();
    addWidget(imagebutton);
    addWidget(mainwindow_->updatevmap_tBtn);
    addWidget(uploadbutton);
    addSeparator();
    addWidget(docksviewbutton);
    addWidget(testmapbutton);
    addSeparator();
    addWidget(mainwindow_->postureEcho_tBtn);
    addWidget(mainwindow_->rulerDis_tBtn);
    addSeparator();
    addWidget(cloudLinkbutton);
    addWidget(mainwindow_->userLogin_tBtn);
    addSeparator();
    addWidget(versionbutton);
    // addAction()
}

QToolButton *ToolBar::creatQToolButtonMenu() {
    QToolButton *toolButton = new QToolButton(this);
    toolButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    toolButton->setPopupMode(QToolButton::InstantPopup);
    toolButton->setStyleSheet(QString("QToolButton::menu-indicator{image:none;}"
                                      "QToolButton{border-style: none;}"
                                      "QToolButton{padding:2px;}"
                                      "QToolButton{font:12px;}"));
    toolButton->setIconSize(QSize(20, 20));
    toolButton->setCursor(QCursor(Qt::PointingHandCursor));
    toolButton->setFixedHeight(45);
    toolButton->setFixedWidth(45);
    return toolButton;
}
