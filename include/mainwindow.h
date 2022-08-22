#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGroupBox>
#include <QComboBox>
#include <QSpinBox>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QSplitter>
#include <QToolButton>
#include <QDockWidget>
#include <QMessageBox>
#include <QFrame>
#include <QSignalMapper>
#include <QDir>
#include <QTimer>
#include <QDateTime>
#include <QColorDialog>
#include <QInputDialog>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include<sys/stat.h>

class MapWidget;
#include "map_widget/map_widget.h"
#include "manage_map.h"
class Task_map;
#include "task_map.h"
class Unoptimized_map;
#include "unoptimized_map.h"
class CTest;
#include "test/ctest.h"

class ToolBar;
#include "toolbar.h"
class TaskListUi;
#include "tasklistui.h"
class DialogWidgetTools;
#include "dialog_view.h"
class RegularInfoDialog;
#include "dialog_widget/regular_dialog.h"
class DocksVehicleViewDialog;
#include "dialog_widget/docksview_dialog.h"
class TagInfoDialog;
#include "dialog_widget/tag_dialog.h"
class GetGridValueDialog;
#include "dialog_widget/gridvalue_dialog.h"
class ExportCloudPcdDialog;
#include "dialog_widget/exportpcd_dialog.h"
class LoginDialog;
#include "dialog_widget/loginUi.h"
class CloudPubDialog;
#include "dialog_widget/cloudpub_dialog.h"
class SetGidDialog;
#include "dialog_widget/gidtype_dialog.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    typedef QMap<QString, QSize> CustomSizeHintMap;
    explicit MainWindow(const CustomSizeHintMap &customSizeHints,
                        QWidget *parent = Q_NULLPTR,
                        Qt::WindowFlags flags = 0);

Q_SIGNALS:
    void uploadMapInfoSignal(QString CurQString_);
    void deleteMysqlInfo(QString projectName);
    void modifyMysqlInfo();
    void updateMapTool();
    void createMysqlInfo();
    // void gridColorSignal(Ogre::ColourValue color);

public Q_SLOTS:
    void refrushMapSelectiveUi();
    void closeMapChooseUiAndOpenPaint(QString);
    void closeMapAndChooseDataView(QString);
    void openOptimizeToolAndUpload(QString);
    void openToolAndDealMultiple(QList<QString>);
    void pushNewestDockInfo(QString);
    //-----triggered action-----
    void loadPcds_Action();
    void readOnlyPcdFromProject(QString id,QString path);
    void loadBlockPcd_Action();
    void readAllMap_Action();
    void readSlamData_act_triggered();
    void readSlamfile_act_triggered();
    void saveAllMap_Action();
    void exportCloudPcd_act_triggered();
    void exit_Action();
    void initDisplayData();
    void saveOnlyMap_Action();
    void saveLanelet_Action();
    void aixColor_Action();
    void intensity_Action();
    void gridParam_Action();
    // void gridColor_Action();
    void mapPcdSize_Action();
    void view2D3D_Action();
    void twoDimenact_Action();
    void threeDimenact_Action();
    void topThreeDimenact_Action();
    void doPcdChange_Action(const QString &actionname);
    void pcdAll_Action();
    void refreshZero_Action();
    void displayWidget_Action();
    void imageWidget_Action();
    void upLoad_Action();
    void image_view_Action();
    void testMap_Action();
    void doProjectPcdChange_Action();
    void docksview_Act_triggered();
    void create_batch_act_Action();
    void modify_batch_act_Action();
    void delete_batch_act_Action();
    void signaldelete_batch_act_Action();

    void destroyDockWidget(QAction *action);
    void delBackpBtn();
    //-----edit-----
    // void pcdzmin_edit_Finished();
    // void pcdzmax_edit_Finished();
    void localx_edit_Finished();
    void localy_edit_Finished();
    void localz_edit_Finished();
    void localw_edit_Finished();
    void latitude_edit_Finished();
    void longitude_edit_Finished();
    void limitvelocity_edit_Finished();
    void linktypenum_edit_Finished();
    void laneletid_edit_Finished();
    void countcell_edit_Finished();
    void celllength_edit_Finished();
    void cellwidth_edit_Finished();
    void regularid_edit_Finished();
    //-----chlicked-----
    void changeMainWindowClicked();
    void refreshPcdClicked();
    void choosPointClicked();
    void creatnewlaneClicked();
    void clearAllMap_checkbox_clicked();
    void forwardClicked();
    void backwardClicked();
    void forwardInsertClicked();
    void backwarInsertdClicked();
    void deletemapClicked();
    void breaklaneClicked();
    void bindClicked();
    void gidSet_pBtn_Clicked();
    void chooseVmaps_pBtn_Clicked();
    void inspect_Act_checkbox_clicked();
    void update_Act_checkbox_clicked();
    void repairMysql_Act_clicked();
    void deleteMysql_Act_clicked();
    void createMysql_Act_clicked();
    void framePose_Act_triggered();
    // void reloadpcdClicked();
    void right_on_btn_hide_clicked();
    void left_on_btn_hide_clicked();
    void objectvisible_checkbox_clicked(bool);
    void mapvisible_checkbox_clicked(bool);
    void docksvisible_checkbox_clicked(bool);
    void initpose_checkbox_clicked(bool);
    void mapremove_checkbox_clicked();
    void docksremove_checkbox_clicked();
    void updatevmap_tBtn_Clicked();
    void postureEcho_tBtn_clicked();
    void rulerDis_tBtn_clicked();
    void userLogin_tBtn_clicked();

    void chooselanelettypes_pBtn_clicked();
    void addlanelet_pBtn_clicked();
    void jointlanelet_pBtn_clicked();
    void inverlanelet_pBtn_clicked();
    void extendLinestring_pBtn_clicked();
    void addregular_pBtn_clicked();
    void editregular_pBtn_clicked();
    void deletregular_pBtn_clicked();
    void addtags_pBtn_clicked();
    void originalpcdsave_pbtn_clicked();
    void filterpcdsave_pbtn_clicked();
    void datafilesave_pbtn_clicked();
    //--
    void regulinfo_dialog_okpBtn_clicked();
    void regulinfo_dialog_cancelpBtn_clicked();
    void taginfo_dialog_okpBtn_clicked();
    void taginfo_dialog_cancelpBtn_clicked();
    void docksview_dialog_okpBtn_clicked();
    void docksview_dialog_cancelpBtn_clicked();
    void gridsize_dialog_okpBtn_clicked();
    void gridsize_dialog_cancelpBtn_clicked();
    void exportclod_dialog_cancelpBtn_clicked();
    void exportclod_dialog_okpBtn_clicked();
    void loginui_dialog_cancelpBtn_clicked();
    void loginui_dialog_okpBtn_clicked();
    void cloudpub_dialog_cancelpBtn_clicked();
    void cloudpub_dialog_okpBtn_clicked();
    void setgid_dialog_cancelpBtn_clicked();
    void setgid_dialog_okpBtn_clicked();

    void rBtn_addNone_Toggled();
    void rBtn_addNewmap_Toggled(bool);
    void rBtn_pointCorrection_Toggled(bool);
    void createNewDockWidget();

    //-----indexchanged-----
    void maptypes_comboBox_IndexChanged();
    void maptypetype_comboBox_IndexChanged(int index);
    void waytypetype_comboBox_IndexChanged(int index);
    void bypasstype_comboBox_IndexChanged(int index);
    void traveldirectiontype_comboBox_IndexChanged(int index);
    void traveltype_comboBox_IndexChanged(int index);
    void travelslopedtype_comboBox_IndexChanged(int index);
    void roomtype_comboBox_IndexChanged(int index);
    void linktype_comboBox_IndexChanged(int index);
    void maptype_spinBox_valueChanged(int index);
    void gidnumb_spinbox_valueChanged(int index);
    void lanelet2map_mode_checkbox_clicked(bool status);
    void vmap_mode_checkbox_clicked(bool status);
    void parkName_comboBox_IndexChanged();

    void getGridValidColor();
    void onMouseEventClicked(float,float,float);
    void onrightMouseEventClicked(float,float,float,int,int);

public:
    //QAction
    QAction *pcdImport = nullptr;
    QAction *blockPcdImport = nullptr;
    QAction *allmapImport = nullptr;
    QAction *slamdataImport_Act = nullptr;
    QAction *slamfileImport_Act = nullptr;
    QAction *allmapSave = nullptr;
    QAction *exportPcd_Act = nullptr;
    QAction *exitAct = nullptr;
    QAction *onlymapSave = nullptr;
    QAction *laneletSave = nullptr;
    QAction *setview = nullptr;
    QAction *view2d3dact = nullptr;
    QAction *imageview = nullptr;
    QAction *creatAct = nullptr;
    QAction *uploadAct = nullptr;
    QAction *testMapAct = nullptr;
    QAction *refreshZeroAct = nullptr;
    QAction *pcd_all = nullptr;
    QAction *docksviewAct = nullptr;

    QToolButton *undoact;
    QToolButton *refreshPcdAct;
    QAction *clear_pBtn = nullptr;
    QPushButton *delete_pBtn;
    QToolButton *updatevmap_tBtn;
    QToolButton *postureEcho_tBtn;
    QToolButton *rulerDis_tBtn;
    QToolButton *userLogin_tBtn;
    QAction *create_batch_act = nullptr;
    QAction *modify_batch_act = nullptr;
    QAction *delete_batch_act = nullptr;
    QAction *signaldelete_batch_act = nullptr;
    QAction *rBtn_addNewmap = nullptr;
    QAction *rBtn_pointCorrection = nullptr;
    QAction *rBtn_addNone = nullptr; 
    QAction *objectvisible_checkbox = nullptr;
    QAction *mapvisible_checkbox = nullptr;
    QAction *docksvisible_checkbox = nullptr;
    QAction *initpose_checkbox = nullptr;
    QAction *mapremove_pbtn = nullptr;
    QAction *docksremove_pbtn = nullptr;
    QAction *gridparamact = nullptr;
    QToolButton *changeMainWindow;
    //button
    QPushButton *newtype_pBtn;
    QPushButton *choosepoint_pBtn;
    QPushButton *breakLine_pBtn;
    QPushButton *forward_pBtn;
    QPushButton *backward_pBtn;
    QPushButton *forwardInsert_pBtn;
    QPushButton *backwardInsert_pBtn;
    QPushButton *bind_pBtn;
    QPushButton *gidSet_pBtn;
    QPushButton *choosevmaps_pBtn;
    // QPushButton *reload_pBtn;
    QToolButton *pButton;
    QToolButton *l_toolButton;
    QToolButton *r_toolButton;
    //menu
    // QMenu *paintertool;
    QMenu *colorViewmenu;
    QMenu *sizesetmenu;
    QMenu *mapchangemenu;
    // QMenu *gridViewmenu;
    QMenu *view2d3dmenu;
    QMenu *modemenu;
    QMenu *isviewmenu;
    QMenu *clearmenu;
    QMenu *versionmenu;
    QMenu *platformmenu;
    QActionGroup *changegroup;
    //QLabel QLineedit
    QLabel *vmapnum_label;
    QLabel *lanelet2id_label;
    QLabel *status_label;
    QLabel *fps_label;
    QLabel *view_label;
    QLabel *camera_base_label;
    QLabel *pcd_z_label;
    QLabel *autosave_label;
    QLineEdit *localx_edit;
    QLineEdit *localy_edit;
    QLineEdit *localz_edit;
    QLineEdit *localw_edit;
    // QLineEdit *pcdzmin_edit;
    // QLineEdit *pcdzmax_edit;
    QLineEdit *latitude_edit;
    QLineEdit *longitude_edit;
    QLineEdit *limitvelocity_edit;
    QLineEdit *linktypenum_edit;
    QLineEdit *laneletid_edit;
    QLineEdit *count_edit;
    QLineEdit *length_edit;
    QLineEdit *width_edit;
    QLineEdit *regularid_edit;
    // Qconbobox
    QComboBox * maptypes_comboBox  = nullptr;
    QComboBox * maptypetype_comboBox = nullptr;
    QComboBox * waytypetype_comboBox = nullptr;
    QComboBox * bypasstype_comboBox = nullptr;
    QComboBox * traveldirectiontype_comboBox = nullptr;
    QComboBox * traveltype_comboBox = nullptr;
    QComboBox * travelslopedtype_comboBox = nullptr;
    QComboBox * roomtype_comboBox = nullptr;
    QComboBox * linktype_comboBox = nullptr;
    QSpinBox * maptype_spinBox = nullptr;
    //qcheckbox
    QCheckBox *vmap_mode_checkbox;
    QCheckBox *lanelet2map_mode_checkbox;

protected:
    void closeEvent(QCloseEvent *event);

private:
//初始化
    void initPoseGet();
    void initMainView();
    void InitcreatModeGroup();
    void initStatusBar();
    void initVmapProperty(int type_id, Way_Point_ &wp);
    void setupToolBar();
    void setupMenuBar();
    void setupDockWidgets();
    void setupotherWidgets();
    void setupRightInfoWidget();
    void getFileNameIDList(const QStringList& fileNames);
    void getChooseLanelet2Ids(int type_id, Way_Point_ pose);
    void getLanelet2mapFunQWidgetStatus();
    void setMainWidgetView(bool status,bool state);
    void setVmapProQWidgetStatus(bool editStatus);
    void setQWidgetStatus(bool editStatus, bool editStatus1);
    void setQWidgetRegularStatus();
    int setCurrentRegularComboxIndex(std::string value_str, int num);
    void createCurrentWayPoint(int type_id, Way_Point_ &wp);
    void createCurrentLanelet2Type(int type_id, lanelet::Point3d &point3d);
    TaskListUi *creattasklistinfo();
    QWidget *creatWorkDesk();
    QWidget *creatLaneletWorkDesk();
    QFrame *creatimageInfo();
//获取type id
    int getCurrentVmapTypeNumID();
    int getCurrentVmapTypeID();
//更新
    void updateFromClickPoint(Way_Point_ pose);
    void updateQWidgetEnableStatus();
    void updateDisplayMapElement(void);
    void updateVmapcomboBoxInfo();
    void updateCoordinatesProDisplay();
    void updateCoordinatesTagDisplay();
    void udpateMenuActEnabled(bool status);
    void updateStatusBar();
    bool updateCurrentTypeTags();
    void updateGridParamEdit();
    std::string updateCurrentTypeRegular();
    std::string getNextLaneletId();
    bool isLimitVmapWayPoint(const int limit_type, const int limit_num);
    bool choiceVmapNearPoint(int type_id, Way_Point_ wp);
    void choosePullMapDocksView();
    void addregularinfo();
    void autoTimeSaveProjectData();
    void autoSaveAllProjectData();
    void gridParamSet();
    void setParamGrid();
    void getVehicleViewInfo();
    void getAllProjectName();
    void updateVehicleData(QString mapname);
    void getSlamDataPoses(QString &folderPath);
    void createNewTrajectory(std::vector<Way_Point_> &poses);
    void getAccessToken();
    QString getCurrentParkText();
    int getCurrentParkTypeId();
    void parkNameIdItemShow();
    QString chooseUpdateVehicleView();
    QString chooseProjectName();
    QString chooseProjectNameIndex(int index);
//PushButton Clicked
    void tags_edit_editingFinished();
//
    void readSlamFile(QString &fileName);
    void savePointCloudPcdFile(QString &project_name,
                            std::deque<KeyFrame> &key_frames,
                            boost::optional<Eigen::Vector3d> &map_zero_gnss,
                            bool downsample_flag,
                            double downsample_resolution, 
                            bool saveCompressedPCD,
                            std::promise<bool> &promiseObj);
    void pubInitPoseAndAreaInfo(std::string map_name, std::string map_id,
                                int typeIndex);
    void dealInitPose(std::string map_id);
    void modifyOrPubFullData(bool state, std::string map_name,
                             std::string map_id,
                             int typeIndex);
    void deleteAllData(std::string map_id, int typeIndex);
//--set gid
    void showGidDIalog(int);
    void getGidInfo();
//--调用优化工具
    void runOptimizeTool(QString &unopname, std::string supplyInfo, QList<QString> &listStr);
    void inputProjectToOptimize(QList<QString> &listStr);
//--
    void showPurgeArrow();
    void getChooseVmapPurgeIds(int type_id, Way_Point_ pose);

private:
    QString filePath_pcd_;
    QString filePath_vmap_;
    QString filePath_frame_;
    QString CurQString_;
    QString ProjectName_;
    QString project_vmap_;
    QString mapname_;
    QString map_name;

    std::vector<int> lanelet2_ids_;
    std::vector<int> pugre_vmap_ids_;
    std::vector<lanelet::Point3d> point3d_vec_;
    std::vector<std::string> pcd_paths_;
    std::vector<std::string> map_paths_;
    std::deque<KeyFrame> key_frames_;
    std::vector<Way_Point_> keyframes_poses_;
    std::map<int, VectorMap_> purge_arrows_;
    std::string pubToken_;
    std::vector<int> gidnum_;

    QList<ToolBar*> toolBars;
    QList<QAction*> qActions;
    QList<QPair<QString,QString>> pcdIdPaths;
    QList<QDockWidget *> extraDockWidgets;
    QList<QSplitter *> extraSplitters;
    QList<QAction*> actionLists;
    QList<QPair<QAction *,QDockWidget *>> fixDockWidgets;
    QList<QPushButton *> listVmapButtons;
    QList<QWidget *> listWidgets;
    QList<QString> allProjectNames;
    QList<QMap<QString,QString>> listParksIdName_;
    QList<QMap<QString,QList<QString>>> listGidInfos_;
    std::unordered_map<std::string, std::set<std::string>> robots_info_;
    //
    TaskListUi *tasklist_;
    Way_Point_ wp_point_;
    MapWidget *mapwidget_;
    ChooseMapUi *chooseMapUi_;
    Choose_map *choose_map_;
    Task_map *task_map_;
    Unoptimized_map *unoptimize_map_;
    DialogWidgetTools *dialogtools_;
    ToolBar *toolbar_;
    CTest *test_;
    DocksVehicleViewDialog *dockviewdialog_ = nullptr;
    RegularInfoDialog *regulardialog_ = nullptr;
    TagInfoDialog *taginfodialog_ = nullptr;
    GetGridValueDialog *griddialog_ = nullptr;
    ExportCloudPcdDialog *exportdialog_ = nullptr;
    LoginDialog *logindialog_ = nullptr;
    CloudPubDialog *pubdialog_ = nullptr;
    SetGidDialog *setgiddialog_ = nullptr;
    
    QSignalMapper *signalMapper;
    QSplitter *splitterMain = nullptr;
    QSplitter *splitterLeft = nullptr;
    QSplitter *splitterRight = nullptr;
    QSplitter *splitterCenter = nullptr;
    QSplitter *splitteChoice = nullptr;
    QWidget *rightWidget;
    QWidget *leftWidget;
    QWidget *dockInfoWidget;
    QVBoxLayout *verticalLayoutMain;
    QHBoxLayout *horizontalLayout;
    QHBoxLayout *choicehoriLayout;
    QMenu *filemenu;
    QMenu *dockWidgetMenu;
    QMenu *mainWindowMenu;
    QMenu *workMenu;
    QMenu *destroyDockWidgetMenu;
    QMenu *otherWidgetMenu;
    QMenu *toolBarMenu;
    QAction *topThreeDimenact;
    QAction *threeDimenact;
    QAction *twoDimenact;
    QAction *disWidgetact;
    QAction *imaWidgetact;
    QAction *aixcoloract;
    QAction *intensityact;
    QAction *projectPcd;
    QAction *update_Act;
    QAction *inspect_Act;
    QLabel *dockID_label;
    QLabel *dockname_label;
    QLabel *docktype_label;
    QLabel *docklanlon_label;
    QLabel *dockpose_label;
    QPushButton *chooselanelettypes_pBtn;
    QPushButton *addlanelet_pBtn;
    QPushButton *jointlanelet_pBtn;
    QPushButton *inverlanelet_pBtn;
    QPushButton *extendLinestring_pBtn;
    QPushButton *addregular_pBtn;
    QPushButton *addtags_pBtn;
    QPushButton *deletregular_pBtn;
    QPushButton *editregular_pBtn;
    QDockWidget *mapToolWidget;
    QWidget *modeWidget;
    QWidget *workWidget;
    QGridLayout *modeLayout;
    QDockWidget *imageViewWidget;
    QTimer autosave_timer_;
    QTimer update_timer_;
    QTimer coutdown_timer_;

    int mode_;
    int type_id_;
    int type_num_id_;
    int current_point_index_;
    int current_point3d_id_;
    bool viewEnable_ = false;
    bool choose_ = false;
    bool modify = false;
    bool r_isHide = false;
    bool l_isHide = false;
    bool isTask_ = false;
    bool isDownLoad_ = false;
    double global_points_zmin_, global_points_zmax_;
    double local_points_zmin_, local_points_zmax_;
    double lidar_height_;
    boost::optional<Eigen::Vector3d> map_zero_gnss_;
};

#endif // MAINWINDOW_H
