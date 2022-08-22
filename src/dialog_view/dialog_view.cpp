#include "dialog_view.h"

#include <stdlib.h>

//创建网格设置窗口
DialogWidgetTools::DialogWidgetTools() {}
DialogWidgetTools::~DialogWidgetTools() {}

/*设置action组*/
void DialogWidgetTools::setGroupAction(QAction *action, QMenu *menu, 
                                       QActionGroup *group, QList<QAction *> list) {
    menu->addAction(action);
    action->setCheckable(true);
    group->addAction(action);
    list.append(action);
}
/*设置push button样式*/
void DialogWidgetTools::createPushButtonSize(QPushButton *pushbutton, const QString &info, QList<QPushButton *> list) {
    // QPushButton *pushbutton = new QPushButton();
    pushbutton->setText(info);
    pushbutton->setFixedSize(80,25);
    pushbutton->setStyleSheet("font-size:13px;border：1px solid black; background：white");
    list.append(pushbutton);
    // return pushbutton;
}
/*设置tool button样式*/
void DialogWidgetTools::creatQToolButton(QToolButton *toolButton, const QString &icon, const QString &info, bool status) {
    // QToolButton *toolButton = new QToolButton();
    toolButton->setText(info);
    toolButton->setIcon(QIcon(icon));
    toolButton->setCheckable(status);
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
    // return toolButton;
}
/*设置action添加进menu中*/
void DialogWidgetTools::setupMenuAddAction(QAction *action, const QString &icon, 
                                               QMenu *menu, QList<QAction *> list) {
    // QAction *action = new QAction(QIcon(icon), info, mainwindow_);
    action->setIcon(QIcon(icon));
    menu->addAction(action);
    list.append(action);
    // return action;
}
void DialogWidgetTools::setupMenuAddAction(QAction *action, QMenu *menu, QList<QAction *> list) {
    // QAction *action = new QAction(info, mainwindow_);
    menu->addAction(action);
    list.append(action);
    // return action;
}
void DialogWidgetTools::setupMenuAddAction(QAction *action, QMenu *menu, 
                                           QActionGroup *actiongroup, QList<QAction *> list) {
    // QAction *action = new QAction(info, mainwindow_);
    actiongroup->setExclusive(true);
    menu->addAction(action);
    actiongroup->addAction(action);
    list.append(action);
    // return action;
}
/*设置控件组hide或show*/
bool DialogWidgetTools::setMainWidgetView(QList<QSplitter *> extraSplitters, QSplitter *splitter, bool status,bool state) {
  for (size_t i = 0; i < extraSplitters.size(); i++)
  {
    QSplitter *listQsplitter = (QSplitter *)extraSplitters.at(i);
    listQsplitter->setVisible(status);
  }
  splitter->setVisible(state);
  return true;
}
/*设置状态栏label样式*/
void DialogWidgetTools::setStatusBarLabelView(int size, QLabel *label) {
  // QLabel *label = new QLabel(info);
  label->setStyleSheet("border:1px black;");
  label->setFixedSize(size, 20);
  label->setAlignment(Qt::AlignCenter);
  // return label;
}

/*切割字符串*/
void DialogWidgetTools::SplitString(const std::string& s, std::vector<std::string>& v, const std::string &c){
  int length = s.length();
  if (s.size() > 0)
  {
    std::string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while(std::string::npos != pos2){
      v.push_back(s.substr(pos1,pos2 - pos1));
      pos1 = pos2 + c.size();
      pos2 = s.find(c,pos1);
    }
    if(pos1 != s.length()){
      v.push_back(s.substr(pos1));
    }
  }
}
/*工程文件路径处理*/
QStringList DialogWidgetTools::projectStringDeal(const std::string& home_str,QString &project) {
    QString vmappath_ = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + project + "/VMAP/" + project;
    QStringList string_list;
    string_list.clear();
    QDir vmapdir(vmappath_);
    if (!vmapdir.exists())
        return string_list;

    QStringList filters;
    filters << QString("*.csv");
    filters << QString("*.osm");
    vmapdir.setFilter(QDir::Files | QDir::NoSymLinks); //设置类型过滤器，只为文件格式
    vmapdir.setNameFilters(filters);  //设置文件名称过滤器，只为filters格式
    int dir_count = vmapdir.count();
    if(dir_count <= 0)
        return string_list;

    for(int i=0; i<dir_count; i++)
    {
        QString vmap_path = vmappath_ + "/" + vmapdir[i];//文件路径
        string_list.append(vmap_path);
    }
    return string_list;
}

/*删除layout*/
void DialogWidgetTools::deleteItem(QLayout *layout){
    QLayoutItem *child;
    while ((child = layout->takeAt(0)) != nullptr)
    {
        //setParent为NULL，防止删除之后界面不消失
        if(child->widget())
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

/*输入：指向下拉窗的指针，下拉列表字符串，下拉列表的个数，是否显示的开关*/
void DialogWidgetTools::updateComboBox(QComboBox *cb, const std::string *names, 
                                       int num, bool isShow)
{
  if (!cb) {
    return;
  }
  cb->blockSignals(true);
  cb->clear();
  if (isShow) {
    cb->show();
  } else {
    cb->hide();
  }
  for (int i = 0; i < num; i++) {
    cb->addItem(names[i].c_str());
  }
  cb->blockSignals(false);
}
/*输入：指向label的指针，是否显示的开关*/
void DialogWidgetTools::updateQLabel(QLabel *cb, bool isShow) {
  if (!cb) {
    return;
  }
  if (isShow) {
    cb->show();
  } else {
    cb->hide();
  }
}
/*输入：指向Edit的指针，是否隐藏的开关*/
void DialogWidgetTools::updateQLineEdit(QLineEdit *cb, bool isShow) {
  if (!cb) {
    return;
  }
  cb->blockSignals(true);
  cb->clear();
  if (isShow) {
    cb->show();
  } else {
    cb->hide();
  }
  cb->blockSignals(false);
}
void DialogWidgetTools::updateQPushButton(QPushButton *cb, bool isShow) {
  if (!cb) {
    return;
  }
  if (isShow) {
    cb->show();
  } else {
    cb->hide();
  }
}
void DialogWidgetTools::deleteQActionsIter(QList<QAction *> &qActions) {
  QList<QAction*>::iterator iter = qActions.begin();
  for (; iter != qActions.end(); iter++)
  {
    if((*iter) != nullptr)
    {
      std::cout << "delete!!!!" << std::endl;
      delete (*iter);
      *iter = nullptr;
    }
  }
  qActions.clear();
}
void DialogWidgetTools::deleteQPushButtonsIter(QList<QPushButton *> &qPushButtons) {
  QList<QPushButton*>::iterator iter = qPushButtons.begin();
  for (; iter != qPushButtons.end(); iter++)
  {
    if((*iter) != nullptr)
    {
      std::cout << "delete!!!!" << std::endl;
      delete (*iter);
      *iter = nullptr;
    }
  }
  qPushButtons.clear();
}

//color转换
QString DialogWidgetTools::changeColorNumberToString(Ogre::ColourValue& color) {
  QString color_string = QString::number(color.r*255) + ":" 
                       + QString::number(color.g*255) + ":" 
                       + QString::number(color.b*255);
  return color_string;
}
Ogre::ColourValue DialogWidgetTools::changeColorNumberToString(QColor color) {
  float color_rgba_r = QString::number(color.red()).toFloat() / 255;
  float color_rgba_g = QString::number(color.green()).toFloat() / 255;
  float color_rgba_b = QString::number(color.blue()).toFloat() / 255;
  Ogre::ColourValue ogre_color = Ogre::ColourValue(color_rgba_r, color_rgba_g, color_rgba_b, 0.5f);
  return ogre_color;
}

/*文件存在判断 返回整个路径信息*/
bool DialogWidgetTools::isFileExistance(QString &path, QString &suffix, QStringList &string_list) {
    //判断路径是否存在
    string_list.clear();
    QDir dir(path);
    if(!dir.exists())
        return false;
 
    //查看路径中后缀为.csv格式的文件
    QStringList filters;
    // filters<<QString("*.csv");
    filters << suffix;
    filters << QString("*.osm");
    dir.setFilter(QDir::Files | QDir::NoSymLinks); //设置类型过滤器，只为文件格式
    dir.setNameFilters(filters);  //设置文件名称过滤器，只为filters格式
 
    //统计.csv格式的文件个数
    int dir_count = dir.count();
    if(dir_count <= 0)
        return false;
 
    //  //测试
    //  //文件路径及名称
    //  QFile outFile( "filename.txt");
    //  //看能否打开
    //  if(!outFile.open(QIODevice::WriteOnly | QIODevice::Append))  
    //      return ;  
    //  QTextStream ts(&outFile);
 
    //存储文件名称
    for(int i=0; i<dir_count; i++)
    {
        QString vmap_path = path + "/" + dir[i];  //路径 + 文件名称 = 文件路径
        //ts<<file_name<<"\r\n"<<"\r\n";
        string_list.append(vmap_path);
    }
    return true;
}

/*文件存在判断 仅返回文件名*/
bool DialogWidgetTools::getFileName(QString &path, QString &suffix, QStringList &string_list) {
    //判断路径是否存在
    string_list.clear();
    QDir dir(path);
    if(!dir.exists())
        return false;
 
    //查看路径中后缀为suffix格式的文件
    QStringList filters;
    // filters<<QString("*.csv");
    filters << suffix;
    dir.setFilter(QDir::Files | QDir::NoSymLinks); //设置类型过滤器，只为文件格式
    dir.setNameFilters(filters);  //设置文件名称过滤器，只为filters格式
 
    //统计.csv格式的文件个数
    int dir_count = dir.count();
    if(dir_count <= 0)
        return false;
    //存储文件名称
    for(int i=0; i<dir_count; i++)
    {
        QString file_name = dir[i];  //文件名称
        //ts<<file_name<<"\r\n"<<"\r\n";
        string_list.append(file_name);
    }
    return true;
}

//损坏文件禁止加载，防止工具崩溃 （access(file.toStdString().c_str()，R_OK)!=-1
//判断文件是否可读
bool DialogWidgetTools::isReadOk(QString &file) {
    struct stat sb;
    stat(file.toStdString().c_str(),&sb);
    if(sb.st_mode&S_IROTH)
    {
      std::cout<<"-----R------"<<std::endl;
      return true;
    } else {
      QMessageBox::question(this, tr("提示"), tr("pcd文件不可读，请确认文件正常."),
                      QMessageBox::Yes);
      return false;
    }
}

//删除文件夹 
bool DialogWidgetTools::delDir(const QString &path) 
{ 
    if (path.isEmpty()){ 
        return false; 
    } 
    QDir dir(path); 
    if(!dir.exists()){ 
        return true; 
    } 
    dir.setFilter(QDir::AllEntries | QDir::NoDotAndDotDot); //设置过滤 
    QFileInfoList fileList = dir.entryInfoList(); // 获取所有的文件信息 
    for(auto file : fileList){ //遍历文件信息 
        if (file.isFile()){ // 是文件，删除
            qDebug() << "删除文件";
            file.dir().remove(file.fileName()); 
        }else{ // 递归删除 
            qDebug() << " 递归删除文件";
            delDir(file.absoluteFilePath()); 
        } 
    }
    qDebug() << "删除文件夹";
    return dir.rmdir(dir.absolutePath()); // 删除文件夹 
}

//以空格分隔字符串
std::vector<std::string> DialogWidgetTools::spaceSplit(std::string const &input) {
    std::istringstream buffer(input);
    std::vector<std::string> ret((std::istream_iterator<std::string>(buffer)),
                                  std::istream_iterator<std::string>());
    return ret;
}