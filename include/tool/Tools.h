#ifndef __TOOLS_H__
#define __TOOLS_H__

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <Eigen/Dense>
#include <QString>
#include <QTime>
#include <QFile>
#include <QDir>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "encryption/Encryption.h"

using std::string;
using std::vector;

class Tools {
 public:
  Tools();
  ~Tools();

 public:
  // math
  void leastSquare(std::vector<Eigen::Vector3d> &points_vec);
  void pclRansacLine(std::vector<Eigen::Vector3d> &points_vec);
  Eigen::Vector3d LookRotation(Eigen::Vector3d fromDir);
  Eigen::Quaterniond quat(Eigen::Vector3d fromDir);
  // file
  string getFileName(string file_name);
  void fileEcrept(QString &path_str);
  void fileDecrept(QString &path_str);
  void filePcdDecrept(QString &path_str,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                      Eigen::Vector4f &origin, Eigen::Quaternionf &orientation);
  bool isCsvEcreptFile(QString &path_str);
  bool isOsmEcreptFile(QString &path_str);
  bool isPcdEcreptFile(QString &path_str);
  bool isSameSuffix(std::string &path_str, std::string &suffix_str);
  bool isCsvFile(QString &path_str);
  bool isGidFile(QString &path_str);
  bool isOsmFile(QString &path_str);
  void removeFiles(bool remove_flag, QStringList &files);
  void removeFiles(bool remove_flag, QString &file);
  void removeOldFiles(QString &file);
  void removeOldFile(QString &file);
  //
  QString toQString(const string &s);
  string fromQString(const QString &qs);

 private:
  // Encryption encryption_;
};

string getFileName(string file_name);
void timeStart();
QString timeOff();
// string to QString
QString toQString(const string &s);
// QString to string
string fromQString(const QString &qs);
string joinStrVec(const vector<string> v, string splitor = " ");

#endif
