#include "tool/Tools.h"

QTime myTime;

Tools::Tools() {}

Tools::~Tools() {}
// math
//把已有的点集，拟合在一条线上，得到拟合后的点集
void Tools::leastSquare(std::vector<Eigen::Vector3d> &points_vec) {
  if (points_vec.size() == 0) return;
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  for (auto p : points_vec) {
    x_vec.push_back(p[0]);
    y_vec.push_back(p[1]);
  }
  //
  double x_sum, y_sum, x_squaresum, xy_sum;
  int size = x_vec.size();
  //#pragma omp parallel for
  for (int i = 0; i < size; i++) {
    x_sum += x_vec[i];
    y_sum += y_vec[i];
    x_squaresum += x_vec[i] * x_vec[i];
    xy_sum += x_vec[i] * y_vec[i];
  }
  //
  double k, b;
  if (0 == (size * x_squaresum - (x_sum * x_sum))) {
    return;
  } else {
    k = (size * xy_sum - x_sum * y_sum) /
        (size * x_squaresum - (x_sum * x_sum));
    b = (y_sum - k * x_sum) / size;
    // re point
    for (auto &p : points_vec) {
      p[1] = p[0] * k + b;
    }
  }
}
Eigen::Vector3d Tools::LookRotation(Eigen::Vector3d fromDir) {
  Eigen::Vector3d eulerAngles;

  // AngleX = arc cos(sqrt((x^2 + z^2)/(x^2+y^2+z^2)))
  eulerAngles.x() =
      acos(sqrt((fromDir.x() * fromDir.x() + fromDir.z() * fromDir.z()) /
                (fromDir.x() * fromDir.x() + fromDir.y() * fromDir.y() +
                 fromDir.z() * fromDir.z()))) *
      180.0f / M_PI;
  if (fromDir.y() > 0) eulerAngles.x() = 360 - eulerAngles.x();

  // AngleY = arc tan(x/z)
  eulerAngles.y() = atan(fromDir.x() / fromDir.z()) * 180.0f / M_PI;
  if (eulerAngles.y() < 0) eulerAngles.y() += 180;
  if (fromDir.x() < 0) eulerAngles.y() += 180;
  // AngleZ = 0
  eulerAngles.z() = 0;
  std::cout << "euler angle:" << eulerAngles.x() << " ," << eulerAngles.y()
            << "," << eulerAngles.z() << std::endl;
  return eulerAngles;
}
Eigen::Quaterniond Tools::quat(Eigen::Vector3d fromDir) {
  Eigen::Vector3d vectorBefore(1, 0, 0);
  Eigen::Quaterniond quat =
      Eigen::Quaterniond::FromTwoVectors(vectorBefore, fromDir);
  std::cout << "...quat:" << quat.x() << " ," << quat.y() << "," << quat.z()
            << "," << quat.w() << std::endl;
  return quat;
}
// point cloud　随机采用一致性－线模型，得到在一条线上的点云
void Tools::pclRansacLine(std::vector<Eigen::Vector3d> &points_vec) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr inrange_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  int size = points_vec.size();
  for (auto p : points_vec) {
    pcl::PointXYZ point;
    point.x = p[0];
    point.y = p[1];
    point.z = p[2];
    inrange_cloud->points.push_back(point);
  }
  std::cout << "inrange_cloud.size()  =" << inrange_cloud->size() << std::endl;

  std::vector<int> inliers;
  pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr lineModel(
      new pcl::SampleConsensusModelLine<pcl::PointXYZ>(inrange_cloud));

  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(lineModel);
  ransac.setDistanceThreshold(0.02);
  ransac.setMaxIterations(100);
  ransac.computeModel();
  ransac.getInliers(inliers);
  std::cout << "inliers.size()  =" << inliers.size() << std::endl;

  pcl::copyPointCloud<pcl::PointXYZ>(*inrange_cloud, inliers, *line_cloud);
  Eigen::VectorXf coeff;
  ransac.getModelCoefficients(coeff);
  std::cout << "coeff =" << coeff << std::endl;
  size = line_cloud->size();
  std::vector<Eigen::Vector3d> npoints_vec;
  if (size < 1) {
    npoints_vec.swap(points_vec);
    return;
  }
  //#pragma omp parallel for
  for (int i = 0; i < size - 1; i++) {
    Eigen::Vector3d p;
    p[0] = inrange_cloud->points[i].x;
    p[1] = inrange_cloud->points[i].y;
    p[2] = inrange_cloud->points[i].z;
    npoints_vec.push_back(p);
  }
  npoints_vec.swap(points_vec);
}

// file
string Tools::getFileName(string file_name) {
  string subname;
  for (auto i = file_name.end() - 1; *i != '/'; i--) {
    subname.insert(subname.begin(), *i);
  }
  return subname;
}
//加密
void Tools::fileEcrept(QString &path_str) {
  std::string file_path = fromQString(path_str);
  // encryption_.Ecrept(file_path);
}
//解密
void Tools::fileDecrept(QString &path_str) {
  std::string efile_path = fromQString(path_str);
  // encryption_.DEcrept(efile_path);
  std::string defile_path = efile_path.substr(0, efile_path.length() - 7);
  // std::cout<<"decrept..defile....."<< defile<<std::endl;
  path_str = toQString(defile_path);
}
void Tools::filePcdDecrept(QString &path_str,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                           Eigen::Vector4f &origin,
                           Eigen::Quaternionf &orientation) {
  std::string efile_path = fromQString(path_str);
  // int index= encryption_.Parsefile(efile_path, cloud, origin, orientation);
  // std::cout << "......"<< index<< "......"<<std::endl;
}

bool Tools::isCsvFile(QString &path_str) {
  std::string str = fromQString(path_str);
  std::string suf_ecrept = ".csv";
  bool ret = isSameSuffix(str, suf_ecrept);
  return ret;
}
bool Tools::isGidFile(QString &path_str) {
  std::string str = fromQString(path_str);
  std::string suf_ecrept = "gid.csv";
  bool ret = isSameSuffix(str, suf_ecrept);
  return ret;
}
bool Tools::isOsmFile(QString &path_str) {
  std::string str = fromQString(path_str);
  std::string suf_ecrept = ".osm";
  bool ret = isSameSuffix(str, suf_ecrept);
  return ret;
}
bool Tools::isCsvEcreptFile(QString &path_str) {
  std::string str = fromQString(path_str);
  std::string suf_ecrept = ".csv.ecrept";
  bool ret = isSameSuffix(str, suf_ecrept);
  return ret;
}
bool Tools::isOsmEcreptFile(QString &path_str) {
  std::string str = fromQString(path_str);
  std::string suf_ecrept = ".osm.ecrept";
  bool ret = isSameSuffix(str, suf_ecrept);
  return ret;
}
bool Tools::isPcdEcreptFile(QString &path_str) {
  std::string str = fromQString(path_str);
  std::string suf_ecrept = ".pcd.cti";
  bool ret = isSameSuffix(str, suf_ecrept);
  return ret;
}

bool Tools::isSameSuffix(std::string &path_str, std::string &suffix_str) {
  std::string substr = path_str.substr(path_str.length() - suffix_str.length(),
                                       suffix_str.length());
  // std::cout << " test path_str...." << path_str << std::endl;
  // std::cout << " test suffix...." << substr << std::endl;
  if (strcmp(substr.c_str(), suffix_str.c_str()) == 0) return true;
  return false;
}
void Tools::removeFiles(bool remove_flag, QStringList &files) {
  if (remove_flag) {
    for (auto &file : files) {
      std::string file_path = fromQString(file);
      remove(file_path.c_str());
    }
  }
}
void Tools::removeFiles(bool remove_flag, QString &file) {
  if (remove_flag) {
    std::string file_path = fromQString(file);
    remove(file_path.c_str());
  }
}
void Tools::removeOldFiles(QString &file) {
  std::string file_str = fromQString(file) + ".ecrept";
  remove(file_str.c_str());
  file_str = fromQString(file) + ".key";
  remove(file_str.c_str());
}
void Tools::removeOldFile(QString &file) {
  QFileInfo FileInfo(file);
  if (FileInfo.isFile())
  {
    QFile::remove(file);
  }
}

// std::string <--> qstring
QString Tools::toQString(const string &s) {
  QString qs(s.c_str());
  return qs;
}
string Tools::fromQString(const QString &qs) {
  string s = qs.toUtf8().data();
  return s;
}

string joinStrVec(const vector<string> v, string splitor) {
  string s = "";
  if (v.size() == 0) return s;
  // //#pragma omp parallel for
  for (int i = 0; i != v.size() - 1; ++i) {
    s += (v[i] + splitor);
  }
  s += v[v.size() - 1];
  return s;
}

void timeStart() { myTime.start(); }

QString timeOff() {
  int timediff = myTime.elapsed();
  float f = timediff / 1000.0;
  QString tr_timediff = QString("%1").arg(f);  // float->QString
  return tr_timediff;
}
