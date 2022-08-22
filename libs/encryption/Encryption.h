#include <iostream>
#include <fstream>
#include <string>
// #include <eigen_conversions/eigen_msg.h>
#include <pcl/PCLPointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;

class Encryption
{
    public:
        Encryption();
        ~Encryption();
        void Serialfile(const string &plain_file_in);
        int Parsefile(const string &plain_file_in,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,Eigen::Vector4f &origin,Eigen::Quaternionf &orientation);
        void Ecrept(const string &plain_file_in);
        void DEcrept(const string &plain_file_in);
    private:

};