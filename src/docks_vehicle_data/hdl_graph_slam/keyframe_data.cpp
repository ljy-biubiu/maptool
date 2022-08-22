// SPDX-License-Identifier: BSD-2-Clause

#include <hdl_graph_slam/keyframe_data.hpp>

#include <boost/filesystem.hpp>

// inline double toSec(rclcpp::Time time) {
//     return time.seconds();
// }

namespace hdl_graph_slam {
// 没有加载点云，directory为文件
KeyFrameData::KeyFrameData(const std::string &directory) :
    odom(Eigen::Isometry3d::Identity()), accum_distance(-1) {
    load(directory);
}

KeyFrameData::KeyFrameData() :
    odom(Eigen::Isometry3d::Identity()), accum_distance(-1) {
}

KeyFrameData &KeyFrameData::operator=(const KeyFrameData &keyframe) {
    if (this == &keyframe) {
        return *this;
    }
    this->id = keyframe.id;
    this->estimate = keyframe.estimate;
    // this->stamp = keyframe.stamp;
    this->odom = keyframe.odom;
    this->accum_distance = keyframe.accum_distance;

    if (keyframe.floor_coeffs) {
        Eigen::Vector4d coeffs((*keyframe.floor_coeffs)(0), (*keyframe.floor_coeffs)(1), (*keyframe.floor_coeffs)(2), (*keyframe.floor_coeffs)(3));
        this->floor_coeffs = coeffs;
    }

    if (keyframe.utm_coord) {
        Eigen::Vector3d coord((*keyframe.utm_coord)(0), (*keyframe.utm_coord)(1), (*keyframe.utm_coord)(2));
        this->utm_coord = coord;
    }

    if (keyframe.acceleration) {
        Eigen::Vector3d acc((*keyframe.acceleration)(0), (*keyframe.acceleration)(1), (*keyframe.acceleration)(2));
        this->acceleration = acc;
    }

    if (keyframe.orientation) {
        Eigen::Quaterniond quat((*keyframe.orientation).w(), (*keyframe.orientation).x(), (*keyframe.orientation).y(), (*keyframe.orientation).z());
        this->orientation = quat;
    }

    if (keyframe.lidar_name) {
        std::string name = (*keyframe.lidar_name);
        this->lidar_name = name;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (keyframe.dr_odom_used) {
        int tmp = (*keyframe.dr_odom_used);
        this->dr_odom_used = tmp;
    }

    if (keyframe.dr_odom) {
        Eigen::Isometry3d odom = (*keyframe.dr_odom);
        this->dr_odom = odom;
    }
    //////////
    if (keyframe.visual_odom_used) {
        int tmp = (*keyframe.visual_odom_used);
        this->visual_odom_used = tmp;
    }

    if (keyframe.visual_odom) {
        Eigen::Isometry3d odom = (*keyframe.visual_odom);
        this->visual_odom = odom;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // this->img_stamp = keyframe.img_stamp;
    // if (keyframe.image_stamp) {
        // rclcpp::Time stamp = (*keyframe.image_stamp);
        // this->image_stamp = stamp;
    // }

    if (keyframe.camera_matrix) {
        cv::Mat matrix(3, 3, CV_64F);
        (*keyframe.camera_matrix).copyTo(matrix);
        this->camera_matrix = matrix;
    }

    if (keyframe.distortion_coefficients) {
        cv::Mat matrix(1, 5, CV_64F);
        (*keyframe.camera_matrix).copyTo(matrix);
        this->camera_matrix = matrix;
    }

    if (keyframe.distortion_model) {
        std::string model = (*keyframe.distortion_model);
        this->distortion_model = model;
    }

    if (keyframe.camera_name) {
        std::string name = (*keyframe.camera_name);
        this->camera_name = name;
    }

    if (keyframe.camlidar_rvec) {
        cv::Vec<double, 3> vec;
        for (int i = 0; i < 3; i++)
            vec[i] = (*keyframe.camlidar_rvec)[i];
        this->camlidar_rvec = vec;
    }

    if (keyframe.camlidar_tvec) {
        cv::Vec<double, 3> vec;
        for (int i = 0; i < 3; i++)
            vec[i] = (*keyframe.camlidar_tvec)[i];
        this->camlidar_rvec = vec;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // if (keyframe.gnss_stamp) {
    //     rclcpp::Time stamp = (*keyframe.gnss_stamp);
    //     this->gnss_stamp = stamp;
    // }

    if (keyframe.gnss_yaw) {
        double tmp = (*keyframe.gnss_yaw);
        this->gnss_yaw = tmp;
    }

    if (keyframe.lat) {
        double tmp = (*keyframe.lat);
        this->lat = tmp;
    }

    if (keyframe.lon) {
        double tmp = (*keyframe.lon);
        this->lon = tmp;
    }

    if (keyframe.alt) {
        double tmp = (*keyframe.alt);
        this->alt = tmp;
    }

    if (keyframe.lat_lon_err) {
        double tmp = (*keyframe.lat_lon_err);
        this->lat_lon_err = tmp;
    }

    if (keyframe.alt_err) {
        double tmp = (*keyframe.alt_err);
        this->alt_err = tmp;
    }

    if (keyframe.sats_used) {
        int tmp = (*keyframe.sats_used);
        this->sats_used = tmp;
    }

    if (keyframe.gnss_used) {
        int tmp = (*keyframe.gnss_used);
        this->gnss_used = tmp;
    }

    if (keyframe.baro_height) {
        double tmp = (*keyframe.baro_height);
        this->baro_height = tmp;
    }

    if (keyframe.alt_used) {
        int tmp = (*keyframe.alt_used);
        this->alt_used = tmp;
    }

    if (keyframe.baro_height_used) {
        int tmp = (*keyframe.baro_height_used);
        this->baro_height_used = tmp;
    }

    if (keyframe.gnss_yaw_used) {
        int tmp = (*keyframe.gnss_yaw_used);
        this->gnss_yaw_used = tmp;
    }

    if (keyframe.is_degenerate) {
        int tmp = (*keyframe.is_degenerate);
        this->is_degenerate = tmp;
    }

    return *this;
}

KeyFrameData::~KeyFrameData() {
}

void KeyFrameData::save(const std::string &directory) {
    if (!boost::filesystem::is_directory(directory)) {
        boost::filesystem::create_directory(directory);
    }

    if ((!lat) || (!lon) || (!alt) || (!lat_lon_err) || (!alt_used) || (!alt_err) || (!sats_used)) {
        // gnss_stamp = boost::optional<rclcpp::Time>();
        lat = boost::optional<double>();
        lon = boost::optional<double>();
        lat_lon_err = boost::optional<double>();
        alt_used = boost::optional<int>();
        alt = boost::optional<double>();
        alt_err = boost::optional<double>();
        sats_used = boost::optional<int>();
    }
    if ((!baro_height_used) || (!baro_height)) {
        baro_height_used = boost::optional<int>();
        baro_height = boost::optional<double>();
    }

    long long int val_nanosecs;
    std::ofstream ofs(directory + "/data");
    // val_nanosecs = stamp.nanoseconds();
    // ofs << "stamp " << (val_nanosecs / 1000000000) << " " << (val_nanosecs % 1000000000) << "\n";

    ofs << "estimate\n";
    ofs << estimate << "\n";

    ofs << "odom\n";
    ofs << odom.matrix() << "\n";

    ofs << "accum_distance " << accum_distance << "\n";

    if (floor_coeffs) {
        ofs << "floor_coeffs " << floor_coeffs->transpose() << "\n";
    }

    if (utm_coord) {
        ofs << "utm_coord " << utm_coord->transpose() << "\n";
    }

    if (acceleration) {
        ofs << "acceleration " << acceleration->transpose() << "\n";
    }

    if (orientation) {
        ofs << "orientation " << orientation->w() << " " << orientation->x() << " " << orientation->y() << " " << orientation->z() << "\n";
    }

    {
        ofs << "id " << id << "\n";
    }

    if (lidar_name) {
        ofs << "lidar_name " << *lidar_name << "\n";
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (dr_odom_used) {
        ofs << "dr_odom_used " << *dr_odom_used << "\n";
    }

    if (dr_odom) {
        ofs << "dr_odom\n";
        ofs << dr_odom->matrix() << "\n";
    }

    //////////

    if (visual_odom_used) {
        ofs << "visual_odom_used " << *visual_odom_used << "\n";
    }

    if (visual_odom) {
        ofs << "visual_odom\n";
        ofs << visual_odom->matrix() << "\n";
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // if (image_stamp) {
    //     val_nanosecs = image_stamp->nanoseconds();
    //     ofs << "image_stamp " << (val_nanosecs / 1000000000) << " " << (val_nanosecs % 1000000000) << "\n";
    //     //ofs << "image_stamp " << image_stamp->sec << " " << image_stamp->nsec << "\n";
    // }

    if (camera_name) {
        ofs << "camera_name " << *camera_name << "\n";
    }

    if (distortion_model) {
        ofs << "distortion_model " << *distortion_model << "\n";
    }

    if (camera_matrix) {
        ofs << "camera_matrix\n ";
        for (int i = 0; i < camera_matrix->rows; i++) {
            for (int j = 0; j < camera_matrix->cols; j++) {
                ofs << camera_matrix->at<double>(i, j) << " ";
            }
            ofs << "\n";
        }
    }

    if (distortion_coefficients) {
        ofs << "distortion_coefficients\n";
        for (int i = 0; i < distortion_coefficients->rows; i++) {
            for (int j = 0; j < distortion_coefficients->cols; j++) {
                ofs << distortion_coefficients->at<double>(i, j) << " ";
            }
            ofs << "\n";
        }
    }

    if (camlidar_tvec) {
        ofs << "T_l_c " << (*camlidar_tvec)[0] << " " << (*camlidar_tvec)[1] << " " << (*camlidar_tvec)[2] << "\n";
    }

    if (camlidar_rvec) {
        ofs << "R_l_c " << (*camlidar_rvec)[0] << " " << (*camlidar_rvec)[1] << " " << (*camlidar_rvec)[2] << "\n";
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // if (gnss_stamp) {
    //     val_nanosecs = gnss_stamp->nanoseconds();
    //     ofs << "gnss_stamp " << (val_nanosecs / 1000000000) << " " << (val_nanosecs % 1000000000) << "\n";
    //     //ofs << "gnss_stamp " << gnss_stamp->sec << " " << gnss_stamp->nsec << "\n";
    // }

    if (lat) {
        ofs << "lat " << std::setprecision(16) << *lat << "\n";
    }

    if (lon) {
        ofs << "lon " << std::setprecision(16) << *lon << "\n";
    }

    if (lat_lon_err) {
        ofs << "lat_lon_err " << *lat_lon_err << "\n";
    }

    if (sats_used) {
        ofs << "sats_used " << *sats_used << "\n";
    }

    if (gnss_used) {
        ofs << "gnss_used " << *gnss_used << "\n";
    }

    //////////

    if (alt_used) {
        ofs << "alt_used " << *alt_used << "\n";
    }

    if (alt) {
        ofs << "alt " << *alt << "\n";
    }

    if (alt_err) {
        ofs << "alt_err " << *alt_err << "\n";
    }

    //////////

    if (gnss_yaw_used) {
        ofs << "gnss_yaw_used " << *gnss_yaw_used << "\n";
    }

    if (gnss_yaw) {
        ofs << "gnss_yaw " << *gnss_yaw << "\n";
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (baro_height_used) {
        ofs << "baro_height_used " << *baro_height_used << "\n";
    }

    if (baro_height) {
        ofs << "baro_height " << *baro_height << "\n";
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (is_degenerate) {
        ofs << "is_degenerate " << *is_degenerate << "\n";
    }
}

bool KeyFrameData::load(const std::string &directory) {
    std::ifstream ifs(directory + "/data");
    if (!ifs) {
        return false;
    }

    long long int val_nanosecs;

    while (!ifs.eof()) {
        std::string token;
        ifs >> token;

        if (token == "stamp") {
            long long int tmp_val;
            ifs >> val_nanosecs;
            val_nanosecs *= 1000000000;
            ifs >> tmp_val;
            val_nanosecs += tmp_val;
            // stamp = rclcpp::Time(val_nanosecs);
            //ifs >> stamp.sec >> stamp.nsec;
        }
        else if (token == "estimate") {
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    ifs >> estimate(i, j);
                }
            }
        }
        else if (token == "odom") {
            Eigen::Matrix4d odom_mat = Eigen::Matrix4d::Identity();
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    ifs >> odom_mat(i, j);
                }
            }
            //Eigen::Isometry3d odom构造变换矩形阵
            odom.setIdentity();
            odom.linear() = odom_mat.block<3, 3>(0, 0);
            odom.translation() = odom_mat.block<3, 1>(0, 3);
        }
        else if (token == "accum_distance") {
            ifs >> accum_distance;
        }
        else if (token == "floor_coeffs") {
            Eigen::Vector4d coeffs;
            ifs >> coeffs[0] >> coeffs[1] >> coeffs[2] >> coeffs[3];
            floor_coeffs = coeffs;
        }
        else if (token == "utm_coord") {
            Eigen::Vector3d coord;
            ifs >> coord[0] >> coord[1] >> coord[2];
            utm_coord = coord;
        }
        else if (token == "acceleration") {
            Eigen::Vector3d acc;
            ifs >> acc[0] >> acc[1] >> acc[2];
            acceleration = acc;
        }
        else if (token == "orientation") {
            Eigen::Quaterniond quat;
            ifs >> quat.w() >> quat.x() >> quat.y() >> quat.z();
            orientation = quat;
        }
        else if (token == "id") {
            ifs >> id;
        }
        else if (token == "lidar_name") {
            std::string tmp;
            ifs >> tmp;
            lidar_name = tmp;
            continue;
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        else if (token == "dr_odom_used") {
            int tmp;
            ifs >> tmp;
            dr_odom_used = tmp;
        }
        else if (token == "dr_odom") {
            Eigen::Matrix4d odom_mat = Eigen::Matrix4d::Identity();
            Eigen::Isometry3d tmp;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    ifs >> odom_mat(i, j);
                }
            }
            tmp.setIdentity();
            tmp.linear() = odom_mat.block<3, 3>(0, 0);
            tmp.translation() = odom_mat.block<3, 1>(0, 3);
            dr_odom = tmp;
        }
        //////////
        else if (token == "visual_odom_used") {
            int tmp;
            ifs >> tmp;
            visual_odom_used = tmp;
        }
        else if (token == "visual_odom") {
            Eigen::Matrix4d odom_mat = Eigen::Matrix4d::Identity();
            Eigen::Isometry3d tmp;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    ifs >> odom_mat(i, j);
                }
            }
            tmp.setIdentity();
            tmp.linear() = odom_mat.block<3, 3>(0, 0);
            tmp.translation() = odom_mat.block<3, 1>(0, 3);
            visual_odom = tmp;
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        else if (token == "image_stamp") {
            long long int tmp_val;
            ifs >> val_nanosecs;
            val_nanosecs *= 1000000000;
            ifs >> tmp_val;
            val_nanosecs += tmp_val;
            // image_stamp = rclcpp::Time(val_nanosecs);
            // img_stamp = toSec(*image_stamp);
            continue;
        }
        else if (token == "distortion_model") {
            std::string tmp;
            ifs >> tmp;
            distortion_model = tmp;
            continue;
        }
        else if (token == "camera_name") {
            std::string tmp;
            ifs >> tmp;
            camera_name = tmp;
            continue;
        }
        else if (token == "camera_matrix") {
            cv::Mat matrix(3, 3, CV_64F);
            ifs >> matrix.at<double>(0, 0) >> matrix.at<double>(0, 1) >> matrix.at<double>(0, 2)
                >> matrix.at<double>(1, 0) >> matrix.at<double>(1, 1) >> matrix.at<double>(1, 2)
                >> matrix.at<double>(2, 0) >> matrix.at<double>(2, 1) >> matrix.at<double>(2, 2);
            //matrix.copyTo(camera_matrix);
            camera_matrix = matrix;
            continue;
        }
        else if (token == "distortion_coefficients") {
            std::vector<double> coefficients;
            int distor_model_len = 5;
            if (*distortion_model == "pinhole")
                distor_model_len = 5;
            else if (*distortion_model == "kannala_brandt")
                distor_model_len = 4;
            for (int i = 0; i < distor_model_len; i++) {
                double val;
                ifs >> val;
                coefficients.push_back(val);
            }
            int len = coefficients.size();
            cv::Mat coef_mat(1, len, CV_64F);
            for (int i = 0; i < len; i++)
                coef_mat.at<double>(0, i) = coefficients[i];
            distortion_coefficients = coef_mat;
            continue;
        }
        else if (token == "R_l_c") {
            cv::Vec<double, 3> rvec;
            ifs >> rvec[0] >> rvec[1] >> rvec[2];
            camlidar_rvec = rvec;
            continue;
        }
        else if (token == "T_l_c") {
            cv::Vec<double, 3> tvec;
            ifs >> tvec[0] >> tvec[1] >> tvec[2];
            camlidar_tvec = tvec;
            continue;
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        else if (token == "gnss_stamp") {
            long long int tmp_val;
            ifs >> val_nanosecs;
            val_nanosecs *= 1000000000;
            ifs >> tmp_val;
            val_nanosecs += tmp_val;
            // gnss_stamp = rclcpp::Time(val_nanosecs);
            continue;
        }

        else if (token == "lat") {
            double tmp;
            ifs >> tmp;
            lat = tmp;
        }
        else if (token == "lon") {
            double tmp;
            ifs >> tmp;
            lon = tmp;
        }
        else if (token == "lat_lon_err") {
            double tmp;
            ifs >> tmp;
            lat_lon_err = tmp;
        }
        else if (token == "sats_used") {
            int tmp;
            ifs >> tmp;
            sats_used = tmp;
        }
        else if (token == "gnss_used") {
            int tmp;
            ifs >> tmp;
            gnss_used = tmp;
        }
        //////////
        else if (token == "alt_used") {
            int tmp;
            ifs >> tmp;
            alt_used = tmp;
        }
        else if (token == "alt") {
            double tmp;
            ifs >> tmp;
            alt = tmp;
        }
        else if (token == "alt_err") {
            double tmp;
            ifs >> tmp;
            alt_err = tmp;
        }
        //////////
        else if (token == "gnss_yaw_used") {
            int tmp;
            ifs >> tmp;
            gnss_yaw_used = tmp;
        }
        else if (token == "gnss_yaw") {
            double yaw;
            ifs >> yaw;
            gnss_yaw = yaw;
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        else if (token == "baro_height") {
            double tmp;
            ifs >> tmp;
            baro_height = tmp;
        }
        else if (token == "baro_height_used") {
            int tmp;
            ifs >> tmp;
            baro_height_used = tmp;
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        else if (token == "is_degenerate") {
            int tmp;
            ifs >> tmp;
            is_degenerate = tmp;
        }
    }

    if ((!lat) || (!lon) || (!alt) || (!lat_lon_err) || (!alt_used) || (!alt_err) || (!sats_used)) {
        // gnss_stamp = boost::optional<rclcpp::Time>();
        lat = boost::optional<double>();
        lon = boost::optional<double>();
        lat_lon_err = boost::optional<double>();
        alt_used = boost::optional<int>();
        alt = boost::optional<double>();
        alt_err = boost::optional<double>();
        sats_used = boost::optional<int>();
    }
    if ((!baro_height_used) || (!baro_height)) {
        baro_height_used = boost::optional<int>();
        baro_height = boost::optional<double>();
    }

    return true;
}

} // namespace hdl_graph_slam
