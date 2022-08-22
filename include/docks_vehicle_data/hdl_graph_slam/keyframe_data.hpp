// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_DATA_HPP
#define KEYFRAME_DATA_HPP

// #include <rclcpp/rclcpp.hpp>
#include <boost/optional.hpp>

#include <Eigen/Dense>
#include <memory>  
// #include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace hdl_graph_slam {

/**
 * @brief KeyFrameData (pose node)
 */
class KeyFrameData {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<KeyFrameData>;

    KeyFrameData(const std::string &directory);
    KeyFrameData();

    KeyFrameData &operator=(const KeyFrameData &str);

    virtual ~KeyFrameData();

    void save(const std::string &directory);
    bool load(const std::string &directory);

public:
    long id = -1;                                    // id
    Eigen::Matrix4d estimate;                        // estimate
    // rclcpp::Time stamp;                              // timestamp
    Eigen::Isometry3d odom;                          // odometry (estimated by scan_matching_odometry)
    double accum_distance;                           // accumulated distance from the first node (by scan_matching_odometry)
    boost::optional<Eigen::Vector4d> floor_coeffs;   // detected floor's coefficients
    boost::optional<Eigen::Vector3d> utm_coord;      // UTM coord obtained by GPS
    boost::optional<Eigen::Vector3d> acceleration;   //
    boost::optional<Eigen::Quaterniond> orientation; //
    //---------------------additional data
    boost::optional<std::string> lidar_name;

    boost::optional<int> dr_odom_used;
    boost::optional<Eigen::Isometry3d> dr_odom;

    boost::optional<int> visual_odom_used;
    boost::optional<Eigen::Isometry3d> visual_odom;

    //--img
    double img_stamp;
    // boost::optional<rclcpp::Time> image_stamp;
    boost::optional<cv::Mat> camera_matrix;
    boost::optional<cv::Mat> distortion_coefficients;
    boost::optional<std::string> distortion_model;
    boost::optional<std::string> camera_name;
    boost::optional<cv::Vec<double, 3>> camlidar_rvec;
    boost::optional<cv::Vec<double, 3>> camlidar_tvec;

    //--gnss
    // boost::optional<rclcpp::Time> gnss_stamp;
    boost::optional<double> lat;
    boost::optional<double> lon;
    boost::optional<double> lat_lon_err;
    boost::optional<int> sats_used;
    boost::optional<int> gnss_used;

    boost::optional<int> alt_used; // z constrain switch
    boost::optional<double> alt;
    boost::optional<double> alt_err;

    boost::optional<int> gnss_yaw_used;
    boost::optional<double> gnss_yaw;

    //--baro
    boost::optional<int> baro_height_used; // z constrain switch
    boost::optional<double> baro_height;

    //--degenerate
    boost::optional<int> is_degenerate;
};

} // namespace hdl_graph_slam

#endif // KEYFRAME_DATA_HPP
