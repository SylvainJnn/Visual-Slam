#ifndef VO_MONOCULAR_HPP
#define VO_MONOCULAR_HPP

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include <opencv2/opencv.hpp>



class VisualOdometry_monocular
{
public:
    VisualOdometry_monocular();
    VisualOdometry_monocular(std::string new_data_directory);
    // ~VisualOdometry_monocular();

    cv::Mat motion_estimation();
    int extract_and_matche_features(int image_index);
    cv::Mat fuse_R_t(cv::Mat R, cv::Mat t);

    void main_2D_to_2D();
    void main_3D_to_2D();

private:
    // functions to get dataset information
    int get_images(const std::string& folder_path);
    int get_calibration(const std::string& folder_path);
    cv::Mat get_first_pose(const std::string& folder_path);

    int write_pose(const std::string& folder_path, const cv::Mat& poses);

    // print matched between two images -> use for debug
    void printMatches(const std::vector<cv::DMatch>& matches);
    void printMatchesArray(const std::vector<std::vector<cv::DMatch>>& matches_array);

    // TODO mieux en local ou pas ? 
    // Attribute
    std::string data_directory;
    std::vector<cv::Mat> images;
    
    cv::Mat intrinsic_matrix = cv::Mat::zeros(3, 3, CV_32F);
    cv::Mat projection_matrix = cv::Mat::zeros(4, 4, CV_32F);

    cv::Mat current_descriptors;
    cv::Mat previous_descriptors;

    std::vector<cv::KeyPoint> current_keypoints; 
    std::vector<cv::KeyPoint> previous_keypoints; //    also create a pointer to change ref reasily ? 

    
    cv::Ptr<cv::Feature2D> orb = cv::ORB::create(3000); // we can change the number 

    // TODO Je ne sais pas porquoi ces params la
    cv::Ptr<cv::flann::IndexParams> index_params = cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1);
    cv::Ptr<cv::flann::SearchParams> search_params = cv::makePtr<cv::flann::SearchParams>(50);
    cv::Ptr<cv::FlannBasedMatcher> my_flann = cv::makePtr<cv::FlannBasedMatcher>(index_params, search_params);

    std::vector<cv::Point2f> q_current;
    std::vector<cv::Point2f> q_previous;
};

#endif // VO_MONOCULAR_HPP