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
    VisualOdometry_monocular(std::string new_data_directory, // maybe we should declare this variable in main instead // pareil pour images ? 
                             std::string new_output_poses);
    // ~VisualOdometry_monocular();

    int motion_estimation(std::vector<cv::Point2f>& matching_points1,
                          std::vector<cv::Point2f>& matching_points2,
                          cv::Mat& Ri, 
                          cv::Mat& ti);

    int filter_good_matches(const std::vector<std::vector<cv::DMatch>> matches, 
                            const float ratio_thresh, 
                            std::vector<cv::DMatch>& good_matches);
                              
    int extract_and_matche_features(int image_index, 
                                    cv::Mat& descriptors1, 
                                    cv::Mat& descriptors2, 
                                    std::vector<cv::KeyPoint>& keypoints1,  
                                    std::vector<cv::KeyPoint>& keypoints2,
                                    std::vector<cv::Point2f>& matching_points1,
                                    std::vector<cv::Point2f>& matching_points2,
                                    cv::Mat& matching_descriptors2);
    
    int find_3Dpoints(cv::Mat& projection_matrix1,
                      cv::Mat& projection_matrix2,
                      std::vector<cv::Point2f>& matching_points1,
                      std::vector<cv::Point2f>& matching_points2,
                      std::vector<cv::Point3f>& points3D);

    cv::Mat find_Rti(std::vector<cv::Point2f> matching_points,
                     std::vector<cv::Point3f> points3D,
                     cv::Mat& Rti);

    cv::Mat fuse_R_t(cv::Mat R, cv::Mat t);

    // void main_2D_to_2D();
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
    std::string _data_directory;
    std::string _output_poses;
    std::vector<cv::Mat> images;
    
    cv::Mat intrinsic_matrix = cv::Mat::zeros(3, 3, CV_32F);
    cv::Mat projection_matrix = cv::Mat::zeros(4, 4, CV_32F);

    // cv::Mat desci_0;
    // cv::Mat desci_1;

    // std::vector<cv::KeyPoint> kpi_0; 
    // std::vector<cv::KeyPoint> kpi_1; //    also create a pointer to change ref reasily ? 

    // std::vector<cv::Point2f> matching_kpi_01;
    // std::vector<cv::Point2f> matching_kpi_10;

    
    cv::Ptr<cv::Feature2D> orb = cv::ORB::create(3000); // we can change the number 

    // TODO Je ne sais pas porquoi ces params la
    cv::Ptr<cv::flann::IndexParams> index_params = cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1);
    cv::Ptr<cv::flann::SearchParams> search_params = cv::makePtr<cv::flann::SearchParams>(50);
    cv::Ptr<cv::FlannBasedMatcher> my_flann = cv::makePtr<cv::FlannBasedMatcher>(index_params, search_params);

    
};

#endif // VO_MONOCULAR_HPP