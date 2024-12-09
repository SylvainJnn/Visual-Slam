#ifndef VO_STEREO_HPP
#define VO_STEREO_HPP

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include <opencv2/opencv.hpp>






class VisualOdometry_stereo
{
public:
    VisualOdometry_stereo();
    VisualOdometry_stereo(std::string new_data_directory, // maybe we should declare this variable in main instead // pareil pour images ? 
                          std::string new_output_poses);

    ~VisualOdometry_stereo();

    int main();


private:

    int filter_good_matches(const std::vector<std::vector<cv::DMatch>> matches, 
                            const float ratio_thresh, 
                            std::vector<cv::DMatch>& good_matches);

    
    int filter_matching_points(std::vector<cv::DMatch> good_matches,
                               std::vector<cv::Point2f> points1, 
                               std::vector<cv::Point2f> points2,
                               std::vector<cv::Point2f>& matching_points1,
                               std::vector<cv::Point2f>& matching_points2);
    
    int filter_matching_points(std::vector<cv::DMatch> good_matches,
                               std::vector<cv::Point2f> points1,  
                               std::vector<cv::Point2f> points2,
                               cv::Mat desc1,
                               cv::Mat desc2,
                               std::vector<cv::Point2f>& matching_points1,
                               std::vector<cv::Point2f>& matching_points2,
                               cv::Mat& matching_descriptors1,
                               cv::Mat& matching_descriptors2);
    
    int find_3Dpoints(cv::Mat& projection_matrix1,
                      cv::Mat& projection_matrix2,
                      std::vector<cv::Point2f>& matching_points1,
                      std::vector<cv::Point2f>& matching_points2,
                      std::vector<cv::Point3f>& points3D);

    cv::Mat jsp(cv::Mat Rt);

    int show_matches_points(cv::Mat image1, 
                            cv::Mat image2,
                            std::vector<cv::Point2f> points1,
                            std::vector<cv::Point2f> points2,
                            std::vector<cv::DMatch> matches);

    int show_matches(cv::Mat image1, 
                     cv::Mat image2,
                     std::vector<cv::KeyPoint> keypoints1,
                     std::vector<cv::KeyPoint> keypoints2,
                     std::vector<cv::DMatch> matches);

    // functions to get dataset information
    int get_images(const std::string& folder_path,
                   std::vector<cv::Mat>& images_left,
                   std::vector<cv::Mat>& images_right);

    int get_calibration(const std::string& folder_path,
                        cv::Mat& intrinsic_matrix_left,
                        cv::Mat& intrinsic_matrix_right,
                        cv::Mat& projection_matrix_left,
                        cv::Mat& projection_matrix_right);

    cv::Mat get_first_pose(const std::string& folder_path);

    int write_pose(const std::string& folder_path, const cv::Mat& poses);

    int write_3Dpoints(const std::string& points_path, const std::vector<cv::Point3f>& points3D);

    // print matched between two images -> use for debug
    void printMatches(const std::vector<cv::DMatch>& matches);
    void printMatchesArray(const std::vector<std::vector<cv::DMatch>>& matches_array);

    // Attribute
    std::string _data_directory;
    std::string _output_poses;

    cv::Ptr<cv::Feature2D> _orb = cv::ORB::create(3000); // we can change the number 

    // TODO Je ne sais pas porquoi ces params la
    cv::Ptr<cv::flann::IndexParams> _index_params = cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1);
    cv::Ptr<cv::flann::SearchParams> _search_params = cv::makePtr<cv::flann::SearchParams>(50);
    cv::Ptr<cv::FlannBasedMatcher> _flann = cv::makePtr<cv::FlannBasedMatcher>(_index_params, _search_params);


};

#endif // VO_STEREO_HPP