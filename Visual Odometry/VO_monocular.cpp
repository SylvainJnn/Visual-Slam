#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include <opencv2/opencv.hpp>
#include "include/VO_monocular.hpp"

using namespace std;


// Constructor
VisualOdometry_monocular::VisualOdometry_monocular()
{
    std:cout<<"Nothing specified\n";
}

VisualOdometry_monocular::VisualOdometry_monocular(std::string new_data_directory)
{
    data_directory = new_data_directory;
}

// int dont know (cv::Mat essential_matrix, cv::Mat R1, cv::Mat R2, cv::Mat t)
// {
//     // A - assemble all poential T
//     cv::Mat T1, T2, T3, T4; 
//     T1 = fuse_R_t(R1, t);
//     T2 = fuse_R_t(R2, t);
//     T3 = fuse_R_t(R1, -t);
//     T4 = fuse_R_t(R2, -t);

//     // triangulate to find the relative scale
//     std::vector projections{K * T1, K * T2 ,K * T3, K * T4};

//     // https://realitybytes.blog/tag/monocular-visual-odometry/


//     // find bigger positives
//     return(0);
// }


cv::Mat VisualOdometry_monocular::motion_estimation()
{
    // 3 - Compute essential matrix for image pair Ik-1 and Ik 
    cv::Mat essential_matrix, mask_essential_matrix; 
    essential_matrix = cv::findEssentialMat(q_previous, q_current, intrinsic_matrix, cv::RANSAC);

    // 4 - Decompose essential matriintrinsic_matrixce to Ri and ti
    // Attention, several Rotation matrice are possible
    cv::Mat Ri, ti; // q dans le bon ordre ? 
    
    cv::recoverPose(essential_matrix, q_previous, q_current, intrinsic_matrix, Ri, ti);

    // cv::Mat R1, R2, t;
    // cv::decomposeEssentialMat(essential_matrix, R1, R2, t);

    // jesaispas(essential_matrix, R1, R2, t);
    // std::cout << "ti \n " << ti <<std::endl;
    // std::cout << "Ri \n " <<Ri <<std::endl;

    cv::Mat Ti;

    // std::cout << "Ri  :\n" << Ri <<std::endl;
    // std::cout << "ti  :\n" << ti <<std::endl;
    Ti = fuse_R_t(Ri, ti);
    // write_pose("poses/transform.txt", Ti);
    return(Ti);
}

int VisualOdometry_monocular::extract_and_matche_features(int image_index) // en input le numéros de l'image actuelle 
{
    /*
    A - compute keypoints and descriptors
    B - match them
    C - filter only the good match
    D - put in arrays q_i: coordinates vector of the matching keypoints on image i

    Input
    - image_index : index of the current image

    Output:
    - q_current : vector of points representing the coordinates of the keypoints in the current image (image ith)
    - q_previous : vector of points representing the coordinates of the matching keypoints in the previous image (image i-1th)
    */
    // A - compute keypoints and descriptors
    previous_descriptors = current_descriptors.clone();
    previous_keypoints = current_keypoints;
    orb->detectAndCompute(images[image_index], cv::noArray(), current_keypoints, current_descriptors); // normaleltn c'est update 

    // B - match features
    std::vector<std::vector<cv::DMatch>> matches;
    my_flann->knnMatch(previous_descriptors, current_descriptors, matches, 2);
    // printMatches(matches);


    // C - filter only the good match
    const float ratio_thresh = 0.5f; // TODO c'est quoi ce ration
    std::vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < matches.size(); i++) 
    {
        if(matches[i].empty()) // check if the ith cell of matches is empty, if it is we drop it to avoid a segfault, otherwise we continue
        {
            std::cout << "Error : cell " << i << " of matches in image " << image_index << " is empty." << std::endl;
        }
        else if (matches[i][0].distance < (ratio_thresh * matches[i][1].distance))
        {
            good_matches.push_back(matches[i][0]);
        }
    }
        
    // Filter the best match
    // std::vector<cv::Point2f> q_current;
    // std::vector<cv::Point2f> q_previous;

    // D - put in arrays q_i: coordinates vector of the matching keypoints on image i
    // Reset
    q_current.clear();
    q_previous.clear();
    for (const auto& good : good_matches) 
    {
        q_previous.push_back(previous_keypoints[good.queryIdx].pt);
        q_current.push_back(current_keypoints[good.trainIdx].pt);
    }

    //TEST
    // cv::Mat img_matches;
    // cv::drawMatches(images[image_index-1], previous_keypoints, 
    //                 images[image_index], current_keypoints, 
    //                 good_matches, img_matches, 
    //                 cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    // cv::resize(img_matches, img_matches, cv::Size(), 0.75, 0.75);
    // cv::imshow("Good Matches VO", img_matches);
    // cv::waitKey(0);

    return(0);
}

cv::Mat VisualOdometry_monocular::fuse_R_t(cv::Mat R, cv::Mat t)
{
    cv::Mat T = cv::Mat::eye(4, 4, CV_32F); // Initialiser à une matrice identité 4x4
    R.copyTo(T(cv::Rect(0, 0, 3, 3))); // Copy R in the 3 first lines and columns of M
    t.copyTo(T(cv::Rect(3, 0, 1, 3))); // Copy t dans in the last column of M
    
    // std::cout << "T = \n" << T << std::endl;

    return(T);
}

// main algo --> changer de nom ?
void VisualOdometry_monocular::main()
{
    // 1 - get images et setup calibrations 
    get_images(data_directory);
    get_calibration(data_directory);
    
    cv::Mat Ti = cv::Mat::zeros(4, 4, CV_32F); // Transform from image i-1 and i
    cv::Mat Ci = cv::Mat::eye(4, 4, CV_32F); // current pose
    Ci = get_first_pose(data_directory);

    // Initiate current keypoints and descriptors --> the loop start from iamge 1 and re use the previous keypoints and descriptors 
    orb->detectAndCompute(images[0], cv::noArray(), current_keypoints, current_descriptors);

    for(size_t image_index = 1; image_index < images.size(); image_index++)
    {          
        // 2 - extract and matche feature between Ik-1 and Ik || Todo, only extract one image at the same time (put old image OR feature in a pointer variable )
        extract_and_matche_features((int) image_index);
        
        // 3 - Compute essential matrix for image pair Ik-1 and Ik 
        // 4 - Decompose essential matrix into Rk and tk, and from Tk
        // 5 - Compute relatice scale and resacle tk accordingly // fait dans Recoverpose
        Ti = motion_estimation();
        
        // 6 - Concatenate transformation by computing Ck = Ck-1 Tk
        //Ci = Ci.mul(Ti);//...(C x Ti)
        Ci = Ci * Ti.inv();

        // std::cout << "Ti  :\n" << Ti <<std::endl;
        // std::cout << " Ci :\n" << Ci <<std::endl;
        write_pose("poses/my_poses_seq2_vccheck.txt", Ci);
        std::cout << image_index <<std::endl;
    }
}


int VisualOdometry_monocular::get_images(const std::string& folder_path)
{
    std::string images_folder_path = folder_path + "/image_l/*.png";
    std::vector<cv::String> files_name;
    cv::glob(images_folder_path, files_name, false);
    for (size_t i=0; i<files_name.size(); i++)
        images.push_back(cv::imread(files_name[i]));
    return(0);
}

// get calibration
int VisualOdometry_monocular::get_calibration(const std::string& folder_path)
{
    /*
    Read calib.txt file and get the intrinsict and projection matrix 

    Input
    - folder_path : path of the file containning the calibration

    Output:
    
    */
    std::string calibration_path = folder_path + "/calib.txt";  
    float value;
    std::ifstream file(calibration_path);
    for (int i = 0; i < 3; i++) 
    { 
        for (int j = 0; j < 4; j++) 
        {
            file >> value;
            if(j != 3)
            {
                intrinsic_matrix.at<float>(i, j) = value;
            }
            projection_matrix.at<float>(i, j) = value;
        }
    }
    return(1);
}

cv::Mat VisualOdometry_monocular::get_first_pose(const std::string& folder_path)
{
    cv::Mat C0 = cv::Mat::eye(4, 4, CV_32F);
    std::string poses_path = folder_path + "/poses.txt";
    float value;
    std::ifstream file(poses_path);
    for (int i = 0; i < 3; i++) 
    { 
        for (int j = 0; j < 4; j++) 
        {
            file >> value;
            C0.at<float>(i, j) = value;
        }
    }
    
    return(C0);
}

int VisualOdometry_monocular::write_pose(const std::string& folder_path, const cv::Mat& pose)
{
    std::string poses_path = folder_path;// + "my_poses.txt";
    std::ofstream pose_file(poses_path, std::ios::app);
    if(pose_file.is_open()) 
    {
        for(int i = 0; i < pose.rows; i++)
        {
            for(int j = 0; j < pose.cols ; j++)
            {
                pose_file << pose.at<float>(i,j) << " ";
            }
        }
        pose_file << std::endl;
        pose_file.close();
        std::cout << "line sucessfully written." << std::endl;
    } 
    else 
    {
        std::cerr << "Error : Cannot open file." << std::endl;
    }

    return(0);
}

void VisualOdometry_monocular::printMatches(const std::vector<cv::DMatch>& matches) 
{
    for (size_t i = 0; i < matches.size(); i++) 
    {
        std::cout << "Match " << i << ":\n"
                << "  QueryIdx: " << matches[i].queryIdx
                << ", TrainIdx: " << matches[i].trainIdx
                << ", ImgIdx: " << matches[i].imgIdx
                << ", Distance: " << matches[i].distance
                << "\n";
    }
}

void VisualOdometry_monocular::printMatchesArray(const std::vector<std::vector<cv::DMatch>>& matches_array) 
{
    for (size_t i = 0; i < matches_array.size(); i++) 
    {
        std::cout << "Match array" << i << ":\n";
        printMatches(matches_array[i]);
    }
}


int main()
{
    VisualOdometry_monocular VO = VisualOdometry_monocular("example/KITTI_sequence_2");
    VO.main();
    return(0);
}

/*

- Comprendre flann et pourquoi c'est comme ça 
- pourquoi knn et pas un autre
- c'est quoi le mask à chaque fois ? 

- peut être que le cacule de la matrice Ri est mauvaishttps://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga54a2f5b3f8aeaf6c76d4a31dece85d5d
- L'axe y est vers le haut
- il faut comprendre la partie triangulation MAIS on sait que recover pose fait le taff à notre place
*/




