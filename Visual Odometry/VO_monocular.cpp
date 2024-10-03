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


/**
 * @brief compute the displacement between the current frame (fi) and the previous frame (fi-1)
 * @param q_current contains the keypoints of the frame fi
 * @param q_previous contains the keypoints of the frame fi-1
 * @return Ti Matrice that contains homogenous coordinates. It represents the displacement between Ti-1 and Ti
 */
int VisualOdometry_monocular::motion_estimation(std::vector<cv::Point2f>& q_current,
                                                std::vector<cv::Point2f>& q_previous,
                                                cv::Mat& Ri, 
                                                cv::Mat& ti)
{
    // 3 - Compute essential matrix for image pair Ik-1 and Ik 
    cv::Mat essential_matrix, mask_essential_matrix; 
    essential_matrix = cv::findEssentialMat(q_previous, q_current, intrinsic_matrix, cv::RANSAC);

    // 4 - Decompose essential matriintrinsic_matrixce to Ri and ti
    // Attention, several Rotation matrice are possible
    
    
    cv::recoverPose(essential_matrix, q_previous, q_current, intrinsic_matrix, Ri, ti);

    return(0);
}

/** REFAIRE
 * @brief 
 *  A - compute keypoints and descriptors
 *  B - match them
 *  C - filter only the good match
 *  D - put in arrays q_i: coordinates vector of the matching keypoints on image i
 * @param image_index : index of the current image
 * @return q_current : vector of points representing the coordinates of the keypoints in the current image (image ith)
 * @return q_previous : vector of points representing the coordinates of the matching keypoints in the previous image (image (i-1)th)
 */
int VisualOdometry_monocular::extract_and_matche_features(int image_index, 
                                                          cv::Mat& current_descriptors, 
                                                          cv::Mat& previous_descriptors, 
                                                          std::vector<cv::KeyPoint>& current_keypoints,  
                                                          std::vector<cv::KeyPoint>& previous_keypoints,
                                                          std::vector<cv::Point2f>& q_current,
                                                          std::vector<cv::Point2f>& q_previous) 
{
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


    return(0);
}

int VisualOdometry_monocular::triangulate(cv::Mat& P_current,
                                          cv::Mat& P_previous,
                                          std::vector<cv::Point2f>& q_current,
                                          std::vector<cv::Point2f>& q_previous,
                                          std::vector<cv::Point3f>& points3D)
{
    // Triangulate points
    //points4D.release(); // output of the function  // FIND better name
    cv::Mat points4D;

    cv::triangulatePoints(P_previous, 
                          P_current, 
                          q_previous, 
                          q_current,
                          points4D);

    // set the 4D points to 3D
    cv::Mat X = points4D.row(0);
    cv::Mat Y = points4D.row(1);
    cv::Mat Z = points4D.row(2);
    cv::Mat W = points4D.row(3);    

    // Diviser Xn, Yn, Zn par Wn
    cv::Mat Xn = X / W;
    cv::Mat Yn = Y / W;
    cv::Mat Zn = Z / W;

    // std::vector<cv::Point3f> points3D;
    points3D.clear();
    points3D.reserve(points4D.cols); // Reserve space for points
    // Fill points3D with the converted coordinates
    for (int i = 0; i < points4D.cols; ++i) 
    {
        points3D.emplace_back(Xn.at<float>(i), Yn.at<float>(i), Zn.at<float>(i));
    }

    return(0);
}

cv::Mat VisualOdometry_monocular::find_Rti(std::vector<cv::Point2f>& q_current,
                                        std::vector<cv::Point3f>& points3D,
                                        cv::Mat& Rti)
{
    std::cout << "debut find rti" << std::endl;
    // Coefficients de distorsion (ici on suppose qu'il n'y en a pas)
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

    // Paramètres pour solvePnPRansac
    int iterationsCount = 100;      // Nombre d'itérations
    float reprojectionError = 8.0;  // Tolérance d'erreur de reprojection en pixels
    double confidence = 0.99;       // Confiance de l'algorithme RANSAC
    cv::Mat inliers;                // Matrice qui stockera les inliers

    cv::Mat Rvec, tvec;
    // bool success = cv::solvePnPRansac(points3D, 
    //                                   q_current, 
    //                                   intrinsic_matrix, 
    //                                   distCoeffs, 
    //                                   Rvec, 
    //                                   tvec, 
    //                                   false, 
    //                                   iterationsCount, 
    //                                   reprojectionError, 
    //                                   confidence, 
    //                                   inliers);

    cv::solvePnP(points3D, q_current, intrinsic_matrix, cv::Mat(), Rvec, tvec);

    std::cout << "DING,\n\n" << std::endl;
    // ====================
    
    cv::Mat Ri;
    Rti.release();
    cv::Rodrigues(Rvec, Ri);

    // v A ENLEVER
    cv::Mat Ti = fuse_R_t(Ri, tvec);

    // v1
    cv::hconcat(Ri, tvec, Rti); 
    Rti.convertTo(Rti, CV_32F); // Convert to from double to float
    
    // std::cout << "Rti " << Rti.size()<< std::endl;
    std::cout << "fin find rti" << std::endl;

    // CA AUSSI
    return(Ti);
}

/**
 * @brief furse the Matrice R and T into T, the homogenous matrice
 * @param R Rotation matrice
 * @param t Translation vector
 * @return T the homogenous matrice contaning [R T]
 */
cv::Mat VisualOdometry_monocular::fuse_R_t(cv::Mat R, cv::Mat t)
{
    cv::Mat T = cv::Mat::eye(4, 4, CV_32F); // Initialiser à une matrice identité 4x4
    R.copyTo(T(cv::Rect(0, 0, 3, 3))); // Copy R in the 3 first lines and columns of M
    t.copyTo(T(cv::Rect(3, 0, 1, 3))); // Copy t dans in the last column of M

    return(T);
}

void VisualOdometry_monocular::main_2D_to_2D()
{
    std::cout << "Visual odometry monocular: 2D to 2D" << std::endl;
    // 1 - get images et setup calibrations 
    get_images(data_directory);
    get_calibration(data_directory);
    
    cv::Mat Ti = cv::Mat::zeros(4, 4, CV_32F); // Transform from image i-1 and i
    cv::Mat Ci = cv::Mat::eye(4, 4, CV_32F); // current pose
    Ci = get_first_pose(data_directory);

    cv::Mat current_descriptors;
    cv::Mat previous_descriptors;

    std::vector<cv::KeyPoint> current_keypoints; 
    std::vector<cv::KeyPoint> previous_keypoints; //    also create a pointer to change ref reasily ? 

    std::vector<cv::Point2f> q_current;
    std::vector<cv::Point2f> q_previous;

    cv::Mat Ri, ti; 

    // write first the first pose
    write_pose("poses/my_poses_seq2_new.txt", Ci);

    // Initiate current keypoints and descriptors --> the loop start from iamge 1 and re use the previous keypoints and descriptors 
    orb->detectAndCompute(images[0], cv::noArray(), current_keypoints, current_descriptors);

    for(size_t image_index = 1; image_index < images.size(); image_index++)
    {          
        // 2 - extract and matche feature between Ik-1 and Ik || Todo, only extract one image at the same time (put old image OR feature in a pointer variable )
        extract_and_matche_features((int) image_index, 
                                    current_descriptors, 
                                    previous_descriptors, 
                                    current_keypoints,  
                                    previous_keypoints,
                                    q_current,
                                    q_previous);
        
        // 3 - Compute essential matrix for image pair Ik-1 and Ik 
        // 4 - Decompose essential matrix into Rk and tk, and from Tk
        // 5 - Compute relatice scale and resacle tk accordingly // fait dans Recoverpose
        motion_estimation(q_current, 
                          q_previous,
                          Ri, 
                          ti);

        cv::Mat Ti = fuse_R_t(Ri, ti);
        // 6 - Concatenate transformation by computing Ck = Ck-1 Tk
        //Ci = Ci.mul(Ti);//...(C x Ti)
        Ci = Ci * Ti.inv();

        // std::cout << "Ti  :\n" << Ti <<std::endl;
        // std::cout << " Ci :\n" << Ci <<std::endl;
        write_pose("poses/my_poses_seq2_new.txt", Ci);
        std::cout << image_index <<std::endl;
    }
    std::cout << "Over" <<std::endl;
}

void VisualOdometry_monocular::main_3D_to_2D()
{
    std::cout << "Visual odometry monocular: 2D to 2D" << std::endl;
    // 1 - get images et setup calibrations 
    get_images(data_directory);
    get_calibration(data_directory);
    
    cv::Mat Ti = cv::Mat::zeros(4, 4, CV_32F); // Transform from image i-1 and i
    cv::Mat C0 = cv::Mat::eye(4, 4, CV_32F); // current pose
    C0 = get_first_pose(data_directory);

    cv::Mat current_descriptors;
    cv::Mat previous_descriptors;

    std::vector<cv::KeyPoint> current_keypoints; 
    std::vector<cv::KeyPoint> previous_keypoints; //    also create a pointer to change ref reasily ? 

    std::vector<cv::Point2f> q_current;
    std::vector<cv::Point2f> q_previous;

    cv::Mat Ri, ti, Rti;

    // write first the first pose
    write_pose("poses/mono_3D_2D_my_poses_seq2.txt", C0);

    // 1.2 Extract and match features
    // get find q_previous and q_current: the 2D points in the image
    orb->detectAndCompute(images[0], cv::noArray(), current_keypoints, current_descriptors); // extract img1
    extract_and_matche_features(1,  // extract img2 and matche f1 & f2
                                current_descriptors, 
                                previous_descriptors, 
                                current_keypoints,  
                                previous_keypoints,
                                q_current,
                                q_previous);
                                                            
    // 1.3  
    // projection Matrice for each iamges
    cv::Mat P_previous, P_current; 
    // turn the first 3 lines and 4th column of Ci : 4x4 -> 3x4
    P_previous = intrinsic_matrix * C0(cv::Range(0, 3), cv::Range::all());

    // CHANGER UNE FOIS QUE CA MARCHEN MOTION ESTIMATION DOIT PRENDRE Ri ti EN sortit
    motion_estimation(q_current, 
                      q_previous,
                      Ri, 
                      ti);

    
    
    cv::hconcat(Ri, ti, Rti); 
    Rti.convertTo(Rti, CV_32F); // Convert to from double to float

    std::cout <<"INI\n";
    std::cout <<"RTI\n" << Rti;

    P_current = intrinsic_matrix * Rti;

    // std::cout<< P_current << std::endl;

    // Triangulate points
    std::vector<cv::Point3f> points3D; // output of the function  // FIND better name
    triangulate(P_current,
                P_previous,
                q_current,
                q_previous,
                points3D);


    for(size_t image_index = 1; image_index < images.size(); image_index++)
    {          
        // 2.2 - extract and matche feature between Ik-1 and Ik
        extract_and_matche_features((int) image_index, 
                                    current_descriptors, 
                                    previous_descriptors, 
                                    current_keypoints,  
                                    previous_keypoints,
                                    q_current,
                                    q_previous);
        


        triangulate(P_current,
                    P_previous,
                    q_current,
                    q_previous,
                    points3D);

        std::cout << "FIN BOUBLE "<< image_index << std::endl;

        // 2.3 PNP ransac
    
        cv::Mat Ti = find_Rti(q_current,
                                points3D,
                                Rti);

        C0 = C0 * Ti.inv();

        P_previous = P_current.clone(); 
        
        std::cout << "int mat" << std::endl;
        P_current = intrinsic_matrix * Rti; // Matrice de projection = K * [R|t] // il semblemrait que ça soit juste, cependant, je pense que 
        
        
        write_pose("poses/mono_3D_2D_my_poses_seq2.txt", C0);// ici ce n'est pas bon, c'est pas p current non plus je pense 
        std::cout << image_index <<std::endl;
    }
    std::cout << "Over" <<std::endl;

}

/**
 * @brief get images from dataset
 * @param folder_path path of the images
 * @return images a vector or matrice. Each elecment of the vector is an image
 */
int VisualOdometry_monocular::get_images(const std::string& folder_path)
{
    std::string images_folder_path = folder_path + "/image_l/*.png";
    std::vector<cv::String> files_name;
    cv::glob(images_folder_path, files_name, false);
    for (size_t i=0; i<files_name.size(); i++)
        images.push_back(cv::imread(files_name[i]));
    return(0);
}

/**
 * @brief Read calib.txt file and get the intrinsict and projection matrix 
 * @param folder_path : path of the file containning the calibration
 * @return intrinsic_matrix 
 * @return projection_matrix
 */
int VisualOdometry_monocular::get_calibration(const std::string& folder_path)
{
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

/**
 * @brief get the first pose of the dataset form poses.txt
 * @param folder_path : path of the file containning the ground truth poses
 * @return C0, the first pose
 */
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

/**
 * @brief write poses in a txt file
 * @param folder_path : path to write the poses
 * @param poses : matrice containing all the computer poses
 */
int VisualOdometry_monocular::write_pose(const std::string& folder_path, const cv::Mat& poses)
{
    std::string poses_path = folder_path;// + "my_poses.txt";
    std::ofstream pose_file(poses_path, std::ios::app);
    if(pose_file.is_open()) 
    {
        for(int i = 0; i < poses.rows; i++)
        {
            for(int j = 0; j < poses.cols ; j++)
            {
                pose_file << poses.at<float>(i,j) << " ";
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
    VO.main_3D_to_2D();
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
