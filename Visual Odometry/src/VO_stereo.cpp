#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include "VO_stereo.hpp"

// Constructor
VisualOdometry_stereo::VisualOdometry_stereo()
{
    std::cout<<"Nothing specified\n";
}

/**
 * @brief Constructor of the class
 * @param new_data_directory: dataset directory
 * @param new_output_poses: file to write the computed poses
 */
VisualOdometry_stereo::VisualOdometry_stereo(std::string new_data_directory,
                                             std::string new_output_poses)
{
    _data_directory = new_data_directory;
    _output_poses = new_output_poses;
}

VisualOdometry_stereo::~VisualOdometry_stereo()
{
    // Destructor
}


int VisualOdometry_stereo::main()
{
    std::cout << "Visual odometry stereo" << std::endl;

    // Initial variables

    std::vector<cv::Mat> images_left, images_right;
    
    cv::Mat intrinsic_matrix_left, intrinsic_matrix_right;

    //cv::Mat projection_matrix_left_old, projection_matrix_left_new, projection_matrix_right_old, projection_matrix_right_new;
    // on verra ce qu'on fait de ça 
    cv::Mat projection_matrix_left_0, projection_matrix_right_0;

    std::vector<cv::KeyPoint> kp_left_old, kp_right_old, kp_left_new, kp_right_new;
    cv::Mat desc_left_old, desc_right_old, desc_left_new, desc_right_new;

    std::vector<std::vector<cv::DMatch>> matches_old, matches_new;
    
    std::vector<cv::DMatch> good_matches_old, good_matches_new;
    
    std::vector<cv::Point2f> pts_left_old, pts_right_old, pts_left_new, pts_right_new;

    std::vector<cv::Point2f> matching_points_left_old, matching_points_right_old, matching_points_left_new, matching_points_right_new;
    cv::Mat matching_desc_left_old, matching_desc_right_old, matching_desc_left_new, matching_desc_right_new;


    get_images(_data_directory, images_left, images_right);

    get_calibration(_data_directory,
                    intrinsic_matrix_left,
                    intrinsic_matrix_right,
                    projection_matrix_left_0,
                    projection_matrix_right_0);

    cv::Mat Ti = cv::Mat::zeros(4, 4, CV_32F); // Transform from image i-1 and i
    cv::Mat Ci = cv::Mat::eye(4, 4, CV_32F); // current pose
    Ci = get_first_pose(_data_directory);

    // write first the first pose
    write_pose(_output_poses, Ci);

    // Initiate current keypoints and descriptors --> the loop start from iamge 1 and re use the previous keypoints and descriptors 
    // extract keypoints
    std::cout << 1 << std::endl;
    _orb->detectAndCompute(images_left[0], cv::noArray(), kp_left_new, desc_left_new);
    _orb->detectAndCompute(images_right[0], cv::noArray(), kp_right_new, desc_right_new);
    
    // std::cout << 2 << std::endl;
    // match
    _flann->knnMatch(desc_left_new, desc_right_new, matches_new, 2); 

    // find matches
    filter_good_matches(matches_new, 0.5, good_matches_new);  

    // Filter function uses 2d poitns and not keypoints
    cv::KeyPoint::convert(kp_left_new, pts_left_new);
    cv::KeyPoint::convert(kp_right_new, pts_right_new);

    // filter the keypoints and descriptors to only have the one that matche
    filter_matching_points(good_matches_new,
                           pts_left_new,
                           pts_right_new,
                           desc_left_new,
                           desc_right_new,
                           matching_points_left_new,
                           matching_points_right_new,
                           matching_desc_left_new,
                           matching_desc_right_new); 

    std::vector<cv::Point3f> points3D_old, points3D_new;

    // std::cout << projection_matrix_left_0 << std::endl;
    // std::cout << projection_matrix_right_0 << std::endl;

    find_3Dpoints(projection_matrix_left_0, 
                    projection_matrix_right_0,
                    matching_points_left_new, 
                    matching_points_right_new,
                    points3D_new);

    if(images_left.size() != images_right.size()) // check if there are the same numbers of images for left and right cameras
        return(-1);


    std::vector<std::vector<cv::Point3f>> points_array;
    for(size_t image_index = 1; image_index < images_left.size(); image_index++)
    {

        // update old variables before updating new varaibles

        kp_left_old = kp_left_new;
        kp_right_old = kp_right_new;

        pts_left_old = pts_left_new;
        pts_left_old = pts_right_new;

        // desc_left_old = desc_left_new.clone();
        // desc_right_old = desc_right_new.clone();

        matches_old = matches_new;
        good_matches_old = good_matches_new;
        

        points3D_old = points3D_new;
        points3D_new.clear();

        matching_points_left_old = matching_points_left_new;
        matching_points_right_old = matching_points_right_new;

        matching_desc_left_old = matching_desc_left_new.clone();
        matching_desc_right_old = matching_desc_right_new.clone(); 
    
        // 1) Capture two stereo image pairs Il;k1, Ir;k1 and Il;k, Ir;k
        _orb->detectAndCompute(images_left[image_index], cv::noArray(), kp_left_new, desc_left_new);
        _orb->detectAndCompute(images_right[image_index], cv::noArray(), kp_right_new, desc_right_new);

        // 2) Extract and match features between left and right
        matches_new.clear();
        _flann->knnMatch(desc_left_new, desc_right_new, matches_new, 2); 
        int ya;
        // std::cin >> ya;
        good_matches_new.clear();   
        filter_good_matches(matches_new, 0.5, good_matches_new);
        
        // Filter function uses 2d poitns and not keypoints
        cv::KeyPoint::convert(kp_left_new, pts_left_new);
        cv::KeyPoint::convert(kp_right_new, pts_right_new);


        // filter_matching_points(good_matches_old,
        //                        pts_left_old,
        //                        pts_right_old,
        //                        desc_left_old,
        //                        desc_right_old,
        //                        matching_points_left_old,
        //                        matching_points_right_old,
        //                        matching_desc_left_old,
        //                        matching_desc_right_old); // same 
                               
        // keep only the matching points
        filter_matching_points(good_matches_new,
                               pts_left_new,
                               pts_right_new,
                               desc_left_new,
                               desc_right_new,
                               matching_points_left_new,
                               matching_points_right_new,
                               matching_desc_left_new,
                               matching_desc_right_new); // faut que ça renvoie aussi les decriptros 


        ////
        // test - dans un premier temps on va faire le mathc des points 2D left uniquement, on peut améliorer ça en faisant matché tout 
        
        std::cout << "\n\n";
          std::cout << matching_points_right_new.size()<< " - " << matching_points_left_new.size()  << " - " << good_matches_new.size()<< "\n";

        // matche pas
        //show_matches(images_left[image_index], images_right[image_index], matching_points_left_new, matching_points_right_new, good_matches_new);
        
        // puise qu'on utliser match, on doit mettre en entré les point avant filtrage !!!


        std::vector<cv::DMatch> hihi; //
        
        if(matching_points_left_new.size() == matching_points_right_new.size())
        {
            for(size_t i = 0; i < matching_points_left_new.size(); i++)
            {
                cv::DMatch match_i;
                match_i.trainIdx = i;
                match_i.queryIdx = i;
                // match_i.distance = 10;
                hihi.push_back(match_i);

            }
        }


        // show_matches_points(images_left[image_index], images_right[image_index], pts_left_new, pts_right_new, good_matches_new);
        // show_matches_points(images_left[image_index], images_right[image_index], matching_points_left_new, matching_points_right_new, hihi);
        
        std::cout << "\fin\n";

        std::cout << projection_matrix_left_0 << std::endl;
        std::cout << projection_matrix_right_0 << std::endl;

        find_3Dpoints(projection_matrix_left_0, 
                      projection_matrix_right_0,
                      matching_points_left_new,
                      matching_points_right_new,
                      points3D_new);

        

        // filter (put in a function)
        std::vector<std::vector<cv::DMatch>>  matches_left_both;
        
        _flann->knnMatch(matching_desc_left_old, matching_desc_left_new, matches_left_both, 2);

        std::vector<cv::DMatch> good_matches_left_both;

        // std::cin >> ya;
        filter_good_matches(matches_left_both, 0.5, good_matches_left_both);

        std::vector<cv::Point3f> points3D_old_filtered, points3D_new_filtered;
        for(const auto& good_l : good_matches_left_both) 
        {
            points3D_old_filtered.push_back(points3D_old[good_l.queryIdx]); // ATTENTION ON OEUT PAS FAIRE COMME ÇA ici
            points3D_new_filtered.push_back(points3D_new[good_l.trainIdx]);
            // matching_desci_01.push_back(desci_0.row(good.trainIdx));   
        }

        // show_matches(images_left[image_index-1], images_left[image_index], matching_points_left_old, matching_points_left_new, good_matches_left_both);
        // nvm ça math

        // ================
        // ==== 3D VIZ ====
        // ================

        // passer à open3D en python ? -> on cherche à capter la prjections des point 3D pour l'index i
        // write_pose(" step_i")
        std::string points3D_file = "3Dpoints/test_" + std::to_string(image_index); + ".txt"; 
        // ATTENTION IL Y A DES NOMBRE ABÉRAMENT GRAND !! // car des nobmre dsont en 32 et d'autre en 64bit ? 
        write_3Dpoints(points3D_file, points3D_old);
        points_array.push_back(points3D_new);
        std::cout<< "size of 3D points " << points_array.size() << std::endl;
        std::cout<< "size of 3D points of the current one " << points_array[image_index-1].size() << std::endl;
        
        // int lim = 8;

        if(image_index > 7 && false)
        {
            cv::viz::Viz3d window("pcl");
            int v = image_index-5;
            std::cout << " pcl " << std::endl;

            // créé/ajoute un nuage de points
            cv::viz::WCloud cloud1(points_array[v], cv::viz::Color::yellow());
            // cv::viz::WCloud cloud2(points_array[v+1], cv::viz::Color::orange());
            // cv::viz::WCloud cloud3(points_array[v+2], cv::viz::Color::red());
            // cv::viz::WCloud cloud4(points_array[v+3], cv::viz::Color::blue());
            // cv::viz::WCloud cloud5(points_array[v+4], cv::viz::Color::green());

            window.showWidget("Cloud1", cloud1);
            // window.showWidget("Cloud2", cloud2);
            // window.showWidget("Cloud3", cloud3);
            // window.showWidget("Cloud4", cloud4);
            // window.showWidget("Cloud5", cloud5);

            window.spin();
        }

        // time for points arrays
        

        // // faire comm en anglais: on va faire le amtching qu'entre les point Gauche et supposer que c'est bon car on a déjà matché les point g et droite avant: update -> match g old g new, match d d -> on filtre tout les points qui ne match pas (même ordre du coup simple à filtrer)
        
        // std::vector<std::vector<cv::DMatch>>  matches_left_both, matches_right_both;
        // _flann->knnMatch(matching_desc_left_old, matching_desc_left_new, matches_left_both, 2);
        // _flann->knnMatch(matching_desc_right_old, matching_desc_right_new, matches_left_both, 2);

        // std::vector<cv::DMatch> good_matches_left_both, good_matches_right_both;
        // filter_good_matches(matches_left_both, 0.5, good_matches_left_both);
        // filter_good_matches(matches_right_both, 0.5, good_matches_right_both);

        
        // // find a better name 
        // std::vector<cv::Point2f> matching_points_left_old_mieux, matching_points_right_old_mieux, matching_points_left_new_mieux, matching_points_right_new_mieux;

        // filter_matching_points(good_matches_left_both,
        //                        matching_points_left_old,
        //                        matching_points_left_new,
        //                        matching_points_left_old_mieux,
        //                        matching_points_left_new_mieux); 

        // // on re filtre ici sans avoir besoin de mathc car les points sont dans le même ordre ? // pour vérifier on prend une image et on color les 10er points
        // filter_matching_points(good_matches_right_both,
        //                        matching_points_right_old,
        //                        matching_points_right_new,
        //                        matching_points_right_old_mieux,
        //                        matching_points_right_new_mieux); 

        // // std::cout << 5 << std::endl;

        // // 3) Triangulate matched features for each stereo pair



        // //MAYBE NOT HERE / peut être plus besoin
        // // projection_matrix_left_old = projection_matrix_left_new.clone();
        // // projection_matrix_right_old = projection_matrix_right_new.clone();

        // // projection_matrix_left_new = ...;
        // // projection_matrix_right_new = ...;


        // // std::vector<cv::Point3f> points3D_left, points3D_right;

        // // std::cout << projection_matrix_right_old << std::endl;

        // //points3D

        // // solve PNP ? d'abor dpour cococ la position into on triangulate pour l'étape d'après ? 
        
        // std::vector<cv::Point3f> points3D_old, points3D_new;

        // // std::cout << projection_matrix_left_0 << std::endl;
        // // std::cout << projection_matrix_right_0 << std::endl;

        // find_3Dpoints(projection_matrix_left_0, 
        //               projection_matrix_right_0,
        //               matching_points_left_old_mieux, 
        //               matching_points_right_old_mieux,
        //               points3D_old);
        
        // std::cout << "2eme 3D points find" << std::endl;
        // find_3Dpoints(projection_matrix_left_0, 
        //               projection_matrix_right_0,
        //               matching_points_left_new_mieux,
        //               matching_points_right_new_mieux,
        //               points3D_new);

        // // std::cout << 6 << std::endl;



        // // 4) Compute Tk from 3-D features Xk1 and Xk
        cv::Mat inliers;
        cv::Mat Rti;
        // std::cout << points3D_old_filtered.size() << std::endl;

        // std::cout << points3D_new_filtered.size() << std::endl;
        

        cv::estimateAffine3D(points3D_old_filtered, points3D_new_filtered, Rti, inliers);
        // std::cout << 7 << std::endl;
        // std::cout << Ti <<std::endl;
        cv::Mat Ti = to_homogenous(Rti);
        // de toute evidence notre Ti est faux, qu'en es il du reste ? 
        // on peut afficher les points 3d et voir ? 
        // fautdrait il matcher deux fois (on le fait qu'une fois) pour être certian d'avoir les bon point
        // on pourrait en profiter pour clean et pas faire tout deux fois 
        // essayer sove pnp plutôt que estimateAffine3D

        // std::cout <<"voici Ti\n" << inliers << std::endl; 

        std::cout <<"voici Ti à la case " << image_index << "\n" << Ti << std::endl; 
    
        // 5) Concatenate transformation by computing
        Ci = Ci * Ti.inv();
        // 6) Repeat from 1).
        write_pose(_output_poses, Ci);
    }

    return(0);
}


/**
 * @brief filter only the good match using "..."'s threshold  
 * @param matches: vector containinng all the matches
 * @param ratio_thresh:  0=<value <=1; in order to sleect the best from a certain level(make it better)
 * @return good_matches: vector containing all the good matches 
 */
int VisualOdometry_stereo::filter_good_matches(const std::vector<std::vector<cv::DMatch>> matches, 
                                               const float ratio_thresh, 
                                               std::vector<cv::DMatch>& good_matches)
{
    for (size_t i = 0; i < matches.size(); i++) 
    {
        if(matches[i].empty()) // check if the ith cell of matches is empty, if it is we drop it to avoid a segfault, otherwise we continue 
        {
            std::cout << "Error : cell " << i << " is empty." << std::endl; // before we checked with images index
        }
        else if(matches[i].size() != 2 )
        {
            std::cout << "Error : cell " << i << " images of size " << matches[i].size()  << " instead of 2" << std::endl;
        
        }
        else if (matches[i][0].distance < (ratio_thresh * matches[i][1].distance))
        {
            good_matches.push_back(matches[i][0]);
        }
    }
    return(0);
}


/**
 * @brief: use matches to take off from descriptors and 2D points the non matching points and return the matching ones in the same order. The returned matching points and descriptors are in the same order => matching_points 1 and 2 of [i] AND matching descriptors 1 and 2 of [i] -> same point
 * @param points1:  vector contaning image 1's points
 * @param points2:  vector contaning image 2's points
 * @param desc1: matrice of descriptors of points from image 1 
 * @param desc2: matrice of descriptors of points from image 2
 * @return matching_points1: vector of 2D points that match with image 2's points
 * @return matching_points2: vector of 2D points that match with image 1's points
 */
int VisualOdometry_stereo::filter_matching_points(std::vector<cv::DMatch> good_matches, // changer cette fonction 
                                                  std::vector<cv::Point2f> points1, // doit prendre en entré lesp oints 2Df après 
                                                  std::vector<cv::Point2f> points2,
                                                  std::vector<cv::Point2f>& matching_points1,
                                                  std::vector<cv::Point2f>& matching_points2)
{
    matching_points1.clear();
    matching_points2.clear();
    // matching_descriptors2.release();
    
    for(const auto& good : good_matches) 
    {
        matching_points1.push_back(points1[good.queryIdx]); // maybe not .pt
        matching_points2.push_back(points2[good.trainIdx]);
        // matching_desci_01.push_back(desci_0.row(good.trainIdx));   
    }
    return(0);
}

/**
 * @brief: use matches to take off from descriptors and 2D points the non matching points and return the matching ones in the same order. The returned matching points and descriptors are in the same order => matching_points 1 and 2 of [i] AND matching descriptors 1 and 2 of [i] -> same point
 * @param points1:  vector contaning image 1's points
 * @param points2:  vector contaning image 2's points
 * @param desc1: matrice of descriptors of points from image 1 
 * @param desc2: matrice of descriptors of points from image 2
 * @return matching_points1: vector of 2D points that match with image 2's points
 * @return matching_points2: vector of 2D points that match with image 1's points
 * @return matching_descriptors1: matrice of matching descriptors of image 1 to image 2
 * @return matching_descriptors2: matrice of matching descriptors of image 2 to image 1
 */
int VisualOdometry_stereo::filter_matching_points(std::vector<cv::DMatch> good_matches,
                                                  std::vector<cv::Point2f> points1,  
                                                  std::vector<cv::Point2f> points2,
                                                  cv::Mat desc1,
                                                  cv::Mat desc2,
                                                  std::vector<cv::Point2f>& matching_points1,
                                                  std::vector<cv::Point2f>& matching_points2,
                                                  cv::Mat& matching_descriptors1,
                                                  cv::Mat& matching_descriptors2)
{
    matching_points1.clear();
    matching_points2.clear();
    matching_descriptors1.release();
    matching_descriptors2.release();
    
    for(const auto& good : good_matches) 
    {
        matching_points1.push_back(points1[good.queryIdx]); 
        matching_points2.push_back(points2[good.trainIdx]);
        matching_descriptors1.push_back(desc1.row(good.queryIdx));  
        matching_descriptors2.push_back(desc2.row(good.trainIdx));   
    }
    return(0);
}

/**
 * @brief find the 3D points using trinagulate points function and convert them to homogenous coordinates
 * @param projection_matrix1: projection matrix of image 1
 * @param projection_matrix2: projection matrix of image 2
 * @param matching_points1: matching points from image 1 with image 2
 * @param matching_points2: matching points from image 2 with image 1
 * @return points3D: vector with the 3D coordinates of the matching points of image 1 and 2
 */
int VisualOdometry_stereo::find_3Dpoints(cv::Mat& projection_matrix1,
                                         cv::Mat& projection_matrix2,
                                         std::vector<cv::Point2f> matching_points1,
                                         std::vector<cv::Point2f> matching_points2,
                                         std::vector<cv::Point3f>& points3D)
{
    // cv::Mat points4D;
    cv::Mat points4D = cv::Mat::zeros(4, 3, CV_32F);

    // std::cout << 5.1 << std::endl;

    // std::cout << "projection_matrix1\n" << projection_matrix1 << std::endl;
    // std::cout << "projection_matrix2\n" << projection_matrix2 << std::endl;
    // std::cout << "matching_points1\n" << matching_points1 << std::endl;
    // std::cout << "matching_points2\n" << matching_points2 << std::endl;

    cv::triangulatePoints(projection_matrix1, // oldest camera
                          projection_matrix2, // newest caemra
                          matching_points1, 
                          matching_points2,
                          points4D);
    
    
    // ATTENTION OMEGA GIGA TATTNEITON certain de mes W sont négatifs !!!!


    std::cout << "points4D\n" << points4D.col(1) << std::endl;
    // std::cout << points4D.cols(0)<< std::endl;

    // cv::convertPointsFromHomogeneous ?

    // std::cout << 5.2 << std::endl;

    // set the 4D points to 3D // put this into a fucntion ? 
    cv::Mat X = points4D.row(0);
    cv::Mat Y = points4D.row(1);
    cv::Mat Z = points4D.row(2);
    cv::Mat W = cv::abs(points4D.row(3));     // TEST VALEUR ABSOLUE

    // Divede Xn, Yn, Zn by Wn
    cv::Mat Xn = X / W;
    cv::Mat Yn = Y / W;
    cv::Mat Zn = Z / W;

    // std::cout << 5.3 << std::endl;
    // std::vector<cv::Point3f> points3D;
    points3D.clear();
    points3D.reserve(points4D.cols); // Reserve space for points
    // Fill points3D with the converted coordinates
    for (int i = 0; i < points4D.cols; ++i) 
    {
        points3D.emplace_back(Xn.at<float>(i), 
                              Yn.at<float>(i), 
                              Zn.at<float>(i)); // AUTRE MANIERE CHEC FAST
    }

    std::cout << "points3D\n" << points3D[1] << std::endl;
    return(0);
}


/**
 * @brief: 
 * @param Rt: matrice 3x4 containing [R|t] rotation matrix and translation vector
 * @return T: the 4x4 transformation matrice
*/
cv::Mat VisualOdometry_stereo::to_homogenous(cv::Mat Rt)
{
    cv::Mat T = cv::Mat::eye(4, 4, CV_32F);
    Rt.copyTo(T(cv::Rect(0,0,4,3)));
    return(T);
}


/**
 * @brief convert points1 and 2 from points2f to keypoints and call show_matches
 * @param image1: 
 * @param image2: 
 * @param points1: vector contaning image 1's points
 * @param points1: vector contaning image 2's points
 * @param matches: vector containing matches between image 1 and 2
 */
int VisualOdometry_stereo::show_matches_points(cv::Mat image1, 
                                                cv::Mat image2,
                                                std::vector<cv::Point2f> points1,
                                                std::vector<cv::Point2f> points2,
                                                std::vector<cv::DMatch> matches)
{        
    std::vector<cv::KeyPoint> keypoints1, keypoints2;

    for (const auto& pt : points1) 
    {
        cv::KeyPoint kp(pt, 10.0f); // Taille du keypoint = 10.0f (modifiable)
        keypoints1.push_back(kp);
    }
    
    for (const auto& pt : points2) 
    {
        cv::KeyPoint kp(pt, 10.0f); // Taille du keypoint = 10.0f (modifiable)
        keypoints2.push_back(kp);
    }
    std::cout << matches.size() << "v2";

    show_matches(image1, image2, keypoints1, keypoints2, matches);
    
    return(0);
}

/** 
 * @brief: show two images, print keypoints and draw matches
 * @param image1: 
 * @param image2: 
 * @param keypoint1: vector contaning image 1's keypoints
 * @param keypoint2: vector contaning image 2's keypoints
 * @param matches: vector containing matches between image 1 and 2
 */

int VisualOdometry_stereo::show_matches(cv::Mat image1, 
                                        cv::Mat image2,
                                        std::vector<cv::KeyPoint> keypoints1,
                                        std::vector<cv::KeyPoint> keypoints2,
                                        std::vector<cv::DMatch> matches)
{        
    cv::Mat img_matches;

    cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, img_matches, 
                    cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    // Redimension the image to fit screen
    double scale_factor = 0.7; // Facteur de redimensionnement (ajustez en fonction de la taille de vos images)
    cv::resize(img_matches, img_matches, cv::Size(), scale_factor, scale_factor);

    // draw good corespondances
    cv::imshow("Good Matches", img_matches);
    cv::waitKey(0); 
    return(0);
}

/**
 * @brief get images from dataset
 * @param folder_path: path of the images
 * @return images_left: vector of matrice. Each elecment of the vector is an image if the left camera
 * @return images_right: vector of matrice. Each elecment of the vector is an image if the right camera
 */
int VisualOdometry_stereo::get_images(const std::string& folder_path,
                                      std::vector<cv::Mat>& images_left,
                                      std::vector<cv::Mat>& images_right)
{
    std::string images_folder_path_left = folder_path + "/image_l/*.png";
    std::string images_folder_path_right = folder_path + "/image_r/*.png";

    std::vector<cv::String> files_name;
    
    cv::glob(images_folder_path_left, files_name, false);
    for (size_t i=0; i<files_name.size(); i++)
        images_left.push_back(cv::imread(files_name[i]));

    files_name.clear();
    cv::glob(images_folder_path_right, files_name, false);
    for (size_t i=0; i<files_name.size(); i++)
        images_right.push_back(cv::imread(files_name[i]));

    return(0);
}

/**
 * @brief Read calib.txt file and get the intrinsict and projection matrix 
 * @param folder_path : path of the file containning the calibration
 * @return intrinsic_matrix_left: intrinsic matrice of the left camera
 * @return intrinsic_matrix_right: intrinsic matrice of the right camera
 * @return projection_matrix_left: projection matric of the left camera
 * @return projection_matrix_right: projection matric of the left camera
 */
int VisualOdometry_stereo::get_calibration(const std::string& folder_path,
                                              cv::Mat& intrinsic_matrix_left,
                                              cv::Mat& intrinsic_matrix_right,
                                              cv::Mat& projection_matrix_left,
                                              cv::Mat& projection_matrix_right)
{
    intrinsic_matrix_left = cv::Mat::zeros(3, 3, CV_32F);
    intrinsic_matrix_right = cv::Mat::zeros(3, 3, CV_32F);
    projection_matrix_left = cv::Mat::zeros(3, 4, CV_32F);
    projection_matrix_right = cv::Mat::zeros(3, 4, CV_32F);

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
                intrinsic_matrix_left.at<float>(i, j) = value;
            }
            projection_matrix_left.at<float>(i, j) = value;
        }
    }

    // Read calibration data for the right camera (second line)
    for (int i = 0; i < 3; i++) { 
        for (int j = 0; j < 4; j++) {
            float value;
            file >> value;
            if (j != 3) {
                intrinsic_matrix_right.at<float>(i, j) = value;
            }
            projection_matrix_right.at<float>(i, j) = value;
        }
    }

    return(0);
}

/**
 * @brief get the first pose of the dataset form poses.txt
 * @param folder_path : str containg the poses file
 * @return C0: the first pose
 */
cv::Mat VisualOdometry_stereo::get_first_pose(const std::string& folder_path)
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
 * @brief write poses in a  file
 * @param folder_path: str containg the 3D points file 
 * @param pose: matrice of the prose
 */
int VisualOdometry_stereo::write_pose(const std::string& folder_path, const cv::Mat pose)
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

/**
 * @brief: write 3D points in a file
 * @param points_path: str containg the file of the points
 * @param points3D: vector of 3D points
 */
int VisualOdometry_stereo::write_3Dpoints(const std::string& points_path, const std::vector<cv::Point3f>& points3D)
{
    // std::string points_path = folder_path;// + "my_poses.txt";
    std::ofstream points_file(points_path, std::ios::app);
    if(points_file.is_open()) 
    {
        for (const auto& point : points3D) 
        {
            points_file << point.x << " " << point.y << " " << point.z << "\n";
        }
        points_file << std::endl;
        points_file.close();
        std::cout << "file sucessfully written." << std::endl;
    } 

    else 
    {
        std::cerr << "Error : Cannot open file." << std::endl;
    }

    return(0);
}

/**
 * @brief print matches to show its details
 * @param matches: vector of matches 
 */
int VisualOdometry_stereo::print_matches(const std::vector<cv::DMatch>& matches) 
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
    return(0);
}

/**
 * @brief: Take a vector of vector and call in a for loop the function print matches in order to print the whole matches
 * @param matches_arrayvector: 2D vector of matches 
 */
int VisualOdometry_stereo::print_matches_array(const std::vector<std::vector<cv::DMatch>>& matches_array) 
{
    for (size_t i = 0; i < matches_array.size(); i++) 
    {
        std::cout << "Match array" << i << ":\n";
        print_matches(matches_array[i]);
    }
    return(0);
}

int main()
{
    VisualOdometry_stereo VO_stereo = VisualOdometry_stereo("example/KITTI_sequence_1", 
                                                               "yo.txt");
    VO_stereo.main();

    return(0);
}




/**
 * PROCHIANE FOIS
 * clean
 * commente
 * on check si c'es tle même format partout 
 * on print les points qui matches (pas les kp avec les matches, on s'assure que ce sont les même point !!!!)
 * 
 */