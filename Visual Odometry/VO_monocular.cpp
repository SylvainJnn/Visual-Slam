#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include "include/VO_monocular.hpp"

// using namespace std;


// Constructor
VisualOdometry_monocular::VisualOdometry_monocular()
{
    std::cout<<"Nothing specified\n";
}

VisualOdometry_monocular::VisualOdometry_monocular(std::string new_data_directory)
{
    data_directory = new_data_directory;
}


/**
 * @brief compute the displacement between the current frame (fi) and the previous frame (fi-1)
 * @param matching_kpi_01 contains the keypoints of the frame fi
 * @param matching_kpi_10 contains the keypoints of the frame fi-1
 * @return Ti Matrice that contains homogenous coordinates. It represents the displacement between Ti-1 and Ti
 */
int VisualOdometry_monocular::motion_estimation(std::vector<cv::Point2f>& matching_kpi_01,
                                                std::vector<cv::Point2f>& matching_kpi_10,
                                                cv::Mat& Ri, 
                                                cv::Mat& ti)
{
    // 3 - Compute essential matrix for image pair Ik-1 and Ik 
    cv::Mat essential_matrix, mask_essential_matrix; 
    essential_matrix = cv::findEssentialMat(matching_kpi_10, matching_kpi_01, intrinsic_matrix, cv::RANSAC);

    // 4 - Decompose essential matriintrinsic_matrixce to Ri and ti
    // Attention, several Rotation matrice are possible
    
    cv::recoverPose(essential_matrix, matching_kpi_10, matching_kpi_01, intrinsic_matrix, Ri, ti);

    return(0);
}

/**
 * @brief filter only the good match using "..."'s threshold  
 * @param matches : vector containinng all the matches
 * @param ratio_thresh : 0=<value <=1; in order to sleect the best from a certain level(make it better)
 * @return good_matches : vector containing all the good matches 
 */
int VisualOdometry_monocular::filter_good_matches(const std::vector<std::vector<cv::DMatch>> matches, 
                                                  const float ratio_thresh, 
                                                  std::vector<cv::DMatch>& good_matches)
{
    for (size_t i = 0; i < matches.size(); i++) 
    {
        if(matches[i].empty()) // check if the ith cell of matches is empty, if it is we drop it to avoid a segfault, otherwise we continue 
        {
            std::cout << "Error : cell " << i    << " is empty." << std::endl; // before we checked with images index
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

/** REFAIRE
 * @brief 
 *  A - compute keypoints and descriptors
 *  B - match them
 *  C - filter only the good match
 *  D - put in arrays q_i: coordinates vector of the matching keypoints on image i
 * @param image_index : index of the current image
 * @return matching_kpi_01 : vector of points representing the coordinates of the keypoints in the current image (image ith)
 * @return matching_kpi_10 : vector of points representing the coordinates of the matching keypoints in the previous image (image (i-1)th)
 */
int VisualOdometry_monocular::extract_and_matche_features(int image_index, 
                                                          cv::Mat& desci_0, 
                                                          cv::Mat& desci_1, 
                                                          std::vector<cv::KeyPoint>& kpi_0,  
                                                          std::vector<cv::KeyPoint>& kpi_1,
                                                          std::vector<cv::Point2f>& matching_kpi_01,
                                                          std::vector<cv::Point2f>& matching_kpi_10,
                                                          cv::Mat& matching_desci_01) 
{
    // A - compute keypoints and descriptors
    desci_1 = desci_0.clone();
    kpi_1 = kpi_0;
    orb->detectAndCompute(images[image_index], cv::noArray(), kpi_0, desci_0); // normaleltn c'est update 

    // B - match features
    std::vector<std::vector<cv::DMatch>> matches;
    my_flann->knnMatch(desci_1, desci_0, matches, 2);
    // printMatches(matches);


    // C - filter only the good match

    // V1
    // const float ratio_thresh = 0.5f; // TODO c'est quoi ce ration
    // std::vector<cv::DMatch> good_matches;
    // for (size_t i = 0; i < matches.size(); i++) 
    // {
    //     if(matches[i].empty()) // check if the ith cell of matches is empty, if it is we drop it to avoid a segfault, otherwise we continue // ATTENTION on devrait aussi vérifier si le tuple est bien de longeur 2 et pas autre chsoe
    //     {
    //         std::cout << "Error : cell " << i << " of matches in image " << image_index << " is empty." << std::endl;
    //     }
    //     else if (matches[i][0].distance < (ratio_thresh * matches[i][1].distance))
    //     {
    //         good_matches.push_back(matches[i][0]);
    //     }
    // }

    // V2
    std::vector<cv::DMatch> good_matches;
    filter_good_matches(matches,
                        0.5,
                        good_matches);


    // sort good_matches in descending order based on distance
    // std::sort(good_matches.begin(), good_matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) 
    // {
    //     return a.distance > b.distance; // Sort in descending order
    // });
        
    // Filter the best match
    // std::vector<cv::Point2f> matching_kpi_01;
    // std::vector<cv::Point2f> matching_kpi_10;

    // D - put in arrays q_i: coordinates vector of the matching keypoints on image i
    // Reset
    matching_kpi_10.clear();
    matching_kpi_01.clear();
    matching_desci_01.release();
    for (const auto& good : good_matches) 
    {
        matching_kpi_10.push_back(kpi_1[good.queryIdx].pt);
        matching_kpi_01.push_back(kpi_0[good.trainIdx].pt);
        matching_desci_01.push_back(desci_0.row(good.trainIdx));
    }

    if(false)
    {
        // E - Visualiser les correspondances entre les images avec une taille plus petite et en ne montrant que les correspondances
        cv::Mat img_matches;

        // Option pour ne pas dessiner les points individuels
        cv::drawMatches(images[image_index - 1], kpi_1, images[image_index], kpi_0, good_matches, img_matches, 
                        cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        // Redimensionner l'image résultante si elle est trop grande pour l'écran
        double scale_factor = 0.7; // Facteur de redimensionnement (ajustez en fonction de la taille de vos images)
        cv::resize(img_matches, img_matches, cv::Size(), scale_factor, scale_factor);

        // Afficher les correspondances
        cv::imshow("Good Matches", img_matches);
        cv::waitKey(0); // Attend une touche pour fermer la fenêtre
    }

    return(0);
}

int VisualOdometry_monocular::triangulate(cv::Mat& PMi_0,
                                          cv::Mat& PMi_1,
                                          std::vector<cv::Point2f>& matching_kpi_01,
                                          std::vector<cv::Point2f>& matching_kpi_10,
                                          std::vector<cv::Point3f>& points3D)
{
    // Triangulate points
    //points4D.release(); // output of the function  // FIND better name
    cv::Mat points4D;

    cv::triangulatePoints(PMi_1, 
                          PMi_0, 
                          matching_kpi_10, 
                          matching_kpi_01,
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
        points3D.emplace_back(Xn.at<float>(i), Yn.at<float>(i), Zn.at<float>(i)); // AUTRE MANIERE CHEC FAST
    }

    // std::cout << "print points" << std::endl;
    // // ATTENTION JE SAIS PAS SI CETTE PARTIE ET OCRRECT
    // // -> si c'est bon 

    // std::cout << points4D.col(0) << std::endl;
    // std::cout << points3D[0] << std::endl;

    // std::cout << points4D.col(1)<< std::endl;
    // std::cout << points3D[1] << std::endl;

    // int oui;
    // std::cin >> oui;
    
    // // std::cout << points4D << std::endl;
    // // cv::waitKey(0);  // marche pas pour stop
    // std::cout << points3D << std::endl;
    // cv::waitKey(0); 
    // std::cout << "find print" << std::endl;
    return(0);
}

cv::Mat VisualOdometry_monocular::find_Rti(std::vector<cv::Point2f> matching_kp,
                                            std::vector<cv::Point3f> points3D,
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
    //                                   matching_kp, 
    //                                   intrinsic_matrix, 
    //                                   distCoeffs, 
    //                                   Rvec, 
    //                                   tvec, 
    //                                   false, 
    //                                   iterationsCount, 
    //                                   reprojectionError, 
    //                                   confidence, 
    //                                   inliers);
    std::cout << "debut calcul PnP " << std::endl;
    std::cout << "matching_kp "<< matching_kp.size() << std::endl;
    std::cout << "points3D "<< points3D.size() << std::endl;
    std::cout << "intrinsic_matrix "<< intrinsic_matrix << "\n\n";

    // for (int k=0; k<points3D.size(); k++)
    // {
    //     std::cout << "points3D" << k << "value" << points3D[k] << std::endl;
    // }

    
    cv::solvePnP(points3D, matching_kp, intrinsic_matrix, cv::Mat(), Rvec, tvec);
    std::cout << "fin calcul PnP,\n\n" << std::endl;
    

    std::cout << "tvec" << tvec << std::endl;
    std::cout << "Rvec" << Rvec << std::endl;

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
/*
void VisualOdometry_monocular::main_2D_to_2D()
{
    std::cout << "Visual odometry monocular: 2D to 2D" << std::endl;
    // 1 - get images et setup calibrations 
    get_images(data_directory);
    get_calibration(data_directory);
    
    cv::Mat Ti = cv::Mat::zeros(4, 4, CV_32F); // Transform from image i-1 and i
    cv::Mat Ci = cv::Mat::eye(4, 4, CV_32F); // current pose
    Ci = get_first_pose(data_directory);

    cv::Mat desci_0;
    cv::Mat desci_1;

    std::vector<cv::KeyPoint> kpi_0; 
    std::vector<cv::KeyPoint> kpi_1; //    also create a pointer to change ref reasily ? 

    std::vector<cv::Point2f> matching_kpi_01;
    std::vector<cv::Point2f> matching_kpi_10;

    cv::Mat Ri, ti; 

    // write first the first pose
    write_pose("poses/test.txt", Ci);

    // Initiate current keypoints and descriptors --> the loop start from iamge 1 and re use the previous keypoints and descriptors 
    orb->detectAndCompute(images[0], cv::noArray(), kpi_0, desci_0);

    for(size_t image_index = 1; image_index < images.size(); image_index++)
    {          
        // 2 - extract and matche feature between Ik-1 and Ik || Todo, only extract one image at the same time (put old image OR feature in a pointer variable )
        extract_and_matche_features((int) image_index, 
                                    desci_0, 
                                    desci_1, 
                                    kpi_0,  
                                    kpi_1,
                                    matching_kpi_01,
                                    matching_kpi_10);
        
        // 3 - Compute essential matrix for image pair Ik-1 and Ik 
        // 4 - Decompose essential matrix into Rk and tk, and from Tk
        // 5 - Compute relatice scale and resacle tk accordingly // fait dans Recoverpose
        motion_estimation(matching_kpi_01, 
                          matching_kpi_10,
                          Ri, 
                          ti);

        cv::Mat Ti = fuse_R_t(Ri, ti);
        // 6 - Concatenate transformation by computing Ck = Ck-1 Tk
        //Ci = Ci.mul(Ti);//...(C x Ti)
        Ci = Ci * Ti.inv();

        // std::cout << "Ti  :\n" << Ti <<std::endl;
        // std::cout << " Ci :\n" << Ci <<std::endl;
        write_pose("poses/test.txt", Ci);
        std::cout << image_index <<std::endl;
    }
    std::cout << "Over" <<std::endl;
}
*/
void VisualOdometry_monocular::main_3D_to_2D()
{
    std::cout << "Visual odometry monocular: 2D to 2D" << std::endl;
    // 1 - get images et setup calibrations 
    get_images(data_directory);
    get_calibration(data_directory);
    
    cv::Mat Ti = cv::Mat::zeros(4, 4, CV_32F); // Transform from image i-1 and i
    cv::Mat C0 = cv::Mat::eye(4, 4, CV_32F); // current pose
    C0 = get_first_pose(data_directory);

    cv::Mat desci_0;
    cv::Mat desci_1;
    cv::Mat desci_2;

    std::vector<cv::KeyPoint> kpi_0; 
    std::vector<cv::KeyPoint> kpi_1; //    also create a pointer to change ref reasily ? 
    std::vector<cv::KeyPoint> kpi_2;

    cv::Mat matching_desci_01; // rename en matching_desci_01, matching_desci_12
    cv::Mat matching_desci_12;
    // cv::Mat matching_desci_2;

    std::vector<cv::Point2f> matching_kpi_01;
    std::vector<cv::Point2f> matching_kpi_10;
    std::vector<cv::Point2f> matching_kpi_2;

    std::vector<cv::Point2f> matching_kpi_0all; // matching points of frame i-0 of all points (of frame (i,i-1;i-2))
    std::vector<cv::Point2f> matching_kpi_1all;

    cv::Mat Ri, ti, Rti;

    std::vector<cv::Point3f> points_viz;
    std::vector<std::vector<cv::Point3f>> points_array;

    // write first the first pose
    write_pose("poses/3D_2D.txt", C0);

    // 1.2 Extract and match features
    // get find matching_kpi_10 and matching_kpi_01: the 2D points in the image
    orb->detectAndCompute(images[0], cv::noArray(), kpi_0, desci_0); // extract img1
    extract_and_matche_features(1,  // extract img2 and matche f1 & f2
                                desci_0, 
                                desci_1, 
                                kpi_0,  
                                kpi_1,
                                matching_kpi_01,
                                matching_kpi_10,
                                matching_desci_01);
                                                            
    // 1.3  
    // projection Matrice for each iamges
    cv::Mat PMi_1, PMi_0; 
    // turn the first 3 lines and 4th column of Ci : 4x4 -> 3x4
    PMi_1 = intrinsic_matrix * C0(cv::Range(0, 3), cv::Range::all()); // Matrice de projection = K * [R|t], ici R0 et t0

    // CHANGER UNE FOIS QUE CA MARCHEN MOTION ESTIMATION DOIT PRENDRE Ri ti EN sortit / je sais plus pourquoi. pour la première étape il faut faire une motion estimation 2D 2D 
    motion_estimation(matching_kpi_01, 
                      matching_kpi_10,
                      Ri, 
                      ti);

    cv::hconcat(Ri, ti, Rti); 
    Rti.convertTo(Rti, CV_32F); // Convert to from double to float

    PMi_0 = intrinsic_matrix * Rti; // Matrice de projection = K * [R|t], ici R1 et t1

    // Triangulate points
    std::vector<cv::Point3f> points3Di_1; // output of the function  // FIND better name
    triangulate(PMi_0,
                PMi_1,
                matching_kpi_01,
                matching_kpi_10,
                points3Di_1);

    // PCL VIZ
    points_array.push_back(points3Di_1);

    std::cout <<"\ndebut loop\n";

    for(size_t image_index = 1; image_index < images.size(); image_index++)
    {          
        std::cout << " BOUCLE "<< image_index << std::endl;

        // "Swap"

        // déjà fait dans "extract_and_matche_features"
        // desci_1 = desci_0.clone();
        // kpi_1 = kpi_0; 

        // inutile pour suelement 3 frames
        // matching_kpi_10   // matched_kp1_12 = matched_kp2_23
        //  // matched_kp2_12 = matched_kp3_23
        matching_desci_12 = matching_desci_01.clone();

        // 2.2 - extract and matche feature between frame i-1 and i
        std::cout << " etape 2.2 " << std::endl;
        extract_and_matche_features((int) image_index, 
                                    desci_0, 
                                    desci_1, 
                                    kpi_0,  
                                    kpi_1,
                                    matching_kpi_01,
                                    matching_kpi_10,
                                    matching_desci_01);

        std::cout << " etape 2.2 - start tracking" << std::endl;
        // MATCHN BETWEEN i 01 and 12
        // B - match features
        std::vector<std::vector<cv::DMatch>> matches_all;
        my_flann->knnMatch(matching_desci_12, matching_desci_01, matches_all, 2);


        std::vector<cv::DMatch> good_matches_all;
        filter_good_matches(matches_all,
                            0.5,
                            good_matches_all);
        

        matching_kpi_1all.clear();
        matching_kpi_0all.clear();
        std::vector<cv::Point3f> points3Di_1_filtered;
        // matching_desci_01.release();

        std::cout << " etape 2.2 - tracking - sorting " << std::endl;
        for (const auto& good : good_matches_all) 
        {
            // matching_kpi_1all.push_back(kpi_1[good.queryIdx]);
            matching_kpi_0all.push_back(matching_kpi_01[good.trainIdx]);
            points3Di_1_filtered.push_back(points3Di_1[good.queryIdx]); // points3D are points from image i-1 -> it mush be filtered using querry points
            // matching_desci_01.push_back(desci_0.row(good.trainIdx));   
        }
        std::cout << " etape 2.2 - end tracking" << std::endl;
        // filter finded 3D points -> must be matching with matching keypoints of frame i

        //NEW
        // on va devoir faire le matching entre les good descriptors entre i et i-1
        // ensuite, on sauvagardes tout les kp et des précédents
        // enfin on enlève de la précédente triangulation tout les point 3D des points 2D qui ont pas matché 
        // (-> je me demande si on peut pas faire la triangulation avec seulement les points qui on marché ? -> je pense que y'a moyen mais ça a l'air d'êtr eun casse tête en plus alors que en les enlevant simplement c'est plus simple je pense)
        
        // @TODO
        // refaire TOUTES la nomenclature pour que ça soit lisible avec ce changement DONE
        // teste que mono marche correctement avec les changements DONE ça marche à peu près 
        // faire le passe des donnée d'une frame à l'autre
        // faire le matching entre i-2 i-1 et i-1 et i

        // OLD
        // new step -> tracking 3D points
        // kepp all the previous q and descriptors that are still in the current image
        // rajoute un moyen de suivre les points pendant au moins 3 images (entre les points triangulé et l'étape d'après)


        
        // =====================================
        // ================ VIZ ================
        // =====================================
        // if(0)
        // {
            // std::vector<cv::Point3f> points;
            // points = points3Di_1;

            // VESION UN SUEL PONT CLOUDS
            // points_viz.insert(points_viz.end(), points3Di_1.begin(), points3Di_1.end());
            // std::cout<< "NOMBRE DE POINTS\n" <<points_viz.size();
            // Créer un nuage de points
            // cv::viz::WCloud cloud(points_viz, cv::viz::Color::red());


            // cv::viz::Viz3d window("Viz Demo");

            // // Ajouter le nuage de points à la fenêtre
            // window.showWidget("Cloud", cloud);

            // // Boucle de rendu
            // window.spin();
        // }
        // points3Di_1_filtered instead ? 
        points_array.push_back(points3Di_1);
        if(image_index == 3)
        {
            std::cout << " pcl " << std::endl;
            // créé/ajoute un nuage de points
            cv::viz::WCloud cloud1(points_array[0], cv::viz::Color::red());
            cv::viz::WCloud cloud2(points_array[1], cv::viz::Color::blue());
            cv::viz::WCloud cloud3(points_array[2], cv::viz::Color::green());
            cv::viz::WCloud cloud4(points_array[3]);

            cv::viz::Viz3d window("2 pcl");

            window.showWidget("Cloud1", cloud1);
            window.showWidget("Cloud2", cloud2);
            window.showWidget("Cloud3", cloud3);
            window.showWidget("Cloud4", cloud4);
            
            window.spin();
        }

        // while (!window.wasStopped()) {
        //     window.spinOnce(1, true);
        // }

        // ===============================================
        // ================= no more viz =================
        // ===============================================

        // 2.3 PNP ransac
        // (?) attention fonction qui renvoie une varaible (?)
        std::cout << " etape 2.3 " << std::endl;
        cv::Mat Ti = find_Rti(matching_kpi_0all,
                                points3Di_1_filtered,
                                Rti);

        std::cout << " print Rti \n"<< Rti << std::endl;
        std::cout << " print ti "<< Ti << std::endl;
        C0 = C0 * Ti.inv();

        PMi_1 = PMi_0.clone(); 
        
        std::cout << "int mat" << std::endl;
        PMi_0 = intrinsic_matrix * Rti; // Matrice de projection = K * [R|t] // il semblemrait que ça soit juste, cependant, je pense que 

        // 2.4 triangulate
        std::cout << " etape 2.4 " << std::endl;
        triangulate(PMi_0,
                    PMi_1,
                    matching_kpi_01,
                    matching_kpi_10,
                    points3Di_1);

        write_pose("poses/3D_2D.txt", C0);
        std::cout << image_index <<std::endl;

        // NEW STEP "swapping" data


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
- peut être que le cacule de la matrice Ri est mauvaishttps://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga54a2f5b3f8aeaf6c76d4a31dece85d5d


// faire comme dans trackgpt -> actualiser les points 3D qu'on a 
// modifier les noms des var dans extract et peut être changer les fonction tou court
// rajouter une fonction "tracking"


// vérifier que c'est bien el bon déroulé
// Clean le code
// et surtout rechanger les nom de var
// enelver lee 2D->2D ? 
// on sait qu'on a des valeurs abérantes edès al 1rer itération de PnP (x100 trop grand)
/** option possible
 * -> on se plante dans le matching des points et on envoie pas les bon points (doubt)
 * -> points3D et triangualte semblait faux mais je vois pas comment on se rate, puisque que erreur des &ere itération
 * -> distCoeffs -> peut être que ça change tout -> askip les images sont déjà rectifié donc ce serait nul 
 * -> on se plant dans tout les variable qu'on a géchant ? 
 * -> e=on gère mal Ti ou Rti avec rodrigues (doubt)
 */

/**
 * we call 
 * images_index = i
 * kpi_x: all detected keypoints on frame i-x
 * matching_kpi_x: all matching 2d/keypoints on frame i-x
 * desci_x: descriptor of kp_x
 * points3D
 * PMi_x: projection matrix on frame i-x
 * 
 */