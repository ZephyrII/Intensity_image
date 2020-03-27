//
// Created by root on 3/5/20.
//

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"


void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr &cloudMsg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloudMsg, *cloud);


    int SCANNER_COLS = 2048;
    int SCANNER_ROWS = 128;
    float UPPER_BOUND = 21.5;
    float LOWER_BOUND = -21.5;
    float VELO_FACTOR_VER = (SCANNER_ROWS - 1) / (UPPER_BOUND - LOWER_BOUND);
    float VELO_FACTOR_HOR = (SCANNER_COLS - 1) / 360.0;

    // Create a range image
    cv::Mat depthImg = cv::Mat(SCANNER_ROWS, SCANNER_COLS, CV_32FC1);
    depthImg = 0;


//    std::cout << cloud->points.size() << std::endl;
    for (int cloudPointIdx = 0; cloudPointIdx < cloud->points.size(); cloudPointIdx++) {

        // Is it a proper measurement
        if (!pcl_isfinite(cloud->points[cloudPointIdx].x) ||
            !pcl_isfinite(cloud->points[cloudPointIdx].y) ||
            !pcl_isfinite(cloud->points[cloudPointIdx].z)) {
            continue;
        }


        // Is it far enough from the scanner
        float val = cloud->points[cloudPointIdx].intensity;

        int row = 0, col = 0;

        auto &point = cloud->points[cloudPointIdx];

        float angle_hor = std::atan2(point.y, point.x);
        float angle_ver = std::atan(point.z / std::sqrt(point.x * point.x + point.y * point.y));

        col = SCANNER_COLS / 2 - int(((angle_hor * 180 / M_PI)) * VELO_FACTOR_HOR + 0.5) - 1;
        row = SCANNER_ROWS - int(((angle_ver * 180 / M_PI) - LOWER_BOUND) * VELO_FACTOR_VER + 0.5) - 1;
        if (row < 0 || row > SCANNER_ROWS || col < 0 || col > SCANNER_COLS) {
            continue;
        }
        // Filling depth image
        depthImg.at<float>(row, col) = val;
    }
    cv::imshow("lol", depthImg/255.0);
//    int k = cv::waitKey(10);
//    if (k == 27) {
//        exit(0);
//    }


    // Original data from ouster is 1024 x 64
    int wantedWidth = 1024, wantedHeight = 512;
    double fx = 510, fy = 510;

    std::vector<double> beamAngles = {21.4764,
                                      21.1679,
                                      20.8583,
                                      20.5477,
                                      20.2356,
                                      19.9221,
                                      19.6075,
                                      19.2919,
                                      18.975,
                                      18.6567,
                                      18.3375,
                                      18.0171,
                                      17.6955,
                                      17.3729,
                                      17.0492,
                                      16.7242,
                                      16.3982,
                                      16.0716,
                                      15.7437,
                                      15.4141,
                                      15.0841,
                                      14.7537,
                                      14.4218,
                                      14.088,
                                      13.754,
                                      13.4202,
                                      13.0844,
                                      12.7466,
                                      12.409,
                                      12.0718,
                                      11.7326,
                                      11.3909,
                                      11.0499,
                                      10.7097,
                                      10.3671,
                                      10.022,
                                      9.6778,
                                      9.3348,
                                      8.9892,
                                      8.641,
                                      8.2939,
                                      7.9482,
                                      7.5999,
                                      7.249,
                                      6.8994,
                                      6.5512,
                                      6.2005,
                                      5.8474,
                                      5.4956,
                                      5.1452,
                                      4.7925,
                                      4.4377,
                                      4.0842,
                                      3.7318,
                                      3.3775,
                                      3.0216,
                                      2.6668,
                                      2.3127,
                                      1.9573,
                                      1.601,
                                      1.2454,
                                      0.89,
                                      0.5341,
                                      0.1779,
                                      -0.1779,
                                      -0.5341,
                                      -0.89,
                                      -1.2454,
                                      -1.601,
                                      -1.9573,
                                      -2.3127,
                                      -2.6668,
                                      -3.0216,
                                      -3.3775,
                                      -3.7318,
                                      -4.0842,
                                      -4.4377,
                                      -4.7925,
                                      -5.1452,
                                      -5.4956,
                                      -5.8474,
                                      -6.2005,
                                      -6.5512,
                                      -6.8994,
                                      -7.249,
                                      -7.5999,
                                      -7.9482,
                                      -8.2939,
                                      -8.641,
                                      -8.9892,
                                      -9.3348,
                                      -9.6778,
                                      -10.022,
                                      -10.3671,
                                      -10.7097,
                                      -11.0499,
                                      -11.3909,
                                      -11.7326,
                                      -12.0718,
                                      -12.409,
                                      -12.7466,
                                      -13.0844,
                                      -13.4202,
                                      -13.754,
                                      -14.088,
                                      -14.4218,
                                      -14.7537,
                                      -15.0841,
                                      -15.4141,
                                      -15.7437,
                                      -16.0716,
                                      -16.3982,
                                      -16.7242,
                                      -17.0492,
                                      -17.3729,
                                      -17.6955,
                                      -18.0171,
                                      -18.3375,
                                      -18.6567,
                                      -18.975,
                                      -19.2919,
                                      -19.6075,
                                      -19.9221,
                                      -20.2356,
                                      -20.5477,
                                      -20.8583,
                                      -21.1679,
                                      -21.4764};

    double cx = wantedWidth / 2.0, cy = wantedHeight / 2.0;

    // TODO: Create map_x and map_y that for wanted image produces indicies in the original image
    for (int imagePart = 0; imagePart < 4; imagePart++) {

        // We create 6 images so we move by 60 deg
        double thetaIncrement = 2*imagePart * M_PI / 4;

        cv::Mat map_x = cv::Mat(wantedHeight, wantedWidth, CV_32FC1), map_y = cv::Mat(wantedHeight, wantedWidth,
                                                                                      CV_32FC1);

        for (int j = 0; j < wantedHeight; j++) {
            for (int i = 0; i < wantedWidth; i++) {

                double u = i, v = j;

                double theta = atan2(u - cx, fx);
                double phi = atan2((cy - v) * cos(theta), fy);

                theta = theta + thetaIncrement;

                // Normalizing theta to -180, 180 degs
                if (theta > M_PI)
                    theta = theta - 2 * M_PI;
                if (theta < -M_PI)
                    theta = theta + 2 * M_PI;

                // Horizontal angle to index
                double originalU = theta * 180.0 / M_PI * SCANNER_COLS / 360.0 + SCANNER_COLS/2-0.5;

                // Vertical angle to index
                double originalV = phi * 180.0 / M_PI * SCANNER_ROWS / (UPPER_BOUND - LOWER_BOUND) + SCANNER_ROWS/2-0.5;

                // Let's try finding the proper vertical index
                double phiInDeg = phi * 180.0 / M_PI;
                int beamIdx = 0;
                for (; beamIdx < beamAngles.size(); beamIdx++) {
                    if (phiInDeg > beamAngles[beamIdx])
                        break;
                }

                // If it is outside of the intensity image so nothing can be done
                double originalV2 = 64 - originalV;
                if (beamIdx != 0 && beamIdx != beamAngles.size()) {
                    // It is a lower index + a linear interpolation between two vertical beans
                    originalV2 = beamIdx - (phiInDeg - beamAngles[beamIdx]) /
                                           (beamAngles[beamIdx - 1] - beamAngles[beamIdx]);
                }

                map_x.at<float>(j, i) = originalU;
                map_y.at<float>(j, i) = originalV2;

//                    std::cout << u << " " << v << " " << theta * 180.0 / M_PI << " " << phi * 180.0 / M_PI << std::endl;
//                    std::cout << i << " " << j << " to " << originalV << " (" << phi << ") " << originalU << " (" << theta <<")" << std::endl;
            }
        }

        cv::Mat outputImg = cv::Mat(wantedHeight, wantedWidth, CV_8UC1, cv::Scalar(0, 0, 0));
//        cv::Mat inpainted = cv::Mat(wantedHeight, wantedWidth, CV_8UC1, cv::Scalar(0,0,0));
//        cv::Mat mask = cv::Mat(wantedHeight, wantedWidth, CV_8UC1, cv::Scalar(0,0,0));
        cv::remap(depthImg, outputImg, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
        std::string filename =
                "/root/share/tf/dataset/os1_128_intensity/" + std::to_string(imagePart) + "_" +
                std::to_string(cloudMsg->header.stamp.toSec()) + ".png";
//        std::cout<<filename<<std::endl;
//        cv::cvtColor(outputImg, outputImg, cv::COLOR_BGR2GRAY);
//        outputImg = outputImg*255;
//        outputImg.convertTo(outputImg,CV_8U);
//        cv::threshold(outputImg, mask, 10, 1, cv::THRESH_BINARY_INV);
//        cv::imshow("lol2", mask*255);
//        cv::GaussianBlur(outputImg, outputImg, CvSize(5,5), 0);
//        cv::inpaint(outputImg, mask, inpainted, 3, cv::INPAINT_TELEA);
        cv::imshow("lol1", outputImg/255.0);
//        cv::imwrite(filename, outputImg);
        int k = cv::waitKey(0);
        if (k == 27) {
            exit(0);
        }
    }

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "intensity");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/os1_cloud_node/points", 10000, chatterCallback);


    ros::spin();
    return 0;
}