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


    int SCANNER_COLS = 2083;
    int SCANNER_ROWS = 64;
    float UPPER_BOUND = 2.0;
    float LOWER_BOUND = -24.3;
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
    cv::imshow("lol", depthImg);
//    int k = cv::waitKey(10);
//    if (k == 27) {
//        exit(0);
//    }


    // Original data from ouster is 1024 x 64
    int wantedWidth = 1024, wantedHeight = 512;
    double fx = 880, fy = 880;

    std::vector<double> beamAngles = {-24.3, -23.8, -23.3, -22.8, -22.3, -21.8, -21.3, -20.8, -20.3, -19.8, -19.3,
                                      -18.8, -18.3, -17.8, -17.3, -16.8, -16.3, -15.8, -15.3, -14.8, -14.3, -13.8,
                                      -13.3, -12.8, -12.3, -11.8, -11.3, -10.8, -10.3, -9.8, -9.3, -8.8,
                                      -8.3333333, -8.0, -7.666666, -7.3333333, -7.0,
                                      -6.66666666666, -6.3333333333, -6.0, -5.6666667, -5.3333333,
                                      -5.0, -4.6666666667, -4.333333, -4.0, -3.666666667,
                                      -3.333333333333, -3.0, -2.66666667, -2.3333333333, -2.0,
                                      -1.6666666667, -1.3333333333, -1.0, -0.66666666667, -0.333333333333333,
                                      0.0, 0.33333333334, 0.66666666667, 1.0, 1.333333333, 1.6666666667, 2.0};

    double cx = wantedWidth / 2.0, cy = wantedHeight / 2.0;

    // TODO: Create map_x and map_y that for wanted image produces indicies in the original image
    for (int imagePart = 0; imagePart < 6; imagePart++) {

        // We create 6 images so we move by 60 deg
        double thetaIncrement = imagePart * M_PI / 3;

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
                double originalU = theta * 180.0 / M_PI * 2083.0 / 360.0 + 1041.0;

                // Vertical angle to index
                double originalV = phi * 180.0 / M_PI * 64.0 / 26.8 + 31.5;

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

        cv::Mat outputImg = cv::Mat(wantedHeight, wantedWidth, CV_8UC1, cv::Scalar(0,0,0));
//        cv::Mat inpainted = cv::Mat(wantedHeight, wantedWidth, CV_8UC1, cv::Scalar(0,0,0));
//        cv::Mat mask = cv::Mat(wantedHeight, wantedWidth, CV_8UC1, cv::Scalar(0,0,0));
        cv::remap(depthImg, outputImg, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
        std::string filename =
                "/root/share/tf/dataset/kitti_intensity_val_05/" + std::to_string(imagePart) + "_" +
                std::to_string(cloudMsg->header.stamp.toSec()) + ".png";
//        std::cout<<filename<<std::endl;
//        cv::cvtColor(outputImg, outputImg, cv::COLOR_BGR2GRAY);
//        outputImg = outputImg*255;
//        outputImg.convertTo(outputImg,CV_8U);
//        cv::threshold(outputImg, mask, 10, 1, cv::THRESH_BINARY_INV);
//        cv::imshow("lol2", mask*255);
//        cv::GaussianBlur(outputImg, outputImg, CvSize(5,5), 0);
//        cv::inpaint(outputImg, mask, inpainted, 3, cv::INPAINT_TELEA);
        cv::imshow("lol1", outputImg);
//        cv::imwrite(filename, outputImg*255);
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