//
// Created by lxy on 2024/7/10.
//

#include "SlidingWindowPoseOptimizer.h"
int main(){
    std::ifstream tum_file("/home/lxy/event_camera/cannyEVT/Canny-EVT/eventDrone.txt");
    if (!tum_file.is_open()) {
        std::cerr << "Error opening file: "<< std::endl;
        return 0;
    }
    std::string line;
    SlidingWindowPoseOptimizer::Ptr mSlidingOptimizer =std::make_shared<SlidingWindowPoseOptimizer>(5,2);

    while (std::getline(tum_file, line)) {
        std::istringstream iss(line);
        double timestamp, x, y, z, qw, qx, qy, qz;

        // Read timestamp and xyz
        if (!(iss >> timestamp >> x >> y >> z)) {
            std::cerr << "Error reading timestamp, x, y, z from line: " << line << std::endl;
            continue;
        }

        // Read quaternion components
        if (!(iss >> qw >> qx >> qy >> qz)) {
            std::cerr << "Error reading quaternion from line: " << line << std::endl;
            continue;
        }

        // Construct Eigen objects
        Eigen::Vector3d position(x, y, z);
        Eigen::Quaterniond quaternion(qw, qx, qy, qz);

        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
        matrix.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
        matrix.block<3, 1>(0, 3) = position;
        // Add pose to vector
        mSlidingOptimizer->addPose(matrix,timestamp);

    }



}