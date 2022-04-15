#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iomanip>

class DatasetLoader
{
    public:
        DatasetLoader();
        ~DatasetLoader();

        std::vector<std::string> rgb_path;
        std::vector<std::string> depth_path;
        std::map<std::string, Eigen::Matrix4f> poseMap; // 位姿字典
    
    private:

        void readPose(); // 读取位姿
        void readRGBpath(); // 读取RGB路径
        void readDepthpath(); // 读取深度图路径

        // 数据定义
        std::string dataPath = "/home/zed/Project/GraphSLAM/test_dataset/Test_2/sequence/";
        std::string prefix = "frame-";
        std::string rgb_suffix = ".color.jpg";
        std::string depth_suffix = ".depth.pgm";
        std::string pose_suffix = ".pose.txt";
        const int frameNum = 45;

        // 欧拉角转成旋转矩阵
        Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d &euler_angle)
        {
            Eigen::Matrix3d rotation_matrix1;
            rotation_matrix1 = Eigen::AngleAxisd( euler_angle[2], Eigen::Vector3d::UnitZ() ) * 
                            Eigen::AngleAxisd( euler_angle[1], Eigen::Vector3d::UnitY() ) *
                            Eigen::AngleAxisd( euler_angle[0], Eigen::Vector3d::UnitX() );

            return rotation_matrix1;
        }

};