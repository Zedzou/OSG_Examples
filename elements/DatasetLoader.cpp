#include "DatasetLoader.h"

DatasetLoader::DatasetLoader()
{
    readPose();
    readRGBpath();
    readDepthpath();
}

DatasetLoader::~DatasetLoader()
{

}

// 读取RGB路径
void DatasetLoader::readRGBpath()
{
    for(int i=0; i<frameNum+1; i++)
    {
        std::stringstream num;
        std::string pathName;
        num << std::setfill('0') << std::setw(6) << i;
        pathName = "/home/zed/Project/GraphSLAM/test_dataset/code/script/pose/" + prefix + num.str() + rgb_suffix;
        rgb_path.push_back(pathName);
    }
}

// 读取RGB路径
void DatasetLoader::readDepthpath()
{
    for(int i=0; i<frameNum+1; i++)
    {
        std::stringstream num;
        std::string pathName;
        num << std::setfill('0') << std::setw(6) << i;
        pathName = "/home/zed/Project/GraphSLAM/test_dataset/code/script/pose/" + prefix + num.str() + depth_suffix;
        depth_path.push_back(pathName);
    }
}

// 读取位姿
void DatasetLoader::readPose()
{

    for(int i=0; i<frameNum+1; i++)
    {
        // 路径
        std::stringstream num;
        std::string pathName;
        num << std::setfill('0') << std::setw(6) << i;
        pathName = "/media/zed/46BCAB97BCAB8053/Scan3R_ContexCapture/x_left_y_up/pose_inverse/" + prefix + num.str() + pose_suffix;

        // 读取位姿
        Eigen::Matrix4f pose;
        pose.setIdentity();
        std::ifstream file(pathName);

        assert(file.is_open());
        if (file.is_open()) {
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    file >> pose(i, j);
        }
        poseMap[pathName] = pose;
    }

}