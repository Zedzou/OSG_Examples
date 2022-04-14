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
        pathName = "/home/zed/Project/GraphSLAM/3RScan/data/3RScan/754e884c-ea24-2175-8b34-cead19d4198d/sequence/" + prefix + num.str() + rgb_suffix;
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
        pathName = "/home/zed/Project/GraphSLAM/3RScan/data/3RScan/754e884c-ea24-2175-8b34-cead19d4198d/sequence/" + prefix + num.str() + depth_suffix;
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
        pathName = "/home/zed/Project/GraphSLAM/3RScan/data/3RScan/754e884c-ea24-2175-8b34-cead19d4198d/sequence/" + prefix + num.str() + pose_suffix;

        // 读取位姿
        Eigen::Matrix4f pose;
        pose.setIdentity();
        std::ifstream file(pathName);
        assert(file.is_open());
        if (file.is_open()) {
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    file >> pose(i, j);
            // pose.block<3, 1>(0, 3) *= 1000.0f;
            file.close();
        }
        poseMap[pathName] = pose;
    }

}