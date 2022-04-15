#pragma once
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>

class LoadModel
{
    public:
        LoadModel()
        {
            ModelNode = osgDB::readNodeFile(modelPath, a);

            // // 模型旋转
            // rotateMT->setMatrix(osg::Matrix::rotate( osg::inDegrees(0.147763f), 1.0f, 0.0f, 0.0f) );
            // rotateMT->setMatrix(osg::Matrix::rotate( osg::inDegrees(0.0786316f), 0.0f, 1.0f, 0.0f) );
            // rotateMT->setMatrix(osg::Matrix::rotate( osg::inDegrees(87.6724f), 0.0f, 0.0f, 1.0f) );
            // rotateMT->addChild(ModelNode);
            // ModelNode = rotateMT->asNode();

            // // 模型平移
            // mt->setMatrix(osg::Matrix::translate(-0.940349, 1.29038, -0.154145));
            // mt->addChild(ModelNode.get());
            // ModelNode = mt->asNode();
        }

        // 添加子模块
        void addSubGroup(osg::ref_ptr<osg::Group> &rootGroup)
        {
            rootGroup->addChild(ModelNode);
        }

    private:
        osg::ref_ptr<osg::Node> ModelNode;
        osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform();
        osg::MatrixTransform* rotateMT = new osg::MatrixTransform;
        osgDB::Options  *a = new osgDB::Options(std::string("noTriStripPolygons"));
        std::string modelPath = "/media/zed/46BCAB97BCAB8053/Scan3R_ContexCapture/Model - Cloud.ply";
};