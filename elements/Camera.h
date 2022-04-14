#pragma once

#include <osgViewer/ViewerEventHandlers>
#include <osg/LineWidth>
#include <Eigen/Core>
#include "DatasetLoader.h"

class CameraDraw
{
    public:
        CameraDraw(Eigen::Matrix<float, 4, 4> &Trans)
        {   
            transformer = Trans;
            SetVertexColor();
            geometry->setVertexArray(vex);
            geometry->setColorArray(colors);
            geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
            geometry->addPrimitiveSet(primitiveSet);
            geometry->getOrCreateStateSet()->setAttribute(lw, osg::StateAttribute::ON);
            CameraNode->addDrawable(geometry);
        }

        void addSubGroup(osg::ref_ptr<osg::Group> &rootGroup)
        {
            rootGroup->addChild(CameraNode);
        }
        
    private:
        osg::ref_ptr<osg::Geode> CameraNode = new osg::Geode;
        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
        osg::ref_ptr<osg::Vec3Array> vex = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        osg::ref_ptr<osg::PrimitiveSet> primitiveSet = new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 16);
        osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(3.0);
        Eigen::Matrix<float, 4, 4> transformer;
        const float size = 0.08f;

        // 原始相机参数
        Eigen::Matrix<float, 5, 3> SetRotationTransformCamera(Eigen::Matrix<float, 4, 4> &transformer)
        {
            Eigen::Matrix<float, 4, 3> CameraVertex;
            CameraVertex<< -0.5f*size, -0.5f*size, 1*size, 
                            0.5f*size, -0.5f*size, 1*size, 
                            0.5f*size,  0.5f*size, 1*size, 
                            -0.5f*size,  0.5f*size, 1*size;

            Eigen::Matrix<float, 3, 3> Rotation; // 旋转
            Eigen::Matrix<float, 3, 1> Translation; // 平移
            Rotation = transformer.block<3, 3>(0, 0);
            Translation << transformer(0, 3), transformer(1, 3), transformer(2, 3);

            // 新相机的向量
            Eigen::Matrix<float, 3, 4> CameraVector = Rotation * CameraVertex.transpose();
            CameraVector.colwise() += Translation;

            // 相机的4个点
            Eigen::Vector3f point_1, point_2, point_3, point_4;
            point_1 << CameraVector(0, 0), CameraVector(1, 0), CameraVector(2, 0);
            point_2 << CameraVector(0, 1), CameraVector(1, 1), CameraVector(2, 1);
            point_3 << CameraVector(0, 2), CameraVector(1, 2), CameraVector(2, 2);
            point_4 << CameraVector(0, 3), CameraVector(1, 3), CameraVector(2, 3);

            Eigen::Matrix<float, 5, 3> CameraModelVertex; // 相机的顶点
            CameraModelVertex <<    transformer(0, 3), transformer(1, 3), transformer(2, 3),
                                    point_1(0),point_1(1),point_1(2),
                                    point_2(0),point_2(1),point_2(2),
                                    point_3(0),point_3(1),point_3(2),
                                    point_4(0),point_4(1),point_4(2);

            return CameraModelVertex;
        };

        void SetVertexColor()
        {
            // transformer << -0.36795,-0.890503,0.267616,0.408033,-0.723332,0.454972,0.519415,-1.34522,-0.584298,-0.00245673,-0.811535,-0.0849775,0,0,0,1;
            Eigen::Matrix<float, 5, 3> CameraVertex = SetRotationTransformCamera(transformer);

            // 顶点
            vex->push_back(osg::Vec3(CameraVertex(0,0), CameraVertex(0,1), CameraVertex(0,2)));
            vex->push_back(osg::Vec3(CameraVertex(1,0), CameraVertex(1,1), CameraVertex(1,2)));

            vex->push_back(osg::Vec3(CameraVertex(0,0), CameraVertex(0,1), CameraVertex(0,2)));
            vex->push_back(osg::Vec3(CameraVertex(2,0), CameraVertex(2,1), CameraVertex(2,2)));

            vex->push_back(osg::Vec3(CameraVertex(0,0), CameraVertex(0,1), CameraVertex(0,2)));
            vex->push_back(osg::Vec3(CameraVertex(3,0), CameraVertex(3,1), CameraVertex(3,2)));

            vex->push_back(osg::Vec3(CameraVertex(0,0), CameraVertex(0,1), CameraVertex(0,2)));
            vex->push_back(osg::Vec3(CameraVertex(4,0), CameraVertex(4,1), CameraVertex(4,2)));

            vex->push_back(osg::Vec3(CameraVertex(1,0), CameraVertex(1,1), CameraVertex(1,2)));
            vex->push_back(osg::Vec3(CameraVertex(2,0), CameraVertex(2,1), CameraVertex(2,2)));

            vex->push_back(osg::Vec3(CameraVertex(2,0), CameraVertex(2,1), CameraVertex(2,2)));
            vex->push_back(osg::Vec3(CameraVertex(3,0), CameraVertex(3,1), CameraVertex(3,2)));

            vex->push_back(osg::Vec3(CameraVertex(3,0), CameraVertex(3,1), CameraVertex(3,2)));
            vex->push_back(osg::Vec3(CameraVertex(4,0), CameraVertex(4,1), CameraVertex(4,2)));

            vex->push_back(osg::Vec3(CameraVertex(4,0), CameraVertex(4,1), CameraVertex(4,2)));
            vex->push_back(osg::Vec3(CameraVertex(1,0), CameraVertex(1,1), CameraVertex(1,2)));

            // 颜色
            for(int i=0; i<17; i++)
            {
                colors->push_back(osg::Vec4(1.0, 1.0, 0.0, 1.0));
            }

        }

        

        // void SetVertexColor()
        // {
        //     // 顶点
        //     vex->push_back(osg::Vec3(0.0, 0.0, 0.0));
        //     vex->push_back(osg::Vec3(-0.5f*size, -0.5f*size, 1*size));

        //     vex->push_back(osg::Vec3(0.0, 0.0, 0.0));
        //     vex->push_back(osg::Vec3(0.5f*size, -0.5f*size, 1*size));

        //     vex->push_back(osg::Vec3(0.0, 0.0, 0.0));
        //     vex->push_back(osg::Vec3(0.5f*size,  0.5f*size, 1*size));

        //     vex->push_back(osg::Vec3(0.0, 0.0, 0.0));
        //     vex->push_back(osg::Vec3(-0.5f*size,  0.5f*size, 1*size));

        //     vex->push_back(osg::Vec3(-0.5f*size,  0.5f*size, 1*size));
        //     vex->push_back(osg::Vec3(0.5f*size,  0.5f*size, 1*size));

        //     vex->push_back(osg::Vec3(0.5f*size,  0.5f*size, 1*size));
        //     vex->push_back(osg::Vec3(0.5f*size, -0.5f*size, 1*size));

        //     vex->push_back(osg::Vec3(0.5f*size, -0.5f*size, 1*size));
        //     vex->push_back(osg::Vec3(-0.5f*size, -0.5f*size, 1*size));

        //     vex->push_back(osg::Vec3(-0.5f*size, -0.5f*size, 1*size));
        //     vex->push_back(osg::Vec3(-0.5f*size,  0.5f*size, 1*size));

        //     // 颜色
        //     for(int i=0; i<17; i++)
        //     {
        //         colors->push_back(osg::Vec4(1.0, 1.0, 0.0, 1.0));
        //     }
            
        // };

};