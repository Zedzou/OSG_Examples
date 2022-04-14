#pragma once 

#include <osgViewer/ViewerEventHandlers>
#include <osg/LineWidth>

class CoordinateAxis
{
    public:
        CoordinateAxis()
        {
            SetVertexColor();
            geometry->setVertexArray(vex);
            geometry->setColorArray(colors);
            geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
            geometry->addPrimitiveSet(primitiveSet);
            geometry->getOrCreateStateSet()->setAttribute(lw, osg::StateAttribute::ON);
            geode->addDrawable(geometry);
        }

        void addSubGroup(osg::ref_ptr<osg::Group> &rootGroup)
        {
            
            rootGroup->addChild(geode);
        }
        
    private:
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
        osg::ref_ptr<osg::Vec3Array> vex = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        osg::ref_ptr<osg::PrimitiveSet> primitiveSet = new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 6);
        osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(6.0);
        const float size_ = 2.0f;

        void SetVertexColor()
        {
            // 顶点
            vex->push_back(osg::Vec3(0.0, 0.0, 0.0));
            vex->push_back(osg::Vec3(1.0*size_, 0.0, 0.0));

            vex->push_back(osg::Vec3(0.0, 0.0, 0.0));
            vex->push_back(osg::Vec3(0.0, 1.0*size_, 0.0));

            vex->push_back(osg::Vec3(0.0, 0.0, 0.0));
            vex->push_back(osg::Vec3(0.0, 0.0, 1.0*size_));

            // 颜色
            colors->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0));
            colors->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0)); // x

            colors->push_back(osg::Vec4(0.0, 1.0, 0.0, 1.0));
            colors->push_back(osg::Vec4(0.0, 1.0, 0.0, 1.0)); // y

            colors->push_back(osg::Vec4(0.0, 0.0, 1.0, 1.0));
            colors->push_back(osg::Vec4(0.0, 0.0, 1.0, 1.0)); // z
        };
};