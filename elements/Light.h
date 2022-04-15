#pragma once

#include <osg/Light>
#include <osg/LightSource>

class Light
{
    public:
        Light(){};

        void addSubGroup(osg::ref_ptr<osg::Group> &rootGroup)
        {
            boundSphere = rootGroup->getBound();
            rootStateSet = rootGroup->getOrCreateStateSet();
            rootGroup->addChild(lightNode);
        }

    private:
        osg::LightSource* lightNode = new osg::LightSource;
        osg::Light* LightSet = new osg::Light;
        osg::BoundingSphere boundSphere;
        osg::StateSet* rootStateSet; 
	    
        void SetLight()
        {
            LightSet->setLightNum(0);
            LightSet->setPosition(osg::Vec4(boundSphere.center().x(), 
                                  boundSphere.center().y(), 
                                  boundSphere.center().z() + boundSphere.radius(), 0.1f));
            LightSet->setAmbient(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
            LightSet->setDiffuse(osg::Vec4(0.5f, 0.5f, 0.5f, 0.5f));
            LightSet->setSpotCutoff(20.0f);
            LightSet->setSpotExponent(50.0f);
            LightSet->setDirection(osg::Vec3(0.0f, 0.0f, -1.0f));

            lightNode->setLight(LightSet);
            lightNode->setLocalStateSetModes(osg::StateAttribute::ON);
            lightNode->setStateSetModes(*rootStateSet, osg::StateAttribute::ON);
        }
};