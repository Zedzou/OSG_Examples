#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/LineWidth>
#include <osgDB/ReadFile> 
#include "elements/CoordinateAxis.h"
#include "elements/Models.h"
#include "elements/Model_obj.h"
#include "elements/Light.h"
#include "elements/Camera.h"
// #include "DatasetLoader.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#define PI 3.141592653

// 欧拉角转成旋转矩阵
Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d &euler_angle)
{
	Eigen::Matrix3d rotation_matrix1;
    rotation_matrix1 = Eigen::AngleAxisd( euler_angle[2], Eigen::Vector3d::UnitZ() ) * 
					   Eigen::AngleAxisd( euler_angle[1], Eigen::Vector3d::UnitY() ) *
                       Eigen::AngleAxisd( euler_angle[0], Eigen::Vector3d::UnitX() );

	return rotation_matrix1;
}

// 旋转矩阵转成欧拉角
Eigen::Vector3d RotationMatrixToEuler(Eigen::Matrix3d &RotationMatrix)
{
	Eigen::Vector3d eulerAngle1 = RotationMatrix.eulerAngles(2,1,0);
	Eigen::Vector3d Euler;
	Euler << (eulerAngle1(2)/PI)*180, (eulerAngle1(1)/PI)*180, (eulerAngle1(0)/PI)*180; // roll pitch yaw
	return Euler;
}

int main(int argc, char **argv)
{
	// 主节点
	osg::ref_ptr<osg::Group> root = new osg::Group;

	// 坐标轴
	CoordinateAxis* DrawCoordinate = new CoordinateAxis();

	// 导入点云模型
	LoadModel* loadModel = new LoadModel();

	// 导入obj模型
	LoadModel_obj* loadModel_obj = new LoadModel_obj();
	
	// 灯光
	Light* light = new Light();

	DrawCoordinate->addSubGroup(root);
	// loadModel->addSubGroup(root);
	loadModel_obj->addSubGroup(root);
	light->addSubGroup(root);

	// 相机测试
	Eigen::Vector3d euler_42;
	euler_42 << 0.0f, 0.0f, 0.0f;
	Eigen::Matrix3d tr_42 = eulerToRotationMatrix(euler_42);
	Eigen::Matrix4f transformer_test_42;
	transformer_test_42 << tr_42(0,0), tr_42(0,1), tr_42(0,2), 0.0,
						   tr_42(1,0), tr_42(1,1), tr_42(1,2), 0.0,
						   tr_42(2,0), tr_42(2,1), tr_42(2,2), 0.0,
						   0.0, 0.0, 0.0, 1.0;
	CameraDraw* cameraDraw_test_42 = new CameraDraw(transformer_test_42);
	std::cout << transformer_test_42 << std::endl;
	cameraDraw_test_42->addSubGroup(root);

	// // 相机测试
	// Eigen::Vector3d euler_18;
	// euler_18 << -167.329983642115f, -51.0643492764341f, 91.5941126285635f;
	// Eigen::Matrix3d tr_18 = eulerToRotationMatrix(euler_18);
	// Eigen::Matrix4f transformer_test_18;
	// transformer_test_18 << tr_18(0,0), tr_18(0,1), tr_18(0,2),  0.0738629911590837,
	// 					   tr_18(1,0), tr_18(1,1), tr_18(1,2), -2.31122092404561,
	// 					   tr_18(2,0), tr_18(2,1), tr_18(2,2), -0.587560325838498,
	// 					   0.0, 0.0, 0.0, 1.0;
	// CameraDraw* cameraDraw_test_18 = new CameraDraw(transformer_test_18);
	// cameraDraw_test_18->addSubGroup(root);

	// 相机测试
	Eigen::Vector3d euler_10;
	euler_10 << 0.0f, 0.0f, 0.0f;
	Eigen::Matrix3d tr_10 = eulerToRotationMatrix(euler_10);
	Eigen::Matrix4f transformer_test_10;
	transformer_test_10 << tr_10(0,0), tr_10(0,1), tr_10(0,2),-1.75872384601252,
						   tr_10(1,0), tr_10(1,1), tr_10(1,2), 5.73205758913697,
						   tr_10(2,0), tr_10(2,1), tr_10(2,2), 0.955147908118282,
						   0.0, 0.0, 0.0, 1.0;
	CameraDraw* cameraDraw_test_10 = new CameraDraw(transformer_test_10);
	cameraDraw_test_10->addSubGroup(root);

	// 导入数据
	DatasetLoader* datasetLoader = new DatasetLoader();
	std::map<std::string, Eigen::Matrix4f> poseMap = datasetLoader->poseMap;
	for(auto &s:poseMap)
	{
		Eigen::Matrix4f transformer = s.second;
		CameraDraw* cameraDraw = new CameraDraw(transformer);
		cameraDraw->addSubGroup(root);
	}

	// viewer
	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer();
	viewer->addEventHandler(new osgViewer::WindowSizeHandler());
	viewer->setSceneData(root.get());
	viewer->realize();
	viewer->run();
}
