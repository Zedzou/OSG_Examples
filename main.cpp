#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/LineWidth>
#include <osgDB/ReadFile> 
#include "elements/CoordinateAxis.h"
#include "elements/Models.h"
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

	// 导入模型
	LoadModel* loadModel = new LoadModel();
	
	// 灯光
	Light* light = new Light();

	DrawCoordinate->addSubGroup(root);
	loadModel->addSubGroup(root);
	light->addSubGroup(root);

	// 导入数据
	DatasetLoader* datasetLoader = new DatasetLoader();
	std::map<std::string, Eigen::Matrix4f> poseMap = datasetLoader->poseMap;

	// // 测试
	// Eigen::Matrix3d testMatrix;
	// testMatrix <<  0.0406125, -0.999172,   0.00263255,
    //                0.999174,   0.0406159,  0.0012665, 
    //               -0.00137238, 0.00257894, 0.999996;
	// Eigen::Vector3d testEuler = RotationMatrixToEuler(testMatrix);
	// std::cout << testEuler << std::endl;

	// 测试Scan3r数据集


	// Eigen::Matrix4f transformer;
	// transformer <<  0.0123306, -0.999799,  0.0158277, -0.00961019,
	// 			   -0.297868,   0.0114376, 0.954539,  -0.00204422,
	// 			   -0.954527,  -0.0164846, -0.297667,  0.022526,
	// 				0, 0, 0, 1;
	// transformer.block<3, 1>(0, 3) *= 1000.0f;
	// transformer = m_poseTransform * transformer;
	// transformer.block<3, 1>(0, 3) /= 1000.0f;
	// std::cout << transformer << std::endl;

	// CameraDraw* cameraDraw = new CameraDraw(transformer);
	// cameraDraw->addSubGroup(root);

	for(auto &s:poseMap)
	{
		Eigen::Matrix4f m_poseTransform;
		m_poseTransform <<  0.0406125, -0.999172,  0.00263255, -940.349,
							0.999174,   0.0406159, 0.0012665,   1290.38,
							-0.00137238,  0.00257894, 0.999996, -154.145,
							0,           0,           0,           1;

		Eigen::Matrix4f transformer = s.second;
		transformer.block<3, 1>(0, 3) *= 1000.0f;
		transformer = m_poseTransform * transformer;
		transformer.block<3, 1>(0, 3) /= 1000.0f;

		CameraDraw* cameraDraw = new CameraDraw(transformer);
		cameraDraw->addSubGroup(root);
	}

	// // 旋转矩阵画相机
	// Eigen::Matrix4f transformer;
	// transformer << 1.0,   0.0,  0.0,  0.0,
	// 			   0.0,   0.0, -1.0,  0.0,
	// 			   0.0,   1.0,  0.0,  0.0,
	// 			   0.0,   0.0,  0.0,  1.0;
	// CameraDraw* cameraDraw = new CameraDraw(transformer);
	// cameraDraw->addSubGroup(root);

	// // 欧拉角画相机
	// Eigen::Vector3d euler;
	// euler << 1.2105561868276395,1.8045976376508266,1.565226885173696;
	// Eigen::Matrix3d RotationMatrix;
	// RotationMatrix = eulerToRotationMatrix(euler);
	// Eigen::Vector3d translation;
	// translation << -1.4177736030294625,-5.360134805572108,-8.37126018225108;
	// Eigen::Matrix4f transformer_2;
	// transformer_2 << RotationMatrix(0,0), RotationMatrix(0,1), RotationMatrix(0,2), translation(0),
	// 				 RotationMatrix(1,0), RotationMatrix(1,1), RotationMatrix(1,2), translation(1),
	// 				 RotationMatrix(2,0), RotationMatrix(2,1), RotationMatrix(2,2), translation(2),
	// 				 0,0,0,1;
	// CameraDraw* cameraDraw_2 = new CameraDraw(transformer_2);
	// cameraDraw_2->addSubGroup(root);

	// Eigen::Vector3d euler_2;
	// euler_2 << 1.32309351920077,1.7142897026125137,1.7968958237075034;
	// Eigen::Matrix3d RotationMatrix_2;
	// RotationMatrix_2 = eulerToRotationMatrix(euler_2);
	// Eigen::Vector3d translation_2;
	// translation_2 << -1.418843572347092,-1.0918791613911631,1.4614106718419673;
	// Eigen::Matrix4f transformer_3;
	// transformer_3 << RotationMatrix_2(0,0), RotationMatrix_2(0,1), RotationMatrix_2(0,2), translation_2(0),
	// 				 RotationMatrix_2(1,0), RotationMatrix_2(1,1), RotationMatrix_2(1,2), translation_2(1),
	// 				 RotationMatrix_2(2,0), RotationMatrix_2(2,1), RotationMatrix_2(2,2), translation_2(2),
	// 				 0,0,0,1;
	// CameraDraw* cameraDraw_3 = new CameraDraw(transformer_3);
	// cameraDraw_3->addSubGroup(root);

	// Eigen::Vector3d euler_3;
	// euler_3 << 1.2526161765127264,1.756890771030231,1.7153714390114514;
	// Eigen::Matrix3d RotationMatrix_3;
	// RotationMatrix_3 = eulerToRotationMatrix(euler_3);
	// Eigen::Vector3d translation_3;
	// translation_3 << -1.8785933449920251,1.7029067844271792,8.506519004317676;
	// Eigen::Matrix4f transformer_4;
	// transformer_4 << RotationMatrix_3(0,0), RotationMatrix_3(0,1), RotationMatrix_3(0,2), translation_3(0),
	// 				 RotationMatrix_3(1,0), RotationMatrix_3(1,1), RotationMatrix_3(1,2), translation_3(1),
	// 				 RotationMatrix_3(2,0), RotationMatrix_3(2,1), RotationMatrix_3(2,2), translation_3(2),
	// 				 0,0,0,1;
	// CameraDraw* cameraDraw_4 = new CameraDraw(transformer_4);
	// cameraDraw_4->addSubGroup(root);



	// viewer
	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer();
	viewer->addEventHandler(new osgViewer::WindowSizeHandler());
	viewer->setSceneData(root.get());
	viewer->realize();
	viewer->run();
}

// int main()
// {
// 	DatasetLoader* datasetLoader = new DatasetLoader();
// 	std::map<std::string, Eigen::Matrix4f> poseMap = datasetLoader->poseMap;
// 	for (auto &s : poseMap)
// 	{
// 		std::cout << s.second << std::endl << std::endl;
// 	}

// 	return 0;
// }

// #include<osgViewer/Viewer> 
// #include<osgDB/ReadFile> 
// int main()
// {
// 	osgViewer::Viewer viewer;//定义一个视景器
// 	osgDB::Options  *a = new osgDB::Options(std::string("noTriStripPolygons"));
// 	osg::ref_ptr<osg::Node> node = osgDB::readNodeFile("/home/zed/Project/GraphSLAM/test_dataset/Test_2/Models/labels.instances.annotated.v2.ply", a);
// 	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile("cow.osg");//使用readNodeFile读入模型文件
// 	viewer.setSceneData(node);//设置node给视景器作为、所有场景数据的根节点
// 	return viewer.run();//开始执行视景器，循环的绘制场景，并接受用户交互信息。  规划
// }


float size = 1.0;

// // 原始相机参数
// Eigen::Matrix<float, 5, 3> SetRotationTransformCamera(Eigen::Matrix<float, 4, 4> &transformer)
// {
// 	Eigen::Matrix<float, 4, 3> CameraVertex;
// 	CameraVertex<< -0.5f*size, -0.5f*size, 1*size, 
// 					0.5f*size, -0.5f*size, 1*size, 
// 					0.5f*size,  0.5f*size, 1*size,
// 					-0.5f*size,  0.5f*size, 1*size;

// 	Eigen::Matrix<float, 3, 3> Rotation; // 旋转
// 	Eigen::Matrix<float, 3, 1> Translation; // 平移
// 	Rotation = transformer.block<3, 3>(0, 0);
// 	Translation << transformer(0, 3), transformer(1, 3), transformer(2, 3);

// 	// 新相机的向量
// 	Eigen::Matrix<float, 4, 3> CameraVector = Rotation * CameraVertex.transpose() + Translation;

// 	// 相机原点
// 	Eigen::Vector3f originPoint;
// 	originPoint << transformer(0, 3), transformer(1, 3), transformer(2, 3);

// 	// 相机的4个点
// 	Eigen::Vector3f point_1, point_2, point_3, point_4;
// 	point_1(0) = transformer(0, 3) + CameraVector(0, 0);
// 	point_1(1) = transformer(1, 3) + CameraVector(0, 1);
// 	point_1(2) = transformer(2, 3) + CameraVector(0, 2); // point_1

// 	point_2(0) = transformer(0, 3) + CameraVector(1, 0);
// 	point_2(1) = transformer(1, 3) + CameraVector(1, 1);
// 	point_2(2) = transformer(2, 3) + CameraVector(1, 2); // point_2

// 	point_3(0) = transformer(0, 3) + CameraVector(2, 0);
// 	point_3(1) = transformer(1, 3) + CameraVector(2, 1);
// 	point_3(2) = transformer(2, 3) + CameraVector(2, 2); // point_3

// 	point_4(0) = transformer(0, 3) + CameraVector(3, 0);
// 	point_4(1) = transformer(1, 3) + CameraVector(3, 1);
// 	point_4(2) = transformer(2, 3) + CameraVector(3, 2); // point_4

// 	Eigen::Matrix<float, 5, 3> CameraModelVertex; // 相机的顶点
// 	CameraModelVertex << transformer(0, 3), transformer(1, 3), transformer(2, 3),
// 					point_1(0),point_1(1),point_1(2),
// 					point_2(0),point_2(1),point_2(2),
// 					point_3(0),point_3(1),point_3(2),
// 					point_4(0),point_4(1),point_4(2);
	
// 	return CameraModelVertex;
// };

// // 测试矩阵
// int main()
// {
// 	Eigen::Matrix<float, 4, 4> transformer;
// 	transformer << 0.0123306,-0.999799,0.0158277,-0.00961019,-0.297868,0.0114376,0.954539,-0.00204422,-0.954527,-0.0164846,-0.297667,0.022526,0.0,0.0,0.0,1.0;

// 	Eigen::Matrix<float, 4, 3> CameraVertex;
// 	CameraVertex<< -0.5f*size, -0.5f*size, 1*size, 0.5f*size, -0.5f*size, 1*size, 0.5f*size,  0.5f*size, 1*size, -0.5f*size,  0.5f*size, 1*size;

// 	Eigen::Matrix<float, 3, 3> Rotation; // 旋转
// 	Eigen::Matrix<float, 3, 1> Translation; // 平移
// 	Rotation = transformer.block<3, 3>(0, 0);
// 	Translation << transformer(0, 3), transformer(1, 3), transformer(2, 3);

// 	// 新相机的向量
// 	Eigen::Matrix<float, 3, 4> CameraVector = Rotation * CameraVertex.transpose();
// 	CameraVector.colwise() += Translation;

// 	// 相机原点
// 	Eigen::Vector3f originPoint;
// 	originPoint << transformer(0, 3), transformer(1, 3), transformer(2, 3);

// 	// 相机的4个点
// 	Eigen::Vector3f point_1, point_2, point_3, point_4;
// 	point_1 << CameraVector(0, 0), CameraVector(1, 0), CameraVector(2, 0);
// 	point_1 << CameraVector(0, 1), CameraVector(1, 1), CameraVector(2, 1);
// 	point_1 << CameraVector(0, 2), CameraVector(1, 2), CameraVector(2, 2);
// 	point_1 << CameraVector(0, 3), CameraVector(1, 3), CameraVector(2, 3);
// }