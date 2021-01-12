#pragma once
#include <cmath>
#include "typedef.h"
#include <opencv2/opencv.hpp>
class WaveRadar2Image {
public:
	// 构造函数
	WaveRadar2Image(ADAS::CameraPara &cam) {
		std::cout << "WaveRadar2Image func initing..." << std::endl;
		memcpy(&cameraPara, &cam, sizeof(ADAS::CameraPara));
	}
	// 转换函数
	void TransformWRadar2Image2(cv::Mat &posInWave, cv::Mat &posInImage, cv::Mat &posInCamera) {
		// 将雷达坐标转换到相机为中心的世界坐标 posInCamera存入cameraX和cameraY
		float waveX = posInWave.at<float>(0, 0);
		float waveY = posInWave.at<float>(1, 0);
		float theta = cameraPara.camera2wave_radian;
		float waveInCamera_x = cameraPara.waveInCamera.x;
		float waveInCamera_y = cameraPara.waveInCamera.y;
		double posInCamera_x = cos(theta) * waveX - sin(theta) * waveY + waveInCamera_x;
		double posInCamera_y = sin(theta) * waveX + cos(theta) * waveY + waveInCamera_y;
		//posInCamera << posInCamera_x, posInCamera_y;
		posInCamera = cv::Mat_<float>(2, 1);
		posInCamera.at<float>(0, 0) = posInCamera_x;
		posInCamera.at<float>(1, 0) = posInCamera_y;

		// 将相机坐标转换到图像坐标 posInImage存入x1, y1, x2, y2
		// 相机外参
		float pitch_a = cameraPara.pitch;
		float yaw_b = cameraPara.yaw;
		float height = cameraPara.height;
		// 相机内参
		float fu = cameraPara.fu;
		float fv = cameraPara.fv;
		float cu = cameraPara.cu;
		float cv = cameraPara.cv;
		float c1 = cos(pitch_a), c2 = cos(yaw_b), s1 = sin(pitch_a), s2 = sin(yaw_b);
		float oheight = cameraPara.objectHeight;
		float owidth = cameraPara.objectWidth;
		// 目标点中心世界坐标 (posInCamera_x, posInCamera_y, -oheight / 2)
		
		// 左上 (posInCamera_x - owidth / 2, posInCamera_y, 0)
		double point_left_top_x = posInCamera_x * 1000 - owidth / 2;
		double point_left_top_y = posInCamera_y * 1000;
		double point_left_top_z = 0;
		double posInImage_left_top_x = cal_x(fu, fv, cu, cv, c1, c2, s1, s2, 
			point_left_top_x, point_left_top_y, point_left_top_z);
		double posInImage_left_top_y = cal_y(fu, fv, cu, cv, c1, c2, s1, s2, 
			point_left_top_x, point_left_top_y, point_left_top_z);

		// 右下 (posInCamera_x + owidth / 2, posInCamera_y, -oheight)
		double point_right_bottom_x = posInCamera_x * 1000 + owidth / 2;
		double point_right_bottom_y = posInCamera_y * 1000;
		double point_right_bottom_z = -oheight;
		double posInImage_right_bottom_x = cal_x(fu, fv, cu, cv, c1, c2, s1, s2,
			point_right_bottom_x, point_right_bottom_y, point_right_bottom_z);
		double posInImage_right_bottom_y = cal_y(fu, fv, cu, cv, c1, c2, s1, s2,
			point_right_bottom_x, point_right_bottom_y, point_right_bottom_z);
		//posInImage << posInImage_left_top_x << posInImage_left_top_y << posInImage_right_bottom_x << posInImage_right_bottom_y;
		posInImage = cv::Mat_<float>(4, 1);
		posInImage.at<float>(0, 0) = posInImage_left_top_x;
		posInImage.at<float>(1, 0) = posInImage_left_top_y;
		posInImage.at<float>(2, 0) = posInImage_right_bottom_x - posInImage_left_top_x;
		posInImage.at<float>(3, 0) = posInImage_right_bottom_y - posInImage_left_top_y;
		//int kkk = 0;
	}
private:
	ADAS::CameraPara cameraPara;
	double cal_x(	float fu, float fv, float cu, float cv, 
					float c1, float c2, float s1, float s2,
					double x, double y, double z) {
		double top = (fu * c2 + cu * c1 * s2) * x + (cu * c1 * c2 - s2 * fu) * y - (cu * s1) * z;
		double down = c1 * s2 * x + c1 * c2 * y - s1 * z;
		return top / down;
	}
	double cal_y(	float fu, float fv, float cu, float cv,
					float c1, float c2, float s1, float s2,
					double x, double y, double z) {

		double top = s2 * (cv * c1 - fv * s1) * x + c2 * (cv * c1 - fv * s1) * y - (fv * c1 + cv * s1) * z;
		double down = c1 * s2 * x + c1 * c2 * y - s1 * z;
		return top / down;
	}
};