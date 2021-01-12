#pragma once
#include <string>
#include <iostream>
#include "typedef.h"
#include <opencv2/opencv.hpp>
class CGetVideo {
public:
	// 构造函数 - 视频
	CGetVideo(std::string video_path) {
		std::cout << "CGetVideo mp4 initing..." << std::endl;
		vi_path = video_path;
		videoCapture.open(vi_path);
	}
	// 构造函数 - 网口相机
	CGetVideo(int nPort) {
		std::cout << "CGetVideo cam initing..." << std::endl;
		;
	}
	// 取帧函数
	void GetVideoFrame(cv::Mat &src) {
		videoCapture >> src;
		//加载视频进行测试
		//cv::VideoCapture videoCapture;
		//videoCapture.open("/home/zxkj/视频/18_11_11_11_1_59.avi");
		//videoCapture.open("/home/liheng/CLionProjects/kitti/01/left/%06d.png");
		//if( !videoCapture.isOpened() )
		//    return -1;
		;
	}
private:
	std::string vi_path;
	cv::VideoCapture videoCapture;
};