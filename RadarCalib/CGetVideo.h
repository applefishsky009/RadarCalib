#pragma once
#include <string>
#include <iostream>
#include "typedef.h"
#include <opencv2/opencv.hpp>
class CGetVideo {
public:
	// ���캯�� - ��Ƶ
	CGetVideo(std::string video_path) {
		std::cout << "CGetVideo mp4 initing..." << std::endl;
		vi_path = video_path;
		videoCapture.open(vi_path);
	}
	// ���캯�� - �������
	CGetVideo(int nPort) {
		std::cout << "CGetVideo cam initing..." << std::endl;
		;
	}
	// ȡ֡����
	void GetVideoFrame(cv::Mat &src) {
		videoCapture >> src;
		//������Ƶ���в���
		//cv::VideoCapture videoCapture;
		//videoCapture.open("/home/zxkj/��Ƶ/18_11_11_11_1_59.avi");
		//videoCapture.open("/home/liheng/CLionProjects/kitti/01/left/%06d.png");
		//if( !videoCapture.isOpened() )
		//    return -1;
		;
	}
private:
	std::string vi_path;
	cv::VideoCapture videoCapture;
};