//====================================================================//
// Created by liheng on 19-3-7.
//Program:��̬������ز������������ŵ���Σ��õ����ײ����굽ͼ�������ת��
//Data:2019.3.7
//Author:liheng
//Version:V1.0
//====================================================================//

//====================================================================//
// Reproduced by renyili on 20-12-3.
// Program:��̬������ز������������ŵ���Σ��õ����ײ����굽ͼ�������ת��
// Data:2020.12.3
// Author:renyili
// Version:V1.0
//====================================================================//

#include "typedef.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "WaveRadar2Image.h"
#include "CGetVideo.h"

//�������
//CGetVideo m_getVideo(18072414); // ����������:17369069  12.5��18072414
CGetVideo m_getVideo("D:/visual/RadarCalib/road.mp4");

void onChange(int value, void* param) {
	//��ȡwaveX,waveY
	//X:0-10,��Ӧ-5--5
	//Y:0-10 ��Ӧ0-50
	float k_waveX = (-5 - 5.0) / (0 - 10.0);
	float b_waveX = -5.0 - k_waveX * 0;
	float waveX = (float)cv::getTrackbarPos("waveX/m", "TrackBar");
	waveX = k_waveX * waveX + b_waveX;

	float waveY = (float)cv::getTrackbarPos("waveY/m", "TrackBar");

	//��ȡpitch,yaw�Ƕ�
	//0-3000,0��Ӧ-30.0;3000��Ӧ30.0;
	//x     0       3000.0
	//y    -30.0     30.0
	float k_pitch = (-30.0 - 30.0) / (0 - 3000.0);
	float b_pitch = -30.0 - k_pitch * 0;
	float pitch = (float)cv::getTrackbarPos("pitch/��", "TrackBar");
	pitch = k_pitch * pitch + b_pitch;

	float k_yaw = (-30.0 - 30.0) / (0 - 3000.0);
	float b_yaw = -30.0 - k_yaw * 0;
	float yaw = (float)cv::getTrackbarPos("yaw/��", "TrackBar");
	yaw = k_yaw * yaw + b_yaw;

	//��ȡtheta
	float k_theta = (-30.0 - 30.0) / (0 - 3000.0);
	float b_theta = -30.0 - k_theta * 0;

	float theta = (float)cv::getTrackbarPos("theta/��", "TrackBar");
	theta = k_theta * theta + b_theta;

	//====================ͼ����========================//
	cv::Mat posInImage, posInCamera;
	cv::Mat src = *(cv::Mat*)param;
	cv::Mat dst; dst.release();
	{
		ADAS::CameraPara cameraPara;
		// ����ڲ�
		cameraPara.fu = 1007.843;
		cameraPara.fv = 1014.159;
		cameraPara.cu = 649.104;
		cameraPara.cv = 360.322;
		// ������
		cameraPara.height = 1750; // mm ��������߶�
		cameraPara.pitch = pitch * (CV_PI * 1.0 / 180.0); // ����
		cameraPara.yaw = yaw * (CV_PI * 1.0 / 180.0);
		cameraPara.roll = 0;
		// ͼ���С
		cameraPara.image_width = 1280;
		cameraPara.image_height = 720;

		//waveY = 22.0;
		cameraPara.camera2wave_radian = theta * (CV_PI * 1.0 / 180.0); // �״���������ϵ�ĺ�ڽ�
		cameraPara.waveInCamera.x = 0; // mm �״����������ϵ�е�Xƽ��;
		cameraPara.waveInCamera.y = 0; // mm �״����������ϵ�е�Yƽ��;
		cameraPara.objectHeight = 1000.0; //mm �����Ŀ��߶� ���˳���1800
		cameraPara.objectWidth = 1500.0; //mm �����Ŀ���� ���˳���1600
		//cameraPara.objectHeight = 1800.0; //mm �����Ŀ��߶� ���˳���1800
		//cameraPara.objectWidth = 1500.0; //mm �����Ŀ���� ���˳���1600

		cv::Mat posInWave = (cv::Mat_<float>(2, 1) <<
			waveX,
			waveY);	// �״������

		// ���״�㴫��ȥ��ת��Ϊ�������ϵ��ͼ������ϵ
		WaveRadar2Image waveRadar2Image(cameraPara);
		waveRadar2Image.TransformWRadar2Image2(posInWave, posInImage, posInCamera);

		dst = src.clone();

		if (dst.channels() == 1)
			cv::cvtColor(dst, dst, CV_GRAY2BGR);

		cv::Mat temp(100, dst.cols, CV_8UC3, cv::Scalar(255, 255, 255));
		cv::vconcat(dst, temp, dst);

		int nObjectNum = posInImage.cols;
		for (int i = 0; i != nObjectNum; ++i) {
			// ���״������� x1, y1, w, h
			cv::Rect pt = cv::Rect(posInImage.at<float>(0, i),
				posInImage.at<float>(1, i),
				posInImage.at<float>(2, i),
				posInImage.at<float>(3, i));
			cv::rectangle(dst, pt, cv::Scalar(0, 255, 0), 2);
			//cv::rectangle(dst, cv::Rect(0,
			//	0,
			//	1280,
			//	720), cv::Scalar(0, 255, 0), 2);
			// ��Ŀ�����ĵ�
			cv::circle(dst, cv::Point(posInImage.at<float>(0, i) + posInImage.at<float>(2, i) / 2,
				posInImage.at<float>(1, i) + posInImage.at<float>(3, i) / 2), 3, cv::Scalar(0, 255, 0), 4);
			//std::cout<<"width in pixel="<<posInImage.at<float>(2,i)<<std::endl;
		}

		// ����ƫ���� ���ĺ��
		cv::circle(dst, cv::Point(cameraPara.cu, cameraPara.cv), 6, cv::Scalar(0, 0, 255), 10);
	}

	//cv::rectangle(dst,cv::Point(0,0),cv::Point(220,110),cv::Scalar(255,255,0),1);

	// ��ͼ������
	int nHeight = dst.rows;
	int nWidth = dst.cols;
	cv::line(dst, cv::Point(nWidth / 2, 0), cv::Point(nWidth / 2, nHeight), cv::Scalar(0, 0, 255), 2);
	cv::line(dst, cv::Point(0, nHeight / 2), cv::Point(nWidth, nHeight / 2), cv::Scalar(0, 0, 255), 2);

	cv::line(dst, cv::Point(nWidth / 4, 0), cv::Point(nWidth / 4, nHeight), cv::Scalar(0, 0, 255), 1);
	cv::line(dst, cv::Point(nWidth * 3 / 4, 0), cv::Point(nWidth * 3 / 4, nHeight), cv::Scalar(0, 0, 255), 1);

	cv::line(dst, cv::Point(0, nHeight / 4), cv::Point(nWidth, nHeight / 4), cv::Scalar(0, 0, 255), 1);
	cv::line(dst, cv::Point(0, nHeight * 3 / 4), cv::Point(nWidth, nHeight * 3 / 4), cv::Scalar(0, 0, 255), 1);

	// ��ʾ���Ͻǵ�ǰ����
	char info[256];
	sprintf_s(info, "waveX:%.2f", waveX);
	cv::putText(dst, info, cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

	sprintf_s(info, "waveY:%.2f", waveY);
	cv::putText(dst, info, cv::Point(0, 45), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

	sprintf_s(info, "theta:%.1f", theta);
	cv::putText(dst, info, cv::Point(0, 70), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

	// �״�����ת���ɵ��������ϵ
	sprintf_s(info, "cameraX:%.2f", posInCamera.at<float>(0, 0));
	cv::putText(dst, info, cv::Point(0, 95), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

	sprintf_s(info, "cameraY:%.2f", posInCamera.at<float>(1, 0));
	cv::putText(dst, info, cv::Point(0, 120), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

	sprintf_s(info, "pitch:%.1f", pitch);
	cv::putText(dst, info, cv::Point(0, 145), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

	sprintf_s(info, "yaw:%.1f", yaw);
	cv::putText(dst, info, cv::Point(0, 170), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

	//cv::Mat showimg;
	//cv::resize(dst, showimg, cv::Size(640, 360));
	cv::imshow("TrackBar", dst);
}

int main() {
	cv::Mat src;
	//src = cv::imread("../000658.png",0);//����Ҷ�ͼ
	cv::namedWindow("TrackBar", CV_WINDOW_AUTOSIZE);
	//cv::namedWindow("TrackBar", CV_WINDOW_NORMAL);

	//����������
	int waveX = 5;
	cv::createTrackbar("waveX/m", "TrackBar", &waveX, 10, onChange, &src);  // ��ʼֵ5����Χ10
	int waveY = 22;
	cv::createTrackbar("waveY/m", "TrackBar", &waveY, 50, onChange, &src);  // ��ʼֵ5����Χ10

	//���ײ������
	int theta = 1500;
	cv::createTrackbar("theta/��", "TrackBar", &theta, 3000, onChange, &src);  // ��ʼֵ1500����Χ3000

	int pitch = 1500;
	cv::createTrackbar("pitch/��", "TrackBar", &pitch, 3000, onChange, &src);  // ��ʼֵ1500����Χ3000
	int yaw = 1500;
	cv::createTrackbar("yaw/��", "TrackBar", &yaw, 3000, onChange, &src);  // ��ʼֵ1500����Χ3000

	//onChange(0, &src);

	//������Ƶ���в���
	//cv::VideoCapture videoCapture;
	//videoCapture.open("/home/zxkj/��Ƶ/18_11_11_11_1_59.avi");
	//videoCapture.open("/home/liheng/CLionprojects/kitti/01/left/%06d.png");
	//if( !videoCapture.isOpened() )
	//    return -1;

	int nWaitTime = 0;
	while (true) {
		src.release();
		m_getVideo.GetVideoFrame(src);
		//videoCapture >> src;
		if (src.empty())
			break;

		onChange(0, &src);

		char chKey = cv::waitKey(nWaitTime);
		if (chKey == 27) 
			break;
		if (chKey == ' ') 
			nWaitTime = !nWaitTime;
	}
	return 0;
}