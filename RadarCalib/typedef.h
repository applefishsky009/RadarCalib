#pragma once
namespace ADAS {
	struct WaveInCamera {
		int x;
		int y;
	};
	struct CameraPara {
		float fu;
		float fv;
		float cu;
		float cv;
		float pitch;
		float height;	// mm
		float yaw;
		float roll;
		int image_width;
		int image_height;
		float camera2wave_radian;
		WaveInCamera waveInCamera;
		int objectHeight;
		int objectWidth;
	};
};