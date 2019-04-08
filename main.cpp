#include <vector>
#include <iostream>

#include <stdio.h>

#include "MyRio.h"
#include "I2C.h"
#include "Motor_Controller.h"
#include "Utils.h"

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/operations.hpp"

#include "ImageSender.h"

using namespace std;
using namespace cv;

#define ENABLE_SERVER 1

extern NiFpga_Session myrio_session;
NiFpga_Status status;

int main(int argc, char **argv) {
	int speed = 200;
	VideoCapture cam(0); // open the default camera
	if (!cam.isOpened())  // check if we succeeded
		return -1;

	status = MyRio_Open();
	if (MyRio_IsNotSuccess(status)) {
		return status;
	}

	MyRio_I2c i2c;
	status = Utils::setupI2CB(&myrio_session, &i2c);

	Motor_Controller mc = Motor_Controller(&i2c);
	mc.controllerEnable(DC);

	int volt = mc.readBatteryVoltage(1);
	printf("%d\n\n", volt);

	string data;
	Mat display, frame, pts;
	QRCodeDetector qrDecoder = QRCodeDetector();
	
	while(1)
	{
		cam >> frame; //get new frame
		cvtColor(frame, display, COLOR_BGR2GRAY);
		
		if(qrDecoder.detect(display, pts))
		{
			while(1)
			{
				data = qrDecoder.detectAndDecode(display, pts);
				
				if(data == "Ver1")
				{
					//move backwards
					leftCount = -360;
					rightCount = 360;
					mc.setMotorDegrees(DC, speed, leftCount, speed, rightCount);
					
					//wait for 3 seconds
					Utils::waitFor(3);
					
					//turn 90 degrees left
					rightCount = -485;
					mc.setMotorDegrees(DC, 0, leftCount, speed, rightCount);
					
					//wait for 3 seconds
					Utils::waitFor(3);
					
					//move forwards
					leftCount = 360;
					rightCount = -360;
					mc.setMotorDegrees(DC, speed, leftCount, speed, rightCount);
					
					//turn 90 degrees right
					rightCount = 485; //fix
					mc.setMotorDegrees(DC, 0, leftCount, speed, rightCount);
					
					//cleanup
					Utils::waitFor(3);
					mc.controllerReset(DC);

					status = MyRio_Close();
					return status;
				}
				
				if(data == "right")
				{
					//add relevant code here
					
					//move backwards
					leftCount += 360;
					rightCount -= 360;
					mc.setMotorDegrees(DC, speed, leftCount, speed, rightCount);
					
					//wait for 3 seconds
					Utils::waitFor(3);
					
					//turn 90 degrees
					rightCount -= 465;
					mc.setMotorDegrees(DC, 0, leftCount, speed, rightCount);
					Utils::waitFor(3);
					
					
					Utils::waitFor(2);
					mc.controllerReset(DC);

					status = MyRio_Close();
					return status;					
				}
			}
		}
	}
	
	Utils::waitFor(2);
	mc.controllerReset(DC);

	status = MyRio_Close();

	return status;
}
