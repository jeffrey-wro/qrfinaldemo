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
/*
	for(;;)
	{
    	Mat display, frame;
	
		cam >> frame; //get new frame
		cvtColor(frame, display, COLOR_BGR2GRAY);
		
		Mat pts;
		QRCodeDetector qrDecoder = QRCodeDetector();
		if(qrDecoder.detect(display, pts))
		{
			//set data to decoded qr code
			data = qrDecoder.detectAndDecode(display, pts);

			printf(data.c_str());
			fflush(stdout);
			
			if(data == "Forwards") //forwards
			{
				//set motor speed
				mc.setMotorSpeeds(DC, 50, -50);
				//wait for 3 seconds
				Utils::waitFor(3);
				//set speed 0
				mc.setMotorSpeeds(DC, 0, 0);

			}
			if(data == "Square") //move in a square
			{
				int leftCount = 0;
				int rightCount = 0;
				
				for(int i=0; i<4; i++){
					//move forward
					leftCount += 360;
					rightCount -= 360;
					mc.setMotorDegrees(DC, speed, leftCount, speed, 					rightCount);
					//wait for 3 seconds
					Utils::waitFor(3);

					//turn 90 degrees
					rightCount -= 465;
					mc.setMotorDegrees(DC, 0, leftCount, speed, rightCount);
					Utils::waitFor(3);
				}
			}
			if(data == "Backwards") //backwards
			{
				//set motor speed
				mc.setMotorSpeeds(DC, -50, 50);
				//wait for 3 seconds
				Utils::waitFor(3);
				//set speed 0
				mc.setMotorSpeeds(DC, 0, 0);
				
			}
			
			//delay and reset
			data = "0";
		}

	}
	*/
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
				
				if(data == "Forwards")
				{
					//add relevant code here
					Utils::waitFor(2);
					mc.controllerReset(DC);

					status = MyRio_Close();
					return status;
				}
				
				if(data == "Backwards")
				{
					//add relevant code here
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
