#include <windows.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include "public.h"
#include "vjoyinterface.h"
#include <omp.h>

using namespace cv;
using namespace std;
Mat src; Mat src_gray;

int thresh = 100;
int max_thresh = 255;
int expo = 5;
int maxExp = 20;
int blurS = 5;
int maxblur = 40;
int width = 1280;
int height = 720;
int fps = 30;
int K0 = 100;
int K1 = 100;
int K2 = 100;
int K3 = 100;
int maxK = 200;
int Z = 100;
int appro = 1;
int opticalCenterX	 = 500;
int opticalCenterY	= 500;
int marqueeZoom		= 100;



std::string comPort = "COM6";

cv::Mat map1;
cv::Mat map2;
cv::Size sizeFrame = { width, height };

RNG rng(12345);
int drawframe = 1;

void save()
{
	// Declare what you need
	cv::FileStorage file("conf.yaml", cv::FileStorage::WRITE);
	// Write to file!
	file << "thresh"			<< thresh			;
	file << "expo"				<< expo				;
	file << "blurS"				<< blurS			;
	file << "width"				<< width			;
	file << "height"			<< height			;
	file << "appro"				<< appro			;
	file << "K0"				<< K0				;
	file << "K1"				<< K1				;
	file << "K2"				<< K2				;
	file << "K3"				<< K3				;
	file << "Z"					<< Z				;	
	file << "comPort"			<< comPort			;
	file << "opticalCenterX"	<< opticalCenterX	;	
	file << "opticalCenterY"	<< opticalCenterY	;
	file << "marqueeZoom"		<< marqueeZoom		;





	
}

void load()
{
	FileStorage fs("conf.yaml", FileStorage::READ);
	if (!fs.isOpened())
	{
		cerr << "failed to open " << "conf.yaml" << endl;
		return ;
	}
	thresh			= fs["thresh"			];
	expo			= fs["expo"				];
	blurS			= fs["blurS"			];
	width			= fs["width"			];
	height			= fs["height"			];
	appro			= fs["appro"			];
	K0				= fs["K0"				];
	K1				= fs["K1"				];
	K2				= fs["K2"				];
	K3				= fs["K3"				];
	Z				= fs["Z"				];
	comPort			= fs["comPort"			];
	opticalCenterX	= fs["opticalCenterX"	];
	opticalCenterY	= fs["opticalCenterY"	];
	marqueeZoom		= fs["marqueeZoom"		];

}



cv::Point2f ApplyPerspective(cv::Mat &M, const cv::Point2f& p)
{
	double* data = (double*)(M.data);
	float newX = data[0] * p.x + data[1] * p.y + data[2];
	float newY = data[3] * p.x + data[4] * p.y + data[5];
	float newZ = data[6] * p.x + data[7] * p.y + data[8];
	return cv::Point2f(newX / newZ, newY / newZ);
}


void ChangeExpo(int pos, void* userdata)
{
	VideoCapture*cap = (VideoCapture*)userdata;
	cap->set(CAP_PROP_EXPOSURE, -expo);
}

void ComputeDisto(int pos, void* userdata)
{
	//undist
	float Kdata[3][3] = { {height,	0.0f,	width / 2.0f	},
						  {0.0f,	height,	height / 2.0f	},
						  {0.0f,	0.0f,	1.0f		} };
	float Ddata[4] = { K0 / 100.0f - 1.0f,
						K1 / 100.0f - 1.0f,
						K2 / 100.0f - 1.0f,
						K3 / 100.0f - 1.0f };
	float Idata[3][3] = { {Z / 100.0f,	0.0f,	0.0f	},
						  {0.0f,	Z / 100.0f,	0.0f	},
						  {0.0f,	0.0f,	1.0f	} };
	Mat K = cv::Mat(3, 3, CV_32F, &Kdata);
	Mat D = cv::Mat(4, 1, CV_32F, &Ddata);
	Mat I = cv::Mat(3, 3, CV_32F, &Idata);
	cv::fisheye::initUndistortRectifyMap(K, D, I, K, sizeFrame, CV_16SC2, map1, map2);
}

/*
HANDLE serialHandle;
serial::Serial* my_serial;
bool openComPort(std::string id)
{
	my_serial = new serial::Serial(id, 9600, serial::Timeout::simpleTimeout(1000));
	if (my_serial->isOpen())
		return true;
	return false;
}

void closeComPort()
{
	delete my_serial;
}



void FillReportInLoop(JOYSTICK_POSITION* Report)
{
	if (!openComPort(comPort))
	{
		printf("Failed opening port %d.\n", comPort);
		exit(-3);
	}
	size_t curCount;
	while(true)
	{
		string buffer = my_serial->read(1);
		curCount = buffer.length();
		if (curCount == 1)
		{
			if (buffer[0] == 'D')
			{

				string buffer = "";
				while(buffer.length() != 22)
					buffer += my_serial->read(22 - buffer.length());
				//parse
				int buttonstate[6];
				int axisstate[2];
				sscanf_s(buffer.c_str(), "%d %d %d %d %d %d %d %d",
					&(buttonstate[0]),
					&(buttonstate[1]),
					&(buttonstate[2]),
					&(buttonstate[3]),
					&(buttonstate[4]),
					&(buttonstate[5]),
					&(axisstate[0]),
					&(axisstate[1])
				);
				Report->wAxisXRot = axisstate[0] * 32767 / 1024;
				Report->wAxisYRot = axisstate[1] * 32767 / 1024;
				for (int b = 0; b < 6; ++b)
					if (!buttonstate[b])
						Report->lButtons |= 0x1 << b;
					else
						Report->lButtons &= (0xFFFFFFFF - (0x1 << b));
			}
		}
	}
	closeComPort();
}
*/

HANDLE serialHandle;

bool openComPort()
{
	// Open serial port
	char buffer[256];
	sprintf_s(buffer, "\\\\.\\%s", comPort.c_str());
	serialHandle = CreateFileA(buffer, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

	if (serialHandle == (HANDLE)0xffffffffffffffff)
		return false;

	// Do some basic settings
	DCB serialParams = { 0 };
	serialParams.DCBlength = sizeof(serialParams);

	GetCommState(serialHandle, &serialParams);
	serialParams.BaudRate = 9600;
	serialParams.ByteSize = 8;
	serialParams.StopBits = 1;
	serialParams.Parity = 0;
	if (SetCommState(serialHandle, &serialParams) == 0)
		return false;

	// Set timeouts
	COMMTIMEOUTS timeout = { 0 };
	/*
	timeout.ReadIntervalTimeout = MAXDWORD;
	timeout.ReadTotalTimeoutConstant = 0;
	timeout.ReadTotalTimeoutMultiplier = 0;
	timeout.WriteTotalTimeoutConstant = 0;
	timeout.WriteTotalTimeoutMultiplier = 20000L / 9600;
	*/
	timeout.ReadIntervalTimeout = 50;
	timeout.ReadTotalTimeoutConstant = 50;
	timeout.ReadTotalTimeoutMultiplier = 50;
	timeout.WriteTotalTimeoutConstant = 50;
	timeout.WriteTotalTimeoutMultiplier = 10;

	if (SetCommTimeouts(serialHandle, &timeout) == 0)
		return false;

	return true;
}

void closeComPort()
{
	CloseHandle(serialHandle);
}


void FillReportInLoop(JOYSTICK_POSITION* Report)
{
	if (!openComPort())
	{
		printf("Failed opening port %d.\n", comPort);
		exit(-3);
	}

	OVERLAPPED overlap;
	DWORD curCount;
	char buffer[64];
	while (true)
	{
		//find D
		ReadFile(serialHandle, buffer, 1, &curCount, &overlap);
		if (curCount == 1)
		{
			if (buffer[0] == 'D')
			{
				//read byte				
				DWORD count = 0;
				while (count != 22)
				{
					ReadFile(serialHandle, buffer, 22 - count, &curCount, &overlap);
					count += curCount;
				}
				buffer[22] = 0;
				//parse
				int buttonstate[6];
				int axisstate[2];
				sscanf_s(buffer, "%d %d %d %d %d %d %d %d",
					&(buttonstate[0]),
					&(buttonstate[1]),
					&(buttonstate[2]),
					&(buttonstate[3]),
					&(buttonstate[4]),
					&(buttonstate[5]),
					&(axisstate[0]),
					&(axisstate[1])
				);
				Report->wAxisXRot = axisstate[0] * 32767 / 1024;
				Report->wAxisYRot = axisstate[1] * 32767 / 1024;
				for (int b = 0; b < 6; ++b)
					if (!buttonstate[b])
						Report->lButtons |= 0x1 << b;
					else
						Report->lButtons &= (0xFFFFFFFF - (0x1 << b));
			}
		}
	}
	closeComPort();
}





void doCam(JOYSTICK_POSITION *Report)
{
	int AxisMax = 32767;
	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened())  // check if we succeeded
		exit(-1);
	cap.set(CAP_PROP_FRAME_WIDTH, width);
	cap.set(CAP_PROP_FRAME_HEIGHT, height);
	cap.set(CAP_PROP_FPS, fps);

	namedWindow("frame", WINDOW_NORMAL);
	namedWindow("warped", WINDOW_NORMAL);
	namedWindow("param", WINDOW_NORMAL);
	createTrackbar("Thres:", "param", &thresh, max_thresh, NULL);
	createTrackbar("blur:", "param", &blurS, maxblur, NULL);
	createTrackbar("exp:", "param", &expo, maxExp, ChangeExpo, (void*)&cap);
	createTrackbar("appro:", "param", &appro, 40, NULL);
	createTrackbar("K0:", "param", &K0, maxK, ComputeDisto);
	createTrackbar("K1:", "param", &K1, maxK, ComputeDisto);
	createTrackbar("K2:", "param", &K2, maxK, ComputeDisto);
	createTrackbar("K3:", "param", &K3, maxK, ComputeDisto);
	createTrackbar("Zoom:", "param", &Z, maxK, ComputeDisto);
	createTrackbar("opticalCenterX", "param", &opticalCenterX, 1000, NULL);
	createTrackbar("opticalCenterY", "param", &opticalCenterY, 1000, NULL);
	createTrackbar("marqueeZoom", "param", &marqueeZoom, 100, NULL);

	//init stuff
	ComputeDisto(0, NULL);
	ChangeExpo(0, &cap);
	Point2f posJoy(0, 0);



	auto prev = chrono::steady_clock::now();
	for (;;)
	{
		auto start = chrono::steady_clock::now();



		Mat frame, uframe, nb, edge;
		Mat warped = cv::Mat(3, 3, CV_8UC3);
		cap >> frame; // get a new frame from camera
		if (frame.cols != 0)
		{


			cv::remap(frame, uframe, map1, map2, cv::INTER_LINEAR, BORDER_CONSTANT);


			if (blurS > 0)
				GaussianBlur(uframe, uframe, Size(0, 0), blurS / 10.0, blurS / 10.0);

			//edge detect
			cvtColor(uframe, nb, COLOR_BGR2GRAY);


			cv::threshold(nb, nb, thresh, 255, THRESH_BINARY);

			edge = nb.clone();

			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			vector<Point> bestapprox;
			float bestarea = 0;
			findContours(edge, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE, Point(0, 0));
			for (size_t i = 0; i < contours.size(); i++)
			{
				vector<Point> approx;
				approxPolyDP(contours[i], approx, appro, true);

				if (approx.size() == 4)
				{
					float area = -contourArea(approx, true);

					if (area < 0)
						cv::polylines(uframe, approx, true, Scalar(255, 0, 0), 1);
					else
						cv::polylines(uframe, approx, true, Scalar(0, 255, 255), 1);

					if (area > bestarea)
					{
						bestarea = area;
						bestapprox = approx;
					}
				}
				else
					cv::polylines(uframe, approx, true, Scalar(0, 0, 255), 1);


				//Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
				//drawContours(uframe, contours, (int)i, color, 2, 8, hierarchy, 0, Point());
			}

			Point center(width * opticalCenterX / 1000, height * opticalCenterY / 1000);
			Point wp(0, 0);

			Mat h;

			if (bestarea != 0)
			{
				//rotate approx such as first point is the closest to upperright corner using manhattan distance
				//FIXME: use an accelero...
				vector<std::pair<int, float> > distances;
				for (int i = 0; i < 4; ++i)
					distances.push_back(std::make_pair(i, bestapprox[i].x + bestapprox[i].y));
				std::sort(distances.begin(), distances.end(),
					[](const std::pair<int, float>& a, const std::pair<int, float>& b) -> bool
					{
						return a.second < b.second;
					});
				int rotate = distances[0].first;
				if (rotate != 0)
				{
					vector<Point> bestapproxRotated;
					for (int r = 0; r < 4; ++r)
						bestapproxRotated.push_back(bestapprox[(r + rotate) % 4]);
					bestapprox.swap(bestapproxRotated);
				}

				vector<Point> refpts = { {0,-0}, {0,500}, {500,500}, {500,0} };
				cv::polylines(uframe, bestapprox, true, Scalar(0, 255, 0), 2);
				h = findHomography(bestapprox, refpts);
				wp = ApplyPerspective(h, center);
				float marqueeSize = 250.0f * marqueeZoom / 100.0f;
				posJoy = Point2f((wp.x - 250) / marqueeSize, (wp.y - 250) / marqueeSize);				
				Report->wAxisX = (std::max(std::min(posJoy.x, 1.0f), -1.0f) + 1.0f) * (AxisMax / 2);
				Report->wAxisY = (std::max(std::min(posJoy.y, 1.0f), -1.0f) + 1.0f) * (AxisMax / 2);

			}
			auto end = chrono::steady_clock::now();
			int elapsed = chrono::duration_cast<chrono::milliseconds>(end - start).count();
			int ips = 0;// 1000000000 / chrono::duration_cast<chrono::nanoseconds>(start - prev).count();
			prev = start;

			circle(uframe, center, 20, (255, 150, 150), 1);
			char buffer[255];
			sprintf_s(buffer, "%d ms / %d fps", elapsed, ips);
			putText(uframe, buffer, Point(10, 10), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0), 10);
			putText(uframe, buffer, Point(10, 10), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);

			sprintf_s(buffer, "pos: %.2f / %.2f", posJoy.x, posJoy.y);
			putText(uframe, buffer, Point(10, 40), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0), 10);
			putText(uframe, buffer, Point(10, 40), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);



			if (drawframe == 1)
			{
				imshow("frame", uframe);
				if (bestarea != 0)
				{
					warpPerspective(uframe, warped, h, Size(500, 500));
					circle(warped, wp, 5, (0, 0, 255), -1);
					vector<Point> zoomedMarquee;
					float halfW = warped.cols / 2;
					float halfH = warped.rows / 2;
					float Mw = (halfW * marqueeZoom / 100);
					float Mh = (halfH * marqueeZoom / 100);
					zoomedMarquee.push_back(Point(halfW - Mw, halfH - Mh));
					zoomedMarquee.push_back(Point(halfW - Mw, halfH + Mh));
					zoomedMarquee.push_back(Point(halfW + Mw, halfH + Mh));
					zoomedMarquee.push_back(Point(halfW + Mw, halfH - Mh));
					cv::polylines(warped, zoomedMarquee, true, Scalar(0, 255, 255), 2);


					imshow("warped", warped);
				}
			}


			//send to vjoy
			UpdateVJD(Report->bDevice, (PVOID)Report);

			int key = waitKey(1);
			if (key == 'l') load();
			if (key == 's') save();
			if (key == 'q') exit(0);
			if (key == 'v') drawframe = (drawframe + 1) % 2;
		}
	}
}


int main(int, char** argv)
{
	load();

	//init vjoy
	// Get the driver attributes (Vendor ID, Product ID, Version Number)
	if (!vJoyEnabled())
	{
		printf("Failed Getting vJoy attributes.\n");
		return -2;
	}
	else
	{
		printf("Vendor: %S\nProduct :%S\nVersion Number:%S\n",
			(wchar_t*)(GetvJoyManufacturerString()),
			(wchar_t*)(GetvJoyProductString()),
			(wchar_t*)(GetvJoySerialNumberString()));
	}
	// Test interface DLL matches vJoy driver
	WORD VerDll, VerDrv;
	if (!DriverMatch(&VerDll, &VerDrv))
		printf("Failed\r\nvJoy Driver (version %04x) does not match vJoyInterface DLL (version %04x)\n", VerDrv, VerDll);
	else
		printf("OK - vJoy Driver and vJoyInterface DLL match vJoyInterface DLL (version %04x)\n", VerDrv);

	//get Vjoy
	int count = 0;
	int vjoyId = -1;
	GetNumberExistingVJD(&count);
	for (int i = 1; i <= count && vjoyId == -1; ++i)
	{
		// Get the state of the requested device (iInterface)
		VjdStat status = GetVJDStatus(i);
		switch (status)
		{
		case VJD_STAT_OWN:
			printf("vJoy Device %d is already owned by this feeder\n", i);
			vjoyId = i;
			break;
		case VJD_STAT_FREE:
			printf("vJoy Device %d is free\n", i);
			if (!AcquireVJD(i))
			{
				printf("Failed to acquire vJoy device number %d.\n", i);
				return -1;
			}
			else
				printf("Acquired: vJoy device number %d.\n", i);
			vjoyId = i;
			break;
		case VJD_STAT_BUSY:
			printf("vJoy Device %d is already owned by another feeder\n	Cannot continue\n", i);
			break;
		case VJD_STAT_MISS:
			printf("vJoy Device %d is not installed or disabled\n	Cannot continue\n", i);
			break;
		default:
			printf("vJoy Device %d general error ->Cannot continue\n", i);
			break;
		};		
	}
	if(vjoyId == -1)
	{
		printf("Unable to acquire a free vJoy.\n");
		return -2;
	}
	ResetVJD(vjoyId);


	JOYSTICK_POSITION Report;
	memset(&Report, 0, sizeof(JOYSTICK_POSITION));
	Report.bDevice = vjoyId;


	#pragma omp parallel sections
	{
		#pragma omp section
		{
			doCam(&Report);
		}
		#pragma omp section
		{
			FillReportInLoop(&Report);
		}
	}

	

	

	return(0);
}

