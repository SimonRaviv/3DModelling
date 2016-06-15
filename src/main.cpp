#define NOMINMAX
#include <windows.h>
#include <windowsx.h>
#include "3DModellingH.h"
#include "FileProcessingH.h"
#include "ICPAlgorithm.h"
#include "PointCloudSet.h"
#include "camera.h"

/* Single tone class - point cloud viewer */
class Main
{
	static Main *s_instance;

	Main() : viewer("PCL OpenNI Viewer")
	{
		this->record_flag = -1;
	}

public:
	visualization::CloudViewer viewer;
	Camera camera;
	ICPAlgorithm icp_Alg;
	FileProcessing fp;
	PointCloudSet point_cloud_list;
	int record_flag;


	static Main *instance()
	{
		if (!s_instance)
			s_instance = new Main;
		return s_instance;
	}

	void cloud_cb_(PointCloudConstPtr &cloud)
	{
		if (record_flag == 1)
		{
			this->point_cloud_list.it = this->point_cloud_list.set.insert(this->point_cloud_list.it, *cloud);
		}

		if (!viewer.wasStopped())
			viewer.showCloud(cloud);
	}

	void run()
	{
		boost::function<void(PointCloudConstPtr&)> f =
			boost::bind(&Main::cloud_cb_, this, _1);

		camera.registerCallback(f);

		camera.start();

		while (!viewer.wasStopped())
			boost::this_thread::sleep(boost::posix_time::seconds(1));

		camera.stop();
	}

};

/*Windows Main Function */
Main *Main::s_instance = 0;
HWND hwnd, recordButton, stopRecordButton, build3dmodelbutton;
//HINSTANCE hInstance;
LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
	LPSTR lpCmdLine, int nCmdShow)
{
	MSG  msg;
	WNDCLASS wc = { sizeof(wc) };
	wc.lpszClassName = TEXT("Static Control");
	wc.hInstance = hInstance;
	wc.hbrBackground = GetSysColorBrush(COLOR_3DFACE);
	wc.lpfnWndProc = WndProc;
	wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);

	RegisterClass(&wc);
	hwnd = CreateWindow(wc.lpszClassName, TEXT("Build 3D model using kinect "), WS_SYSMENU | WS_MINIMIZEBOX,
		100, 100, 500, 200, 0, 0, hInstance, 0);

	ShowWindow(hwnd, nCmdShow);
	UpdateWindow(hwnd);


	while (GetMessage(&msg, NULL, 0, 0))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}
	return (int)msg.wParam;
}

LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{

	switch (msg)
	{
	case WM_CREATE:
	{
		recordButton = CreateWindow("button", "Record",
			WS_VISIBLE | WS_CHILD | WS_BORDER,
			30, 100, 120, 40,
			hwnd, (HMENU)1, NULL, NULL);

		stopRecordButton = CreateWindow("button", "Stop record",
			WS_VISIBLE | WS_CHILD | WS_BORDER,
			180, 100, 120, 40,
			hwnd, (HMENU)2, NULL, NULL);
		EnableWindow(stopRecordButton, false);
		build3dmodelbutton = CreateWindow("button", "Build the scene",
			WS_VISIBLE | WS_CHILD | WS_BORDER,
			330, 100, 120, 40,
			hwnd, (HMENU)3, NULL, NULL);
		EnableWindow(build3dmodelbutton, false);
		break;
	}
	case WM_DESTROY:
	{
		PostQuitMessage(0);
		return 0;
	}
	case WM_COMMAND:
	{
		switch (LOWORD(wParam))
		{
		case 1:
		{
			vector<PointCloudT>::size_type j;
			vector<int> mapping;
			cout << "*** Video capturing started ***" << endl;

			Main::instance()->record_flag = 1;
			Main::instance()->run();
			cout << "*** Video capturing stopped ***" << endl;
			cout << "*** Filtering started ***" << endl;
			for (j = 0; j != Main::instance()->point_cloud_list.set.size(); j++)
				removeNaNFromPointCloud(Main::instance()->point_cloud_list.set[j], Main::instance()->point_cloud_list.set[j], mapping);
			cout << "*** Filtering stopped ***" << endl;
			EnableWindow(stopRecordButton, true);
			EnableWindow(recordButton, false);
			EnableWindow(build3dmodelbutton, false);
			break;
		}
		case 2:
		{
			Main::instance()->record_flag = 0;
			Main::instance()->camera.stop();
			EnableWindow(build3dmodelbutton, true);
			EnableWindow(recordButton, true);
			EnableWindow(stopRecordButton, false);
			break;
		}
		case 3:
		{
			Main::instance()->icp_Alg.register_with_result(Main::instance()->point_cloud_list.set, 30, 0.08);
			EnableWindow(build3dmodelbutton, false);
			EnableWindow(recordButton, false);
			EnableWindow(stopRecordButton, false);
			break;
		}
		default:
			break;
		}
	}

	}
	return DefWindowProc(hwnd, msg, wParam, lParam);
}
