#define NOMINMAX
#include <windows.h>

#include "3DModellingH.h"
#include "FileProcessingH.h"
#include "ICPAlgorithm.h"

class PointCloudSet
{
public:
	vector<PointCloudT> set;
	vector<PointCloudT>::iterator it;

	PointCloudSet();

	~PointCloudSet();

private:

};

PointCloudSet::PointCloudSet()
{
	this->it = set.begin();
}
PointCloudSet::~PointCloudSet()
{
}

class SimpleOpenNIViewer
{
    static SimpleOpenNIViewer *s_instance;
	SimpleOpenNIViewer() : viewer("PCL OpenNI Viewer")
	{
	this->camera = new OpenNI2Grabber();
	this->counter = -1;
	}
  public:
  visualization::CloudViewer viewer;
  Grabber* camera;
  PointCloudSet point_cloud_list;
  int counter;

    static SimpleOpenNIViewer *instance()
    {
        if (!s_instance)
          s_instance = new SimpleOpenNIViewer;
        return s_instance;
    }

	void cloud_cb_(PointCloudConstPtr &cloud)
	{
		if (counter == 1)
		{
			this->point_cloud_list.it = this->point_cloud_list.set.insert(this->point_cloud_list.it, *cloud);
		}

		if (!viewer.wasStopped())
			viewer.showCloud(cloud);
	}

	void run()
	{
		boost::function<void(PointCloudConstPtr&)> f =
			boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

		this->camera->registerCallback(f);

		this->camera->start();

		while (!viewer.wasStopped())
			boost::this_thread::sleep(boost::posix_time::seconds(1));

		this->camera->stop();
	}

};
SimpleOpenNIViewer *SimpleOpenNIViewer::s_instance = 0;
int rigthTimeClicked =1;
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
	wc.lpfnWndProc = WndProc; // error: undefined reference to `WndProc(HWND__*, unsigned int, unsigned int, long)@16'
	wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);

	RegisterClass(&wc);
	hwnd = CreateWindow(wc.lpszClassName, TEXT("Build 3D model using kinect "),
		WS_OVERLAPPEDWINDOW,
		100, 100, 500, 270, 0, 0, hInstance, 0);

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
			30, 160, 120, 40,
			hwnd, (HMENU)1, NULL, NULL);

		stopRecordButton = CreateWindow("button", "Stop Record",
			WS_VISIBLE | WS_CHILD | WS_BORDER,
			180, 160, 120, 40,
			hwnd, (HMENU)2, NULL, NULL);

		build3dmodelbutton = CreateWindow("button", "build the sciene",
			WS_VISIBLE | WS_CHILD | WS_BORDER ,
			330, 160, 120, 40,
			hwnd, (HMENU)3, NULL, NULL);
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
			if (rigthTimeClicked == 1)
			{
			FileProcessing fp;
			vector<PointCloudT>::size_type j;
			vector<int> mapping;
			cout << "*** Video capturing started ***" << endl;

			SimpleOpenNIViewer::instance()->counter = 1;
			SimpleOpenNIViewer::instance()->run();
			cout << "*** Video capturing stopped ***" << endl;
			cout << "*** Filtering started ***" << endl;
			for (j = 0; j != SimpleOpenNIViewer::instance()->point_cloud_list.set.size(); j++)
				removeNaNFromPointCloud(SimpleOpenNIViewer::instance()->point_cloud_list.set[j], SimpleOpenNIViewer::instance()->point_cloud_list.set[j], mapping);
			cout << "*** Filtering stopped ***" << endl;
			rigthTimeClicked =2;
			}

			break;
		}
		case 2:
		{
			if (rigthTimeClicked == 2)
			{
			SimpleOpenNIViewer::instance()->counter = 0;
			SimpleOpenNIViewer::instance()->camera->stop();
			rigthTimeClicked = 3;
			}
			break;
		}
		case 3:
		{
			if (rigthTimeClicked == 3)
			{

			ICPAlgorithm icp_Alg;
			icp_Alg.register_with_result(SimpleOpenNIViewer::instance()->point_cloud_list.set, 20, 0.05);

			}
			break;
		}
		default:
			break;
		}
	}

	}
	return DefWindowProc(hwnd, msg, wParam, lParam);
}
