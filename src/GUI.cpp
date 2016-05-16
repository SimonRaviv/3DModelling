//#include "GUI.h"
//
//class SimpleOpenNIViewer
//{
//	static SimpleOpenNIViewer *s_instance;
//	SimpleOpenNIViewer() : viewer("PCL OpenNI Viewer")
//	{
//		this->camera = new OpenNI2Grabber();
//		this->record_flag = -1;
//	}
//public:
//	visualization::CloudViewer viewer;
//	Grabber* camera;
//	PointCloudSet point_cloud_list;
//	int record_flag;
//
//	static SimpleOpenNIViewer *instance()
//	{
//		if (!s_instance)
//			s_instance = new SimpleOpenNIViewer;
//		return s_instance;
//	}
//
//	void cloud_cb_(PointCloudConstPtr &cloud)
//	{
//		if (record_flag == 1)
//		{
//			this->point_cloud_list.it = this->point_cloud_list.set.insert(this->point_cloud_list.it, *cloud);
//		}
//
//		if (!viewer.wasStopped())
//			viewer.showCloud(cloud);
//	}
//
//	void run()
//	{
//		boost::function<void(PointCloudConstPtr&)> f =
//			boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);
//
//		this->camera->registerCallback(f);
//
//		this->camera->start();
//
//		while (!viewer.wasStopped())
//			boost::this_thread::sleep(boost::posix_time::seconds(1));
//
//		this->camera->stop();
//	}
//
//};
//
//GUI::GUI()
//{
//}
//
//GUI::~GUI()
//{
//}
//
//SimpleOpenNIViewer *SimpleOpenNIViewer::s_instance = 0;
//
//int WINAPI GUI::simon(HINSTANCE hInstance, HINSTANCE hPrevInstance,
//	LPSTR lpCmdLine, int nCmdShow)
//{
//	MSG  msg;
//	GUI g;
//	WNDCLASS wc = { sizeof(wc) };
//	wc.lpszClassName = TEXT("Static Control");
//	wc.hInstance = hInstance;
//	wc.hbrBackground = GetSysColorBrush(COLOR_3DFACE);
//	wc.lpfnWndProc = g.WndProc ; // error: undefined reference to `WndProc(HWND__*, unsigned int, unsigned int, long)@16'
//	wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
//	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
//
//	RegisterClass(&wc);
//	hwnd = CreateWindow(wc.lpszClassName, TEXT("Build 3D model using kinect "), WS_SYSMENU | WS_MINIMIZEBOX,
//		100, 100, 700, 270, 0, 0, hInstance, 0);
//
//	ShowWindow(hwnd, nCmdShow);
//	UpdateWindow(hwnd);
//
//
//	while (GetMessage(&msg, NULL, 0, 0))
//	{
//		TranslateMessage(&msg);
//		DispatchMessage(&msg);
//	}
//	return (int)msg.wParam;
//}
//
//LRESULT CALLBACK GUI::WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
//{
//
//	switch (msg)
//	{
//	case WM_CREATE:
//	{
//		recordButton = CreateWindow("button", "Record",
//			WS_VISIBLE | WS_CHILD | WS_BORDER,
//			30, 160, 120, 40,
//			hwnd, (HMENU)1, NULL, NULL);
//
//		stopRecordButton = CreateWindow("button", "Stop Record",
//			WS_VISIBLE | WS_CHILD | WS_BORDER,
//			180, 160, 120, 40,
//			hwnd, (HMENU)2, NULL, NULL);
//		EnableWindow(stopRecordButton, false);
//		build3dmodelbutton = CreateWindow("button", "build the sciene",
//			WS_VISIBLE | WS_CHILD | WS_BORDER,
//			330, 160, 120, 40,
//			hwnd, (HMENU)3, NULL, NULL);
//		EnableWindow(build3dmodelbutton, false);
//		break;
//	}
//	case WM_DESTROY:
//	{
//		PostQuitMessage(0);
//		return 0;
//	}
//	case WM_COMMAND:
//	{
//		switch (LOWORD(wParam))
//		{
//		case 1:
//		{
//
//			FileProcessing fp;
//			vector<PointCloudT>::size_type j;
//			vector<int> mapping;
//			cout << "*** Video capturing started ***" << endl;
//
//			SimpleOpenNIViewer::instance()->record_flag = 1;
//			SimpleOpenNIViewer::instance()->run();
//			cout << "*** Video capturing stopped ***" << endl;
//			cout << "*** Filtering started ***" << endl;
//			for (j = 0; j != SimpleOpenNIViewer::instance()->point_cloud_list.set.size(); j++)
//				removeNaNFromPointCloud(SimpleOpenNIViewer::instance()->point_cloud_list.set[j], SimpleOpenNIViewer::instance()->point_cloud_list.set[j], mapping);
//			cout << "*** Filtering stopped ***" << endl;
//			EnableWindow(stopRecordButton, true);
//			EnableWindow(recordButton, false);
//			EnableWindow(build3dmodelbutton, false);
//			break;
//		}
//		case 2:
//		{
//			SimpleOpenNIViewer::instance()->record_flag = 0;
//			SimpleOpenNIViewer::instance()->camera->stop();
//			EnableWindow(build3dmodelbutton, true);
//			EnableWindow(recordButton, true);
//			EnableWindow(stopRecordButton, false);
//			break;
//		}
//		case 3:
//		{
//
//			ICPAlgorithm icp_Alg;
//			icp_Alg.register_with_result(SimpleOpenNIViewer::instance()->point_cloud_list.set, 20, 0.05);
//			EnableWindow(build3dmodelbutton, false);
//			EnableWindow(recordButton, false);
//			EnableWindow(stopRecordButton, false);
//			break;
//		}
//		default:
//			break;
//		}
//	}
//
//	}
//	return DefWindowProc(hwnd, msg, wParam, lParam);
//}
//
//int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
//{
//	GUI main;
//
//	return main.simon( hInstance,  hPrevInstance,  lpCmdLine,  nCmdShow);
//}