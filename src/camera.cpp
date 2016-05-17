#include "camera.h"

Camera::Camera()
{
	this->camera = new OpenNI2Grabber();
}

Camera::~Camera()
{
	this->camera->stop();
	delete camera;
}

void Camera::registerCallback(boost::function<void(PointCloudConstPtr&)> f)
{
	this->camera->registerCallback(f);
}

void Camera::start()
{
	this->camera->start();
}

void Camera::stop()
{
	this->camera->stop();
}