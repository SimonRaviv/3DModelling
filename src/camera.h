#include <pcl/io/openni2_grabber.h>
#include "3DModellingH.h"

class Camera
{
public:
	Grabber* camera;

	Camera();
	~Camera();
	void registerCallback(boost::function<void(PointCloudConstPtr&)> f);
	void start();
	void stop();

private:

};





