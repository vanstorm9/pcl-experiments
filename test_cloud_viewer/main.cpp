#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
typedef pcl::PointCloud<pcl::PointXYZRGBA> cloud_type;

void cloud_callback(cloud_type::ConstPtr cloud, pcl::visualization::CloudViewer* cloud_viewer)
{
	if(!cloud_viewer->wasStopped()){
		cloud_viewer->showCloud(cloud);
	}

}

int main (int argc, const char** argv)
{
	pcl::visualization::CloudViewer cloud_viewer("World Viewer");
	pcl::OpenNIGrabber grabber;
	boost::function<void (const cloud_type::ConstPtr&)> f = boost::bind(cloud_callback, _1, &cloud_viewer);
	grabber.registerCallback (f);
	grabber.start();
	while (!cloud_viewer.wasStopped()) {
		sleep(1);
	}
	grabber.stop();
	return 0;
}




