#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Filter object.
	pcl::PassThrough<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	// Filter out all points with Z values not in the [0-2] range.
	filter.setFilterFieldName("z");
	filter.setFilterLimits(0.0, 2.0);

	filter.filter(*filteredCloud);
}
