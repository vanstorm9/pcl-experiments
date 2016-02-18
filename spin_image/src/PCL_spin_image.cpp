#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>

// A handy typedef.
typedef pcl::Histogram<153> SpinImage;

int
main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the spin image for each point.
	pcl::PointCloud<SpinImage>::Ptr descriptors(new pcl::PointCloud<SpinImage>());

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Note: you would usually perform downsampling now. It has been omitted here
	// for simplicity, but be aware that computation can take a long time.

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	// Spin image estimation object.
	pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> si;
	si.setInputCloud(cloud);
	si.setInputNormals(normals);
	// Radius of the support cylinder.
	si.setRadiusSearch(0.02);
	// Set the resolution of the spin image (the number of bins along one dimension).
	// Note: you must change the output histogram size to reflect this.
	si.setImageWidth(8);

	si.compute(*descriptors);
}
