// 该方法(PointCloudColorHandlerCustom)适用于任何格式点云，不要求点云类型包含RGB三个颜色分量，即将id为"sample cloud"的点云作为一个整体进行着色。

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>

using namespace std;
using namespace pcl;
using namespace io;

int main() {
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

	if (io::loadPLYFile("/home/bian/software/test.ply", *cloud) == -1) { 
		cerr << "can't read file ping2.pcd" << endl;
		return -1;
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    // 这里我们需要创建一个自定义颜色处理器PointCloudColorHandlerCustom对象，并设置颜色为纯绿色。
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0); 

    // 当我们添加点云的时候，我们可以指定添加到视窗中点云的PointCloudColorHandlerRGB着色处理对象。
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");  
	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "sample cloud");// 也可以
	while (!viewer->wasStopped()){
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}
