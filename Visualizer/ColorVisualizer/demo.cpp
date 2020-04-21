#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>

using namespace std;
using namespace pcl;
using namespace io;

int main() {
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

    if (io::loadPLYFile("/home/bian/software/test.ply", *cloud) == -1) { // 读取.ply文件
        cerr << "can't read file file.ply " << endl;
        return -1;
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z"); // 按照z字段进行渲染
    
    viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); // 设置点云大小

    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}
