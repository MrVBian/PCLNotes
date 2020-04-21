#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcl_io.h>
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>

int main(int argc, char** argv){
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    if(pcl::io::loadPLYFile("", *cloud) == -1){
        PCL_ERROR();
        return -1;
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("111"));

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("111"));

    pcl::visualization::PointCloudColorHandleRGBField<pcl::PointXYZRGB> rgb(cloud);

    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "title");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "title");

    while(!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microsedonds(100000));
    }

}

