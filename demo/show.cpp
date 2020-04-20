#include <unistd.h>
#include <iostream>
#include <string>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>
 
using namespace pcl;
using namespace pcl::io;
using namespace std;
int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>("/home/bian/software/test.ply", *cloud) == -1) {       
        PCL_ERROR("Couldnot read file.\n");
        printf("error");
        pause();
        return(-1);
    }

    printf("success");

    // pcl::visualization::CloudViewer viewer("Cloud Viewer");
    // viewer.showCloud(cloud);
    // pause();


    // pcl::visualization::CloudViewer viewer("cloud_viewer");
    // viewer.showCloud(cloud);
    // while (!viewer.wasStopped()){
    //     ;
    // }


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();

    // int v1(0);
    // viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    // viewer->setBackgroundColor (0, 0, 0, v1);
    // viewer->addText ("Radius: 0.01", 10, 10, "v1 text", v1);
    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (cloud);
    // viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

    // int v2(0);
    // viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    // viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
    // viewer->addText ("Radius: 0.1", 10, 10, "v2 text", v2);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color (cloud, 0, 255, 0);
    // viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud2", v2);

    int v1,v2,v3,v4;
    viewer->createViewPort (0.0, 0.0, 0.5, 0.5, v1); //(Xmin,Ymin,Xmax,Ymax)设置窗口坐标
    viewer->createViewPort (0.5, 0.0, 1.0, 0.5, v2);
    viewer->createViewPort (0.0, 0.5, 0.5, 1.0, v3);
    viewer->createViewPort (0.5, 0.5, 1.0, 1.0, v4);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);


    return(0);
}
