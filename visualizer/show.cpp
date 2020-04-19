#include <unistd.h>
#include <iostream>
#include <string>
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


    pcl::visualization::CloudViewer viewer("cloud_viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()){
        ;
    }


    return(0);
}
