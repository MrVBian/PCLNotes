#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
 
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer){
    viewer.setBackgroundColor(0,0,1);
}
 
int main (int argc, char** argv){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader reader;
    // 读入点云PCD文件
    reader.read("/home/bian/software/test.ply",*cloud);
 
    std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional 优化系数
    seg.setOptimizeCoefficients (true);
    // Mandatory 强制性
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    // 距离阈值 单位m
    seg.setDistanceThreshold (0.15);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0){
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }
 
 
    // 提取地面
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.filter (*cloud_filtered);
 
    std::cerr << "Ground cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
 
    pcl::PLYWriter writer;
    writer.write<pcl::PointXYZ> ("3dpoints_ground.pcd", *cloud_filtered, false);
 
    // 提取除地面外的物体
    extract.setNegative (true);
    extract.filter (*cloud_filtered);
 
    std::cerr << "Object cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
 
    writer.write<pcl::PointXYZ> ("3dpoints_object.pcd", *cloud_filtered, false);
 
    // 点云可视化
    pcl::visualization::CloudViewer viewer("Filtered");
    viewer.showCloud(cloud_filtered);
    // 设置背景颜色
    // viewer.runOnVisualizationThreadOnce(viewerOneOff);
    while(!viewer.wasStopped()){
        ;
    }
    return (0);
}
