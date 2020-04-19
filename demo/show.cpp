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

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2 ){
	/*
	 * --------------------------------------------------------
	 * -----Open 3D viewer and add point cloud and normals-----
	 * --------------------------------------------------------
	 */
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "3D Viewer" ) );
	viewer->initCameraParameters();
	/* 以上是创建视图的标准代码 */

	int v1( 0 );                                                    /* 创建新的视口 */
	viewer->createViewPort( 0.0, 0.0, 0.5, 1.0, v1 );               /* 4个参数分别是X轴的最小值，最大值，Y轴的最小值，最大值，取值0-1，v1是标识 */
	viewer->setBackgroundColor( 0, 0, 0, v1 );                      /* 设置视口的背景颜色 */
	viewer->addText( "Radius: 0.01", 10, 10, "v1 text", v1 );       /* 添加一个标签区别其他窗口  利用RGB颜色着色器并添加点云到视口中 */
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb( cloud );
	viewer->addPointCloud<pcl::PointXYZRGB> ( cloud, rgb, "sample cloud1", v1 );
	/* 对第二视口做同样的操作，使得做创建的点云分布于右半窗口，将该视口背景赋值于灰色，以便明显区别，虽然添加同样的点云，给点云自定义颜色着色 */
	int v2( 0 );
	viewer->createViewPort( 0.5, 0.0, 1.0, 1.0, v2 );
	viewer->setBackgroundColor( 0.3, 0.3, 0.3, v2 );
	viewer->addText( "Radius: 0.1", 10, 10, "v2 text", v2 );
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color( cloud, 0, 255, 0 );
	viewer->addPointCloud<pcl::PointXYZRGB> ( cloud, single_color, "sample cloud2", v2 );
	/* 为所有视口设置属性， */
	viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1" );
	viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2" );
	viewer->addCoordinateSystem( 1.0 );
	/* 添加法线  每个视图都有一组对应的法线 */
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> ( cloud, normals1, 10, 0.05, "normals1", v1 );
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> ( cloud, normals2, 10, 0.05, "normals2", v2 );

	return(viewer);
}


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

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = viewportsVis( cloud, cloud, cloud_normals2 );



    return(0);
}
