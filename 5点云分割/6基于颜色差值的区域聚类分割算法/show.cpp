#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>                          /* 搜索　kd树 */
#include <pcl/visualization/cloud_viewer.h>             /* 可视化 */
#include <pcl/filters/passthrough.h>                    /* 直通滤波器 */
#include <pcl/segmentation/region_growing_rgb.h>        /* 基于颜色的区域增长点云分割算法 */

int main( int argc, char** argv ){
	/* 搜索算法 */
	pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > ( new pcl::search::KdTree<pcl::PointXYZRGB>);
	/* 点云的类型 */
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud <pcl::PointXYZRGB>);
	if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ( "../../datas/region_growing_rgb_tutorial.pcd", *cloud ) == -1 )
	{
		std::cout << "Cloud reading failed." << std::endl;
		return(-1);
	}
	/* 存储点云索引　的容器 */
	pcl::IndicesPtr indices( new std::vector <int>);
	/* 直通滤波在Z轴的0到1米之间 剔除　nan　和　噪点 */
	pcl::PassThrough<pcl::PointXYZRGB> pass;        /*  */
	pass.setInputCloud( cloud );
	pass.setFilterFieldName( "z" );
	pass.setFilterLimits( 0.0, 1.0 );
	pass.filter( *indices );                        /* 直通滤波后的的点云的索引　避免拷贝 */

	/* 基于颜色的区域生成的对象 */
	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud( cloud );
	reg.setIndices( indices );                      /* 点云的索引 */
	reg.setSearchMethod( tree );
	reg.setDistanceThreshold( 10 );                 /* 距离的阀值 */
	reg.setPointColorThreshold( 6 );                /* 点与点之间颜色容差 */
	reg.setRegionColorThreshold( 5 );               /* 区域之间容差 */
	reg.setMinClusterSize( 600 );                   /* 设置聚类的大小 */
	std::vector <pcl::PointIndices> clusters;
	reg.extract( clusters );                        /*  */

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::CloudViewer		viewer( "Cluster viewer" );
	viewer.showCloud( colored_cloud );
	while ( !viewer.wasStopped() )
	{
		boost::this_thread::sleep( boost::posix_time::microseconds( 100 ) );
	}

	return(0);
}
