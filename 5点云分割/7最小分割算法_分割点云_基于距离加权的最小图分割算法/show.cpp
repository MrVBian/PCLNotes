#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <iostream>
#include <pcl/segmentation/region_growing_rgb.h>

int main( int argc, char** argv ){
	/* 申明点云的类型 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>);
	/* 法线 */
	pcl::PointCloud<pcl::Normal>::Ptr normals( new pcl::PointCloud<pcl::Normal>);

	if ( pcl::io::loadPCDFile<pcl::PointXYZ>( "../../datas/min_cut_segmentation_tutorial.pcd", *cloud ) != 0 ){
		return(-1);
	}
	/* 申明一个Min-cut的聚类对象 */
	pcl::MinCutSegmentation<pcl::PointXYZ> clustering;
	clustering.setInputCloud( cloud ); /* 设置输入 */
	/*
	 * 创建一个点云，列出所知道的所有属于对象的点
	 * （前景点）在这里设置聚类对象的中心点（想想是不是可以可以使用鼠标直接选择聚类中心点的方法呢？）
	 */
    // 指定打击目标（目标点）
	pcl::PointCloud<pcl::PointXYZ>::Ptr	foregroundPoints( new pcl::PointCloud<pcl::PointXYZ>() );
	pcl::PointXYZ				point;
	point.x = 68.97;
	point.y = -18.55;
	point.z = 0.57;
	foregroundPoints->points.push_back( point );
	clustering.setForegroundPoints( foregroundPoints );     /* 设置聚类对象的前景点 */

	/* 设置sigma，它影响计算平滑度的成本。它的设置取决于点云之间的间隔（分辨率） */
	clustering.setSigma( 0.25 );                            /* cet cost = exp(-(dist/cet)^2) */
	/* 设置聚类对象的半径. 物体大概范围*/
	clustering.setRadius( 3.0433856 );                           /* dist2Center / radius */

	/* 设置需要搜索的临近点的个数，增加这个也就是要增加边界处图的个数.用多少生成图 */
	clustering.setNumberOfNeighbours( 14 );

	/* 设置前景点的权重（也就是排除在聚类对象中的点，它是点云之间线的权重，） */
	clustering.setSourceWeight( 0.8 );

    // 分割结果
	std::vector <pcl::PointIndices> clusters;
	clustering.extract( clusters );

	std::cout << "Maximum flow is " << clustering.getMaxFlow() << "." << std::endl;

	int currentClusterNum = 1;
	for ( std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i ){
		/* 设置聚类后点云的属性 */
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster( new pcl::PointCloud<pcl::PointXYZ>);
		for ( std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++ )
			cluster->points.push_back( cloud->points[*point] );
		cluster->width		= cluster->points.size();
		cluster->height		= 1;
		cluster->is_dense	= true;

		/* 保存聚类的结果 */
		if ( cluster->points.size() <= 0 )
			break;
		std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
		std::string fileName = "cluster" + boost::to_string( currentClusterNum ) + ".pcd";
		pcl::io::savePCDFileASCII( fileName, *cluster );

		currentClusterNum++;
	}
}
