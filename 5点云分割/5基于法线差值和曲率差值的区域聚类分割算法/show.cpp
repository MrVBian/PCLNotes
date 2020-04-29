#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>                      /* 文件io */
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>                  /* 搜索　kd树 */
#include <pcl/features/normal_3d.h>             /* 计算点云法线曲率特征 */
#include <pcl/visualization/cloud_viewer.h>     /* 可视化 */
#include <pcl/filters/passthrough.h>            /* 直通滤波器 */
#include <pcl/segmentation/region_growing.h>    /* 区域增长点云分割算法 */

int main( int argc, char** argv ){
	/* 点云的类型 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>);
	/* 打开点云pdc文件　载入点云 */
	if ( pcl::io::loadPCDFile <pcl::PointXYZ> ( "../../datas/region_growing_tutorial.pcd", *cloud ) == -1 ){
		std::cout << "Cloud reading failed." << std::endl;
		return(-1);
	}
	/* 设置搜索的方式或者说是结构　kd树　搜索 */
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > ( new pcl::search::KdTree<pcl::PointXYZ>);
	/* 求法线　和　曲率　 */
	pcl::PointCloud <pcl::Normal>::Ptr			normals( new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>	normal_estimator;
	normal_estimator.setSearchMethod( tree );
	normal_estimator.setInputCloud( cloud );
	normal_estimator.setKSearch( 50 ); /* 临近50个点 */
	normal_estimator.compute( *normals );

    /************************************************************************************
   创建直通滤波器的对象，设立参数，滤波字段名被设置为Z轴方向，可接受的范围为（0.0，1.0）
   即将点云中所有点的Z轴坐标不在该范围内的点过滤掉或保留，这里是过滤掉，由函数setFilterLimitsNegative设定
   ***********************************************************************************/
	/* 直通滤波在Z轴的0到1米之间 剔除　nan　和　噪点 */
	pcl::IndicesPtr			indices( new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud( cloud );                                //设置输入点云
	pass.setFilterFieldName( "z" );                             //设置过滤时所需要点云类型的Z字段
	pass.setFilterLimits( 0.0, 1.0 );                           //设置在过滤字段的范围
    //pass.setFilterLimitsNegative (true);                      //设置保留范围内还是过滤掉范围内
	pass.filter( *indices );                                    //执行滤波，保存过滤结果在indices
	/* 区域增长聚类分割对象　<点，法线> */
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize( 50 );                                                            /* 最小的聚类的点数 */
	reg.setMaxClusterSize( 1000000 );                                                       /* 最大的聚类的点数 */
	reg.setSearchMethod( tree );                                                            /* 搜索方式 */
	reg.setNumberOfNeighbours( 30 );                                                        /* 设置搜索的邻域点的个数 */
	reg.setInputCloud( cloud );                                                             /* 输入点 */
	/* reg.setIndices (indices); */
	reg.setInputNormals( normals );                                                         /* 输入的法线 */
	reg.setSmoothnessThreshold( 3.0 / 180.0 * M_PI );                                       /* 设置平滑度 法线差值阈值 */
	reg.setCurvatureThreshold( 1.0 );                                                       /* 设置曲率的阀值 */

	std::vector <pcl::PointIndices> clusters;
	reg.extract( clusters );                                                                /* 提取点的索引 */

	std::cout << "点云团数量　Number of clusters is equal to " << clusters.size() << std::endl;   /* 点云团　个数 */
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
	std::cout << "These are the indices of the points of the initial" <<
	std::endl << "cloud that belong to the first cluster:" << std::endl;


    /*
     * int counter = 0;
     * while (counter < clusters[0].indices.size ())
     * {
     *  std::cout << clusters[0].indices[counter] << ", ";//索引
     *  counter++;
     *  if (counter % 10 == 0)
     *    std::cout << std::endl;
     * }
     * std::cout << std::endl;
     */
	/* 可视化聚类的结果 */
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::CloudViewer		viewer( "Cluster viewer" );
	viewer.showCloud( colored_cloud );
	while ( !viewer.wasStopped() ){
	}

	return(0);
}
