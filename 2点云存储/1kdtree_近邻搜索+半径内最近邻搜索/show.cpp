#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>                        /* time */

int main( int argc, char** argv ) {
	srand( time( NULL ) );          /* 随机数 */

	time_t begin, end;
	begin = clock();                /* 开始计时 */
	/* 点云对象指针 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr	cloud_ptr( new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> &	cloud = *cloud_ptr;

	/* 产生假的点云数据 */
	cloud.width	= 400000;       /* 40万数据点 */
	cloud.height	= 1;
	cloud.points.resize( cloud.width * cloud.height );

	for ( size_t i = 0; i < cloud.points.size(); ++i ){
		cloud.points[i].x	= 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].y	= 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].z	= 1024.0f * rand() / (RAND_MAX + 1.0f);
	}
	/* kdtree对象 */
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	/* 输入点云 */
	kdtree.setInputCloud( cloud_ptr );
	/* 随机定义一个 需要搜寻的点 */
	pcl::PointXYZ searchPoint;
	searchPoint.x	= 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y	= 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z	= 1024.0f * rand() / (RAND_MAX + 1.0f);

	/* K 个最近点去搜索 nearest neighbor search */
	int K = 10;

	std::vector<int>	pointIdxNKNSearch( K );         /* 最近临搜索得到的索引 */
	std::vector<float>	pointNKNSquaredDistance( K );   /* 平方距离 */

	std::cout	<< "K nearest neighbor search at (" << searchPoint.x
			<< " " << searchPoint.y
			<< " " << searchPoint.z
			<< ") with K=" << K << std::endl;
	/* 开始搜索 */


	/***********************************************************************************************
	 * kdtree 近邻搜索
	 * template<typename PointT>
	 * virtual int pcl::KdTree< PointT >::nearestKSearch  ( const PointT &  p_q,
	 *                                                     int  k,
	 *                                                     std::vector< int > &  k_indices,
	 *                                                     std::vector< float > &  k_sqr_distances
	 *                                                     )  const [pure virtual]
	 * Search for k-nearest neighbors for the given query point.
	 * Parameters:
	 *     [in] the given query point
	 *     [in] k the number of neighbors to search for
	 *     [out] the resultant indices of the neighboring points
	 *     [out] the resultant squared distances to the neighboring points
	 * Returns:
	 *     number of neighbors found
	 ********************************************************************************************/
	if ( kdtree.nearestKSearch( searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance ) > 0 ){
		for ( size_t i = 0; i < pointIdxNKNSearch.size(); ++i )
			std::cout	<< " " << cloud.points[pointIdxNKNSearch[i]].x
					<< " " << cloud.points[pointIdxNKNSearch[i]].y
					<< " " << cloud.points[pointIdxNKNSearch[i]].z
					<< " (squared distance: "
					<< pointNKNSquaredDistance[i]
					<< ")"
					<< std::endl;
	}

	/* 半径内最近领搜索 Neighbors within radius search */
	std::vector<int>	pointIdxRadiusSearch;                           /* 半价搜索得到的索引 */
	std::vector<float>	pointRadiusSquaredDistance;                     /* 平方距离 */
	float			radius = 256.0f * rand() / (RAND_MAX + 1.0f);   /* 随机产生一个半价 */
	std::cout	<< "Neighbors within radius search at (" << searchPoint.x
			<< " " << searchPoint.y
			<< " " << searchPoint.z
			<< ") with radius="
			<< radius << std::endl;
	/* 开始搜索 */
	if ( kdtree.radiusSearch( searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance ) > 0 ){
		for ( size_t i = 0; i < pointIdxRadiusSearch.size(); ++i )
			std::cout	<< " " << cloud.points[pointIdxRadiusSearch[i]].x
					<< " " << cloud.points[pointIdxRadiusSearch[i]].y
					<< " " << cloud.points[pointIdxRadiusSearch[i]].z
					<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}
	/* -------------------------------------------------------------------------------------------- */
	end = clock();                                          /* 结束计时 */
	double Times = double(end - begin) / CLOCKS_PER_SEC;    /* 将clock()函数的结果转化为以秒为单位的量 */

	std::cout << "time: " << Times << "s" << std::endl;

	return(0);
}
