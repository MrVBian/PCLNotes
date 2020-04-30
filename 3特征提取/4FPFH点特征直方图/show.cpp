#include <pcl/point_types.h>
/* #include <pcl/features/pfh.h> */
#include <pcl/features/fpfh.h>
#include <pcl/io/pcd_io.h>                              /* 点云文件pcd 读写 */
#include <pcl/features/normal_3d.h>                     /* 法线特征 */
#include <pcl/visualization/histogram_visualizer.h>     /* 直方图的可视化 */
#include <pcl/visualization/pcl_plotter.h>              /* 直方图的可视化 方法2 */

/* using namespace std; */
using std::cout;
using std::endl;
int main( int argc, char** argv ){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr( new pcl::PointCloud<pcl::PointXYZ>);
    /* ======【1】 读取点云文件　填充点云对象====== */
	pcl::PCDReader reader;
	reader.read( "../../datas/Sphere.pcd", *cloud_ptr );
	if ( cloud_ptr == NULL ){
		cout << "pcd file read err" << endl; return(-1);
	}
	cout	<< "PointCLoud size() " << cloud_ptr->width * cloud_ptr->height
		<< " data points ( " << pcl::getFieldsList( *cloud_ptr ) << "." << endl;

    /* =====【2】计算法线========创建法线估计类==================================== */
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud( cloud_ptr );

    /*
     * 法线估计类NormalEstimation的实际计算调用程序内部执行以下操作：
     * 对点云P中的每个点p
     * 1.得到p点的最近邻元素
     * 2.计算p点的表面法线n
     * 3.检查n的方向是否一致指向视点，如果不是则翻转
     * 在PCL内估计一点集对应的协方差矩阵，可以使用以下函数调用实现：
     * //定义每个表面小块的3x3协方差矩阵的存储对象
     * Eigen::Matrix3fcovariance_matrix;
     * //定义一个表面小块的质心坐标16-字节对齐存储对象
     * Eigen::Vector4fxyz_centroid;
     * //估计质心坐标
     * compute3DCentroid(cloud,xyz_centroid);
     * //计算3x3协方差矩阵
     * computeCovarianceMatrix(cloud,xyz_centroid,covariance_matrix);
     */
    /* 添加搜索算法 kdtree search  最近的几个点 估计平面 协方差矩阵PCA分解 求解法线 */
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZ> () );
	ne.setSearchMethod( tree );     /* 设置近邻搜索算法 */
	/* 输出点云 带有法线描述 */
	pcl::PointCloud<pcl::Normal>::Ptr	cloud_normals_ptr( new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal> &		cloud_normals = *cloud_normals_ptr;
	/* Use all neighbors in a sphere of radius 3cm */
	ne.setRadiusSearch( 0.03 );     /* 半径内搜索临近点 3cm */
	/* 计算表面法线特征 */
	ne.compute( cloud_normals );

    /*
     * =======【3】创建FPFH估计对象fpfh, 并将输入点云数据集cloud和法线normals传递给它=================
     * pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;// phf特征估计器
     */
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    /* pcl::FPFHEstimationOMP<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh;//多核加速 */
	fpfh.setInputCloud( cloud_ptr );
	fpfh.setInputNormals( cloud_normals_ptr );
	/*
	 * 如果点云是类型为PointNormal,则执行pfh.setInputNormals (cloud);
	 * 创建一个空的kd树表示法，并把它传递给PFH估计对象。
	 * 基于已给的输入数据集，建立kdtree
	 */
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2( new pcl::search::KdTree<pcl::PointXYZ> () );
	/* pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree2 (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); //-- older call for PCL 1.5- */
	fpfh.setSearchMethod( tree2 );                                                                          /* 设置近邻搜索算法 */
	/*
	 * 输出数据集
	 * pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_fe_ptr (new pcl::PointCloud<pcl::PFHSignature125> ());//phf特征
	 */
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_fe_ptr( new pcl::PointCloud<pcl::FPFHSignature33>() );  /* fphf特征 */
	/*
	 * 使用半径在5厘米范围内的所有邻元素。
	 * 注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
	 */
	fpfh.setRadiusSearch( 0.05 );
	/* 计算pfh特征值 */
	fpfh.compute( *fpfh_fe_ptr );

	cout << "phf feature size : " << fpfh_fe_ptr->points.size() << endl;
	/* 应该与input cloud->points.size ()有相同的大小，即每个点都有一个pfh特征向量 */

    /* ========直方图可视化============================= */
	pcl::visualization::PCLHistogramVisualizer view;                                /* 直方图可视化 */
	view.setBackgroundColor( 255, 0, 0 );                                           /* 背景红色 */
	view.addFeatureHistogram<pcl::FPFHSignature33> ( *fpfh_fe_ptr, "fpfh", 100 );   /* 对下标为100的点的直方图特征可视化 */
	view.spinOnce();                                                                /* 循环的次数 */
	
	view.spin();  //无限循环
	pcl::visualization::PCLPlotter plotter;
	plotter.addFeatureHistogram(*fpfh_fe_ptr, 300); //设置的很坐标长度，该值越大，则显示的越细致
	plotter.plot();

	return(0);
}
