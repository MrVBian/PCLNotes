/*
 * 功能：利用octree进行点云压缩和解压
 */

#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include <iostream>

int main(){
	/* 加载点云 */
	pcl::PointCloud<pcl::PointXYZRGB> sourceCloud;
	if ( pcl::io::loadPCDFile( "../../datas/milk.pcd", sourceCloud ) == -1 )
		return(-1);

	/* 是否查看压缩信息 */
	bool showStatistics = true;
	/* 配置文件，如果想看配置文件的详细内容，可以参考: /io/include/pcl/compression/compression_profiles.h */
	pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

	/* 初始化点云压缩器和解压器 */
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder;
	PointCloudEncoder	= new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>( compressionProfile, showStatistics );
	PointCloudEncoder	= new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>( compressionProfile, true, 0.002 );


	/* 压缩结果stringstream */
	std::stringstream compressedData;
	/* 输出点云 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut( new pcl::PointCloud<pcl::PointXYZRGB>() );
	/* 压缩点云 */
	PointCloudEncoder->encodePointCloud( sourceCloud.makeShared(), compressedData );
	std::cout << compressedData.str() << std::endl;
	/* 解压点云 */
	PointCloudEncoder->decodePointCloud( compressedData, cloudOut );
	pcl::visualization::CloudViewer viewer( "Simple Cloud Viewer" );
	viewer.showCloud( cloudOut );
	while ( !viewer.wasStopped() ){
	}

	std::system( "pause" );
	return(0);
}

// 压缩选项详见 /io/include/pcl/compression/compression_profiles.h
// LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR:分辨率1cm3，无颜色，快速在线编码
// LOW_RES_ONLINE_COMPRESSION_WITH_COLOR:分辨率1cm3，有颜色，快速在线编码
// MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR:分辨率5mm3，无颜色，快速在线编码
// MED_RES_ONLINE_COMPRESSION_WITH_COLOR:分辨率5mm3，有颜色，快速在线编码
// HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR:分辨率1mm3，无颜色，快速在线编码
// HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR:分辨率1mm3，有颜色，快速在线编码
// LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR:分辨率1cm3，无颜色，高效离线编码
// LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR:分辨率1cm3，有颜色，高效离线编码
// MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR:分辨率5mm3，无颜色，高效离线编码
// MED_RES_OFFLINE_COMPRESSION_WITH_COLOR:分辨率5mm3，有颜色，高效离线编码
// HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR:分辨率5mm3，无颜色，高效离线编码
// HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR:分辨率5mm3，有颜色，高效离线编码
// MANUAL_CONFIGURATION允许为高级参数化进行手工配置

