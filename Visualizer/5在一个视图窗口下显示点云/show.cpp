#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;

int main(){
    //***************************read file*****************************************
	//在一个视图下同时显示两张点云图像
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source2(new pcl::PointCloud<pcl::PointXYZRGB>());
	//输入点云路径
	string filename1 = "../../datas/SphereDivision.ply";
	string filename2 = "../../datas/SphereDivision.ply";
	pcl::io::loadPLYFile(filename1, *source);
	pcl::io::loadPLYFile(filename2, *source2);
	cout << "点云加载成功！" << endl;
	boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	
	//在一个视图里显示两张点云图
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
    //第一个参数:设置文本;第二、三个参数:设置位置;
	viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(source);
	viewer->addPointCloud<pcl::PointXYZRGB>(source, rgb, "sample cloud1", v1);
    //上面几行代码，创建新的视口，所需的四个参数是视口在X轴的最小值、最大值、Y轴的最小值、最大值，取值在0-1之间。我们创建的视口分布于窗口的左半部分，最后一个字符串参数，用来唯一标识该视口，在其他改变该视口内容的调用中，需要以该唯一标识为参数，我们还设置该视口的背景颜色，添加一个标签以区别于其他视口，利用RGB颜色着色器并添加点云到当前视口中。
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(source2, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(source2, single_color, "sample cloud2", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");

    //查看复杂的点云经常会让用户感到没方向感，为了让用户保持正确的坐标判断，需要显示坐标系统方向，可以通过使用X (红色)、Y (绿色) 、Z (蓝色)圆柱体代表坐标轴的显示方式来解决，圆柱体的大小通过scale参数控制。本例中，我们将scale参数设置为1.0，该值也为缺省值，该方法的另一重载函数可实现对点云中的每个点的坐标方向进行显示。
    //参数为坐标系相对大小
	viewer->addCoordinateSystem(1.0); 
	viewer->spin();
	return 0;
}
