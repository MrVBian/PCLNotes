[GASD描述子介绍](https://blog.csdn.net/xiuzhang5738/article/details/84879093)

# 全局一致的空间分布描述子特征

pcl版本 >= 1.9才有

Globally Aligned Spatial Distribution (GASD) descriptors

可用于物体识别和姿态估计。

是对可以描述整个点云的参考帧的估计，这是用来对准它的正则坐标系统之后，根据其三维点在空间上的分布，计算出点云的描述符。描述符也可以扩展到整个点云的颜色分布。匹配点云(icp)的全局对齐变换用于计算物体姿态。

```c++
#include <pcl/features/gasd.h>
--------------------------------------------------
// 创建 GASD 全局一致的空间分布描述子特征 传递 点云
 // pcl::GASDColorEstimation<pcl::PointXYZRGBA, pcl::GASDSignature984> gasd;//包含颜色
  pcl::GASDColorEstimation<pcl::PointXYZ, pcl::GASDSignature984> gasd;
  gasd.setInputCloud (cloud_ptr);
  // 输出描述子
  pcl::PointCloud<pcl::GASDSignature984> descriptor;
  // 计算描述子
  gasd.compute (descriptor);
  // 得到匹配 变换
  Eigen::Matrix4f trans = gasd.getTransform();
```