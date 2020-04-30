# 特征提取

如果要对一个三维点云进行描述，光有点云的位置是不够的，常常需要计算一些额外的参数，比如法线方向、曲率、纹理特征、颜色、领域中心距、协方差矩阵、熵等等。如同图像的特征（sifi surf orb）一样，我们需要使用类似的方式来描述三维点云的特征。

常用的特征描述算法有：
1. 法线和曲率计算 normal_3d_feature 
2. 特征值分析
3. PFH  点特征直方图描述子 nk2
4. FPFH 快速点特征直方图描述子 FPFH是PFH的简化形式 nk
5. 3D Shape Context、 文理特征
6. Spin Image
7. VFH视点特征直方图(Viewpoint Feature Histogram)
8. NARF关键点  pcl::NarfKeypoint narf特征 pcl::NarfDescriptor
9. RoPs特征(Rotational Projection Statistics)

(GASD）全局一致的空间分布描述子特征 Globally Aligned Spatial Distribution (GASD) descriptors