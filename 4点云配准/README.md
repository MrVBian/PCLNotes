# 点云配准 Registration

在逆向工程，计算机视觉，文物数字化等领域中，**由于点云的不完整，旋转错位，平移错位等，使得要得到的完整的点云就需要对局部点云进行配准**，为了得到被测物体的完整数据模型，**需要确定一个合适的坐标系，将从各个视角得到的点集合并到统一的坐标系下形成一个完整的点云**，然后就可以方便进行可视化的操作，这就是点云数据的配准。

实质就是**把不同的坐标系中测得到的数据点云进行坐标系的变换**，以得到整体的数据模型，问题的关键是如何让得到坐标变换的参数R（旋转矩阵）和T（平移向量），使得两视角下测得的三维数据经坐标变换后的距离最小，目前**配准算法按照过程可以分为整体配准和局部配准**。PCL中有单独的配准模块，实现了配准相关的基础数据结构，和经典的配准算法如ICP。 

给定两个来自不同坐标系的三维数据点集，找到两个点集空间的变换关系，使得两个点集能统一到同一坐标系统中，即配准过程

求得旋转和平移矩阵
P2 = R*P1  + T　　　　[R t]

点云配准的概念也可以类比于二维图像中的配准，

只不过二维图像配准获取得到的是$$x，y，alpha，beta$$等放射变化参数

三维点云配准可以模拟三维点云的移动和对其，也就是会获得一个旋转矩阵和一个平移向量，通常表达为一个$$4×3$$的矩阵，其中$$3×3$$是旋转矩阵，$$1×3$$是平移向量。**严格说来是6个参数**，因为旋转矩阵也可以通过**罗格里德斯**变换转变成1*3的旋转向量。



## 常用的点云配准算法有两种：

1. 正态分布变换方法  NDT  正态分布变换进行配准（normal Distributions Transform） 
2. 著名的迭代最近点 Iterative Closest Point （ICP） 点云配准

**NDT 耗时稳定，跟初值相关不大，初值误差大时，也能很好的纠正过来**；  
**ICP耗时多，容易陷入局部最优**；  

资料：https://zhuanlan.zhihu.com/p/96908474

# 其它配准算法

1. ICP：稳健ICP、point to plane ICP、point to line ICP、MBICP、GICP
2. NDT: NDT 3D、Multil-Layer NDT
3. FPCS、KFPSC、SAC-IA
4. Line Segment Matching、ICL

# 算法对比

**1、 ICP**

A．要剔除不合适的点对（点对距离过大、包含边界点的点对）
B．基于点对的配准，并没有包含局部形状的信息
C．每次迭代都要搜索最近点，计算代价高昂

​	存在多种优化了的变体算法，如八叉树等

**2、 IDC**

​	ICP的一种改进，采用极坐标代替笛卡尔坐标进行最近点搜索匹配

**3、 PIC**

​	考虑了点云的噪音和初始位置的不确定性

**4、 Point-based probabilistic registration**

​	需要首先建立深度图的三角面片

**5、 NDT——正态分布变换**

​	计算正态分布是一个一次性的工作（初始化），不需要消耗大量代价计算最近邻搜索匹配点

​	概率密度函数在两幅图像采集之间的时间可以离线计算出来 

**6、 Gaussian fields**

​	和NDT正态分布变换类似，利用高斯混合模型考察点和点的距离和点周围表面的相似性

**7、 Quadratic patches**

**8、 Likelihood-field matching——随机场匹配**

**9、 CRF匹配**

​	缺点： 运行速度慢，在3d中实时性能不好，误差大。

**10、 Branch-and-bound registration**

**11、 Registration using local geometric features**

## 两个数据集的计算步骤：

  1. 识别最能代表两个数据集中的场景的兴趣点（interest points）（即关键点 keypoints）

  2. 在每个关键点处，计算特征描述符;

  3. 从特征描述符集合以及它们在两个数据集中的x,y,z位置，基于特征和位置之间的相似性来估计对应关系;

  4. 假设数据被认为包含噪声的，并不是所有的对应关系都是有效的，所以舍弃对配准过程产生负面影响的那些负影响对应关系;

  5. 利用剩余的正确的对应关系来估算刚体变换，完整配准。

**Feature based registration 配准**

1. SIFT 关键点 (pcl::SIFT…something)
2. FPFH 特征描述符  (pcl::FPFHEstimation)  
3. 估计对应关系 (pcl::CorrespondenceEstimation)
4. 错误对应关系的去除( pcl::CorrespondenceRejectionXXX )  
5. 坐标变换的求解

# 迭代最近点 （ICP）

ICP（Iterative Closest Point）算法**本质上是基于最小二乘法的最优配准方法**。

ICP算法对待拼接的2片点云，首先根据一定的准则确立对应点集P与Q，其中对应点对的个数，然后通过最小二乘法迭代计算最优的坐标变换，即旋转矩阵R和平移矢量t，使得误差函数最小。

该算法重复进行选择对应关系点对，计算最优刚体变换这一过程，直到满足正确配准的收敛精度要求。
算法的**输入**：**参考点云**和**目标点云**，停止迭代的标准。
算法的**输出**：旋转和平移矩阵，即**转换矩阵**。

使用点匹配时，使用点的XYZ的坐标作为特征值，**针对有序点云和无序点云数据的不同的处理策略**：
1. 穷举配准（brute force matching）;
2. kd树最近邻查询（FLANN）;
3. 在有序点云数据的图像空间中查找;
4. 在无序点云数据的索引空间中查找.

特征描述符匹配：
1. 穷举配准（brute force matching）;
2. kd树最近邻查询（FLANN）。

in cloud B for every point in cloud A .

**ICP处理流程分为四个主要的步骤**：

1. 对原始点云数据进行采样(关键点 keypoints(NARF, SIFT 、FAST、均匀采样 UniformSampling)、
特征描述符　descriptions，NARF、 FPFH、BRIEF 、SIFT、ORB )
2. 确定初始对应点集(匹配 matching )
3. 去除错误对应点对(随机采样一致性估计 RANSAC )
4. 坐标变换的求解

## 错误对应关系的去除（correspondence rejection）

由于噪声的影响，通常并不是所有估计的对应关系都是正确的，由于错误的对应关系对于最终的刚体变换矩阵的估算会产生负面的影响，所以必须去除它们，可以采用随机采样一致性估计，或者其他方法剔除错误的对应关系，最终只使用一定比例的对应关系，这样既能提高变换矩阵的估计精度也可以提高配准点的速度。

## 变换矩阵的估算（transormation estimation）的步骤

1. 在对应关系的基础上评估一些错误的度量标准
2. 在摄像机位姿（运动估算）和最小化错误度量标准下估算一个刚体变换(  rigid  transformation )
3. 优化点的结构  (SVD奇异值分解 运动估计;使用Levenberg-Marquardt 优化运动估计;)
4. 使用刚体变换把源旋转/平移到与目标所在的同一坐标系下，用所有点，点的一个子集或者关键点运算一个内部的ICP循环
5. 进行迭代，直到符合收敛性判断标准为止。

## PCL类的相关的介绍:

对应关系基类　　   pcl::CorrespondenceGrouping< PointModelT, PointSceneT >
几何相似性对应　   pcl::GeometricConsistencyGrouping< PointModelT, PointSceneT >
相似性度量　　　   pcl::recognition::HoughSpace3D
多实例对应关系　   pcl::Hough3DGrouping< PointModelT, PointSceneT, PointModelRfT, PointSceneRfT >
CRH直方图　　　   pcl::CRHAlignment< PointT, nbins_ >

随机采样一致性估计 pcl::recognition::ObjRecRANSAC::Output
      pcl::recognition::ObjRecRANSAC::OrientedPointPair
      pcl::recognition::ObjRecRANSAC::HypothesisCreator
      pcl::recognition::ObjRecRANSAC
      pcl::recognition::ORROctree::Node::Data
      pcl::recognition::ORROctree::Node
      pcl::recognition::ORROctree
      pcl::recognition::RotationSpace
