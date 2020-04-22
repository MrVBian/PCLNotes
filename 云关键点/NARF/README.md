# NARF

Normal Aligned Radial Feature关键点是为了**从深度图像中识别物体**而提出的。对NARF关键点的提取过程有以下要求：

1. 提取的过程考虑边缘以及物体表面变化信息在内；

2. 在不同视角关键点可以被重复探测；

3. 关键点所在位置有足够的支持区域，可以计算描述子和进行唯一的估计法向量

其对应的探测步骤如下：

1. 遍历每个深度图像点，通过寻找在近邻区域有深度变化的位置进行边缘检测。

2. 遍历每个深度图像点，根据近邻区域的表面变化决定一测度表面变化的系数，及变化的主方向。

3. 根据step(2)找到的主方向计算兴趣点，表征该方向和其他方向的不同，以及该处表面的变化情况，即该点有多稳定。

4. 对兴趣值进行平滑滤波。

5. 进行无最大值压缩找到的最终关键点，即为NARF关键点。  

类 NarfKeypoint 实现提取 NARF(Normal Aligned Radial Feature）关键点，输入为一副距离图像，输出为 NARF 关键点，该类检测到的 NARF 常与 NARF点特征描述子配合使用，以便后期配准、识别等应用。



类NarfKeypoint：

|                                                              | [NarfKeypoint](http://docs.pointclouds.org/trunk/classpcl_1_1_narf_keypoint.html#a7261c9792e464f6452eeb2fad3036b24) ([RangeImageBorderExtractor](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image_border_extractor.html) *range_image_border_extractor=nullptr, float support_size=-1.0f) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
|                                                              | 重构函数， range_image_border_extractor 是对距离图像进行边缘检测的对象指针，默认为空， support_size 为检测是用支持域的大小。 |
| void                                                         | clearData()                                                  |
|                                                              | 删除与当前距离图像计算所得到的相关数据。                     |
| void                                                         | [setRangeImageBorderExtractor](http://docs.pointclouds.org/trunk/classpcl_1_1_narf_keypoint.html#a9b101e191efa338cb42761b00382c879) ([RangeImageBorderExtractor](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image_border_extractor.html) *range_image_border_extractor) |
|                                                              | 设置 range_image_border_extractor 是对距离图像进行边缘检测的对象指针，此项在计算之前必须设置。 |
| void                                                         | [setRangeImage](http://docs.pointclouds.org/trunk/classpcl_1_1_narf_keypoint.html#a86b01311b9bed5200964ad464033b02d) (const [RangeImage](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html) *range_image) |
|                                                              | 设置输入的距离图像。                                         |
| float *                                                      | [getInterestImage](http://docs.pointclouds.org/trunk/classpcl_1_1_narf_keypoint.html#ab5354883380de350b56d7a04ddae2b88) () |
|                                                              | 获取距离图像中每个点的感兴趣值 。                            |
| const ::[pcl::PointCloud](http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html)< [InterestPoint](http://docs.pointclouds.org/trunk/structpcl_1_1_interest_point.html) > & | [getInterestPoints](http://docs.pointclouds.org/trunk/classpcl_1_1_narf_keypoint.html#a4330f1b3b2962e8df799592a45ce6736) () |
|                                                              | 获取距离图像中关键点，返回对象 InterestPoint 中存储了点以及改点对应的感兴趣值。 |
| const std::vector< bool > &                                  | [getIsInterestPointImage](http://docs.pointclouds.org/trunk/classpcl_1_1_narf_keypoint.html#ad0be2b830e18c364b680df615e0fb442) () |
|                                                              | 返回的布尔值向量中包含整幅距离图像各个点是否为关键点，距离图像中为关键点的设置为 true ，否则设置为 false . |
| [Parameters](http://docs.pointclouds.org/trunk/structpcl_1_1_narf_keypoint_1_1_parameters.html) & | [getParameters](http://docs.pointclouds.org/trunk/classpcl_1_1_narf_keypoint.html#a2ab427b4391a1f855cdbe83d76e7a467) () |
|                                                              | 获取 Parameters 结构体对象引用，其中存储了与 Narf 关键点提取算法的很多参数： float support_size 检测关键点的支持区域。 int max_no_of_interest_points 返回关键点数目的上限 。 float min_distance_between_interest_points 关键点之间的最小距离，影响支持区域。 float optimal_distance_to_high_surface_change 关键点与表面特征变化大的位置之间的距离。 float min interest value 候选关键点感兴趣值下限。 float min_s.urface_change_score  候选关键点表面特征变化大小下限。 int optimal_range_image_patch_size 计算每个关键点时考虑的距离图像大小（像素〉。 float distance_for_additional_points 所有与最大兴趣值点之间距离在 distance_for_additional_points 之内的点，只要感兴趣值大于 min_interest_value ，该点则成为关键点候选点。 bool add_points_on_straight_edges 如果该值设置为 true ，则在空间直的边缘处添加关键点，表示空间曲面变化大。 bool do_non_max.imum_suppression 如果该值设置为 false，只要大于 min_interest_ value 兴趣值的点都会被加入到关键点队列。 bool no_of_polynomial_approximations_per_point 如果该值设置为 true ，则对于每个关键点的位置，需要采用双变量多项式插值获取，这样获取的位置更精确。 int max no of threads 采用 OpenMP 机制时，设置的建立线程数的上限。 bool use recursive scale reduction 如果设置为真，距离图像在小范围内有多个关键点，则调整小分辨率，加快计算速度。 bool calculate_sparse_interest_image 使用启发式机制，自适应调整那些距离图像区域不需要计算关键点，从而加快计算速度 。 |

# help


用法　Usage: ./show [options] <file.ply>

Options:
-------------------------------------------
-r <float>   角度　angular resolution in degrees (default 0.5)
-c <int>     坐标系　coordinate frame (default 0)
-m           将所有看不见的点视为最大范围读数
-s <float>   感兴趣点的尺寸（球面的直径 - default 0.2 )
-h           this help


./show ../../datas/SphereDivision.ply
