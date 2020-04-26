# pcl::DifferenceOfNormalsEstimation

基于不同领域半径估计的　法线的差异类分割点云

步骤：
	1. 估计大领域半径 r_l 下的　法线    
	2. 估计small领域半径 r_ｓ 下的　法线 
	3. 法线的差异  det(n,r_l, r_s) = (n_l - n_s)/2
	4. 条件滤波器 
	5. 欧式聚类 法线的差异
