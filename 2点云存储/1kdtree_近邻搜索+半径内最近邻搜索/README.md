#  kdtree 近邻搜索
​    template<typename PointT> 
​    virtual int pcl::KdTree< PointT >::nearestKSearch  ( const PointT &  p_q,  
​                                                        int  k,  
​                                                        std::vector< int > &  k_indices,  
​                                                        std::vector< float > &  k_sqr_distances  
​                                                        )  const [pure virtual] 
​    Search for k-nearest neighbors for the given query point. 
​    Parameters:
​        [in] the given query point 
​        [in] k the number of neighbors to search for  
​        [out] the resultant indices of the neighboring points
​        [out] the resultant squared distances to the neighboring points
​    Returns:
​        number of neighbors found 

