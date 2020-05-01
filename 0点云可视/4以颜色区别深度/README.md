# 以颜色区别深度

该方法(PointCloudColorHandlerGenericField)将不同的深度值显示为不同的颜色，实现以颜色区分深度的目的；方法PointCloudColorHandlerCustom是将点云作为整体并统一着色，PointCloudColorHandlerGenericField方法是将点云按深度值(“x”、“y”、"z"均可)的差异着以不同的颜色。

# 其它

以上代码的不同处：

```c++
pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);   //显示点云颜色特征
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0); // 自定义点云颜色特征，此处为green
pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z"); // 以颜色区别深度，按照z字段进行渲染
```

