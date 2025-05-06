#pragma once

#include<Eigen/Dense>



/*
 * 把这个头文件和源代码放一个目录下不合理，这里只是为了演示方便
*/

// to doooooooooooooo
class Filter{
public:
  // 构造函数
  Filter();
  
  // 更新IMU数据，完成一次惯导推算
  void updateImu();

  // 更新GNSS数据
  void updateGnss();

  // 判断是否需要对IMU数据内插
  bool is_toInterpolate();

  // 完成一次互补滤波算法，并进行误差反馈
  void fused_gnss_ins();
}