#pragma once

#include<Eigen/Dense>



/*
 * 把这个头文件和源代码放一个目录下不合理，这里只是为了演示方便
 * 
*/

// to doooooooooooooo
class Filter{
public:
  // 构造函数，explicit表示只能显示的调用构造函数创建一个滤波器对象
  explicit Filter();
  
  // 更新IMU数据，完成一次惯导推算
  void updateImu();

  // 更新GNSS数据
  void updateGnss();

  // 完成一次互补滤波算法，并进行误差反馈
  void fused_gnss_ins();
}