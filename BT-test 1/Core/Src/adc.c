#include "adc.h"

double Detection(double data[], double baddata[], int datanum, int rule)
{
  double data_b[datanum];                                                                                                                                                                  // 临时存放保留的数据
  double v[datanum];                                                                                                                                                                       // 残差
  double g95[] = {1.15, 1.46, 1.67, 1.82, 1.94, 2.03, 2.11, 2.18, 2.23, 2.29, 2.33, 2.37, 2.41, 2.44, 2.47, 2.50, 2.53, 2.56, 2.58, 2.60, 2.62, 2.64, 2.66, 2.74, 2.81, 2.87, 2.96, 3.17}; // 格拉布斯95%
  double g99[] = {1.16, 1.49, 1.75, 1.94, 2.10, 2.22, 2.32, 2.41, 2.48, 2.55, 2.61, 2.66, 2.71, 2.75, 2.79, 2.82, 2.85, 2.88, 2.91, 2.94, 2.96, 2.99, 3.01, 3.10, 3.18, 3.24, 3.34, 3.58}; // 格拉布斯99%
  double bsl;                                                                                                                                                                              // 贝塞尔公式结果
  double maxdev;                                                                                                                                                                           // 有效的莱特 or 格拉布斯准则的最大偏差
  double sum;                                                                                                                                                                              // 累加临时存储
  double average;                                                                                                                                                                          // 平均值
  int badindex;                                                                                                                                                                            // 某次剔除数据数
  int validNum = 0;                                                                                                                                                                        // 有效数据数
  int proindex = 0;                                                                                                                                                                        // 循环的次数
  int badnum = 0;                                                                                                                                                                          // 初始化坏数据数
  double lg;                                                                                                                                                                               // 莱特 or 格拉布斯准则的系数
  int i;
  if (rule <= 3) // 当rule小于等于3时，直接用莱特系数3或自定义的rule值
    lg = rule;
  else if (rule > 5) // 当rule大于5时，强制设为莱特准则
    lg = 3;

  // 循环，直到有效数据个数小于等于5或没有坏数据为止
  while (1)
  {
    // 根据rule值选择不同的格拉布斯准则
    if (rule == 4) // 格拉布斯95%
    {
      if (datanum >= 100)
        lg = g95[27]; // 数据个数大于100个时
      else if (datanum >= 50)
        lg = g95[26];
      else if (datanum >= 40)
        lg = g95[25];
      else if (datanum >= 35)
        lg = g95[24];
      else if (datanum >= 30)
        lg = g95[23];
      else if (datanum >= 25) // 数据个数大于25个但小于30个时
        lg = g95[22];
      else // 数据个数小于25个时
        lg = g95[datanum - 3];
    }
    // 当rule为5时，使用格拉布斯99%准则
    else if (rule == 5) // 格拉布斯99%
    {
      if (datanum >= 100) // 数据个数大于100个时
        lg = g99[27];
      else if (datanum >= 50)
        lg = g99[26];
      else if (datanum >= 40)
        lg = g99[25];
      else if (datanum >= 35)
        lg = g99[24];
      else if (datanum >= 30)
        lg = g99[23];
      else if (datanum >= 25) // 数据个数大于25个但小于30个时
        lg = g99[22];
      else // 数据个数小于25个时
        lg = g99[datanum - 3];
    }
    proindex++; // 更新循环次数

    sum = 0;
    for (i = 0; i < datanum; i++)
      sum += data[i];
    average = sum / datanum; // 计算平均值

    sum = 0;
    for (i = 0; i < datanum; i++)
    {
      v[i] = data[i] - average; // 计算残差
      sum += v[i] * v[i];       // 计算残差平方和
    }

    bsl = sqrt(sum / (datanum - 1)); // 计算贝塞尔公式标准差
    maxdev = lg * bsl;               // 计算最大偏差

    // 剔除坏值，即剔除粗差数据
    validNum = 0;
    badindex = 0;
    for (i = 0; i < datanum; i++)
      if (fabs(v[i]) >= maxdev && maxdev != 0) // 当|Vi|>准则偏差值时
      {
        baddata[badnum++] = data[i]; // 将该Xi作为粗差数据，放入坏数据数组
        badindex++;
      }
      else
        data_b[validNum++] = data[i]; // 否则将效数数据暂存到data_b数组
    for (i = 0; i < validNum; i++)    // 将暂存的效数数据送回数据数组data
      data[i] = data_b[i];
    datanum = validNum; // 将当前有效数据个数作为数据个数
    // 判断是否满足停止条件
    if (datanum > 5) // 有效数据大于5个，则继续进行处理
    {
      if (badindex == 0) // 若没有可剔除的粗差数据
        break;           // 跳出循环，即粗差数据处理完毕
    }
    else
      break; // 有效数据小于等于5个，直接跳出循环
  }
  return average; // 子程序返回有效数据的均值
}
