# ros topic可视化工具
## 1.HMI数据监控
![alt text](docs/images/image.png)
### HMI显示功能介绍
hmi按键&大屏开关:未按下,图标为灰色,按下为黄色
![alt text](docs/images/image-1.png)

watchdog状态(目前只加入了图标中的,后续需要可以再加入)
![alt text](docs/images/image-2.png)

status_report对应topic内部变量状态(可以自定义)
![alt text](docs/images/image-3.png)

数据曲线:
![alt text](docs/images/image-4.png)

 <span style="color:red">Note:</span>
 - local time为程序运行时的系统时间
 - 曲线数据可以动态增加或者删减.



## 2.车辆横向状态监控
![alt text](docs/images/image-5.png)

#### 目前监控的变量:
- <span style="color:green">adas状态方向盘颜色</span>
- <span style="color:green">方向盘状态:方向盘转角,角速度,扭矩</span>
- <span style="color:green">车辆状态:轮速,角速度</span>           
- <span style="color:green">学习算法数据:方向盘零偏扭矩学习结果</span>



## 3.车辆纵向状态监控
![alt text](docs/images/image-6.png)

#### 目前监控的变量:

- <span style="color:green">车辆状态(反馈):速度,加速度,轮速,车辆姿态</span>
- <span style="color:green">控制数据:期望加速度,刹车期望</span>
- <span style="color:green">刹车状态:刹车压力....</span>
- <span style="color:green">刹车计算相关数据</span>

