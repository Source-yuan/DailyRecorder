对该领域的一些**主要问题和方法**有一定的了解 不懂的记录下来。综述的参考出发去追踪（自底向上，追根溯源）



### 1.挑战：

实现真正强大的swarm（例如编队前行等），需要有强大的单体自主性。

也即;

​	**轻量化**

​	**鲁棒性**

​	**灵动性**



### 2.项目：

**轻量化**

https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan（离线规划，base project。N3ctrl的控制）

https://github.com/HKUST-Aerial-Robotics/Fast-Planner（fastplanner，在线规划）

  （egoplanner：一个全局的ESDF是低性价比的。实际只要完成occupyded 区域里轨迹的拉出即可）



**鲁棒性**

***External Forces Resilient Safe Motion Planning for Quadrotor,*** Yuwei Wu, Ziming Ding, Chao Xu, **Fei Gao\***, IEEE Robotics and Automation Letter (**RA-L**).（抗恒定扰动的规划）



**灵动性**



***Geometrically Constrained Trajectory Optimization for Multicopters\***, Zhepei Wang, Xin Zhou, Chao Xu, **Fei Gao***, submitted to IEEE Transactions on Robotics (**TRO**). [[preprint](https://arxiv.org/pdf/2103.00190.pdf)] [[code](http://zju-fast.com/gcopter)]（考虑更底层的运动学约束，在se3中搜索。实物验证部分用到：动作捕捉定位系统（nokov））



**编队**

***EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments***, Xin Zhou, Jiangchao Zhu, Hongyu Zhou, Chao Xu, **Fei Gao*******, IEEE International Conference on Robotics and Automation (**ICRA 2021**). [[preprint](https://arxiv.org/abs/2011.04183v1)] [[code](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)]（ego-planner的扩充，存在通讯）



### 3.无人机硬件架构

<img src="C:\Users\Ede\AppData\Roaming\Typora\typora-user-images\image-20210825232017031.png" alt="image-20210825232017031" style="zoom:80%;" />

#### 〇.飞控

科研常用PX4的飞控。最好有：内置减震和恒温

#### Ⅰ.电调（ESC）

分为 四合一\分体式

**力效表 **->额定电流

KV值

#### Ⅱ.机架

重点-轴距

#### Ⅲ.机载电脑

dji妙算（CPU强）

Xavier X（视觉识别，神经网络）



#### 4.无人机选型

<img src="C:\Users\Ede\AppData\Roaming\Typora\typora-user-images\image-20210825233139301.png" alt="image-20210825233139301" style="zoom:80%;" />

![image-20210825233542981](C:\Users\Ede\AppData\Roaming\Typora\typora-user-images\image-20210825233542981.png)



### 4.规划方法：







### 5.控制算法

#### Ⅰ.基本控制（悬停和低速）

**线性、小角度近似为悬停**

[上层电脑]->位置、速度、加速度->期望的推力和角速度->[飞控]

调节PID

【估计】会有漂移，只要缓慢并一致即可

项目：n3ctrl : https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/tree/experiment/onboard_computer/controller

![image-20210826102511126](C:\Users\Ede\AppData\Roaming\Typora\typora-user-images\image-20210826102511126.png)

![image-20210826110105700](C:\Users\Ede\AppData\Roaming\Typora\typora-user-images\image-20210826110105700.png)

![image-20210827164729197](C:\Users\Ede\AppData\Roaming\Typora\typora-user-images\image-20210827164729197.png)

#### Ⅱ.先进控制（se3）

**非线性、考虑机身姿势**

**【paper】：**Minimum snap trajectory generation and control for quadrotors,"2011 IEEE International Conference on Robotics and Automation, 2011, pp. 2520-2525, doi: 10.1109/ICRA.2011.5980409.

【p,v,a->姿态】

**【paper】：**Geometric tracking control of a quadrotor UAV on SE (3) for extreme maneuverability【se3奠基之作，2010年，可直接抄】

**CMPCC** ：鲁棒实验论文--Jijialin



#### Ⅲ.仿真

MATLAB

模块化的仿真



### 6.感知方法（perception）

 感知分为：语义和几何感知

传感器模型及特点：RGBD （TOF/结构光 ）

3D点云->导航地图表示形式





### 7.related notion：

vio 八叉树

ESDF

**外部定位：**需要高精度的室内定位，单独研究规划时——**动作捕捉定位系统（nokov）**

《最优化》	状态机

控制理论->系统、反馈。架构：p 内环 v 外环  a 前馈 【感知】反馈

扭矩

