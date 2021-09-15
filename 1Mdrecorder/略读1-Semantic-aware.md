**Semantic-aware** **Active Perception** for UAVs using **Deep Reinforcement Learning**



target at：对于vision-based navigation的UAV（比如依赖vision-based state estimation），要避免飞跃感知退化区（perceptually degraded regions.） 



solution：(语义线索（semantic cues）等价某区域的信息量)。用强化学习识别高信息量区域：深度策略网（deep policy network）识别每个语义类(semantic class)的相对信息感知力->**形式的信息->轨迹优化。



result（comparison）：

视频：3D高渲染环境，20次飞行成功率、速度、轨迹可视化。

图1：训练集展示（获奖曲线和收敛曲线）

表1：多实验场景下（赛马场、村庄、公路等，20+次的）success rate、 average localization RMSE、missed distance from the goal 、path length。

![image-20210908104534621](C:\Users\Ede\AppData\Roaming\Typora\typora-user-images\image-20210908104534621.png)

图2：轨迹可视化(ego：*绿色*）

主要看：是否经过危险区域(如湖泊、森林-*青色*)、摄像机状态估计系统是否出现多弯路(跟随合适的landmark->定位？-*黄色*)。成功率低-*红色*

![image-20210908105004359](C:\Users\Ede\AppData\Roaming\Typora\typora-user-images\image-20210908105004359.png)



方法：为场景中的每个语义类分配重要权重，将语义信息映射到感知信息



分析：

