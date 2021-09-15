## 三部曲

### paper[1]FastPlanner：



### paper[2]：PGO-Robust real-time uav replanning using guided gradient-based optimization and topological paths,

##### Ⅰ.PGO:前端路径搜索算法

warmup trajectory

##### Ⅱ.拓扑等价关系:

UVD类路径的等效路径会随着障碍物的增加呈指数增长。所以我们每次只选几条路径，譬如Kmax条，然后对于长度大于最短路径R max 倍的路径我们也会丢弃掉


1.replan time/ topologicall quivalent.

*B. Zhou, F. Gao, J. Pan, and S. Shen, “Robust real-time uav replanning*
*using guided gradient-based optimization and topological paths,” in*
*Proc. of the IEEE Intl. Conf. on Robot. and Autom. (ICRA). IEEE,*
*2020, to appear. [Online]. Available: https://arxiv.org/abs/1912.12644*

2.多geometric guiding paths路paths并行引导优化

3.optimistic assumption or 实验条件



### paper[3]：RAPTOR-



a risk-aware trajectory refinement approach :safe reaction distance

[Two-step motion planning framework] : add yaw angle->smater the FOV



VI：多路引优化	PGO		

V：拓扑路径选择标准     

**VD**-L. Jaillet and T. Sim´eon, “Path deformation roadmaps: Compact graphs
with useful cycles for motion planning,” The International Journal of
Robotics Research, vol. 27, no. 11-12, pp. 1175–1188, 2008.

**UVD** :is more efficient for equivalence checking.

VI：







eliminate

local minima

 discrete state space

deforms

piece-wise straight line paths

detoured.

Visibility Metric