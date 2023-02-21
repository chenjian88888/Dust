# Dust
An autonomous vehicle software's frame used by beihang university.
1. 入口
planning.cc
2. reference_line构造函数
订阅、发布话题空间的创造，初始化planningbase。将其指向具体的planning方法（lattice）
3. 进入的reference_line.run函数
订阅障碍物
平滑参考线，将平滑后的结果储存在referenceline_
利用函数referencePointsCalc（）计算参考点的Kappa theta，将最终的参考点数据放在容器reference_points中
4. 判断参考点有数据并且订阅到gps信号进入下一步
计算规划起点、lattice规划、将lattice的结果发布出去供control接收
5. 问题
1.静态障碍物car的sl boundary怎么算的？
obstacle->polygon_points
2.planning节点在运行后频率大约在10hz，但是一旦启动control节点频率就会直线下降，这是为啥？
