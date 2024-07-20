## 实验报告

### 代码思路

根据一般的代码风格，主要节点一定都会在`plan_manage`的包里面的，然后里面只有三个 cpp 文件，一个是主要的节点文件，也就是主函数的文件，名字为`ego_planner_node.cpp`，这个文件只是用来调用主类的；一个就是`ego_replan_fsm.cpp`，这个就是实时飞行中有限状态机的类定义，看了一下里面有个`swarm_relative_pts`的变量，因此说明这个相关位置的目标点点就是在这里改的。最后一个是`palnner_manager.cpp`的文件，看了一下其实是飞行时候各种状态的操作。所以综上所述，最重要的节点一定在`ego_replan_fsm.cpp`里面。

这个代码的主要功能是根据我所给定的`2d goal`，然后给中心点规划一条主要的路径，然后其他相对中心点则根据`ego_planner`的性质实时规划避障。那么修改好这个`formationWaypointCallback`函数，那就可以实现自定义的目标转移，或者说相对位置改变。但是又有一个问题，就是单独设计 4 次目标点，实际飞出来的是，只有目标点的时候才符合 SYSU 的形状，因此我需要设计 7 次目标点，其逻辑是这样的

0. 目标点 x = -20, 从"S"开始
1. 目标点 x =-9, "S"持续飞一段时间
2. 目标点 x = -5, 转化为"Y"
3. 目标点 x = 2, "Y"持续飞一段时间
4. 目标点 x = 8, 转化为"S"
5. 目标点 x = 15, "S"持续飞一段时间
6. 目标点 x = 22, 转化为"U"
7. 目标点 x = 25, "U"持续飞一段时间

那么自然而然的就可以想到设计一个计数器 count，然后每次收到目标点就 count++。然后根据目标点的个数修改这个相对位置，以下是根据 count 的值进行形状改变的代码。

```cpp
switch (count){
  case 0,3,4:
    changeToShape('s', relative_pos);
  case 1,2:
    changeToShape('y', relative_pos);
  case 5,6:
    changeToShape('u', relative_pos);
}
```

这个 changeToShape 的代码如下所示：

```cpp
void EGOReplanFSM::changeToShape(const char shape, Eigen::Vector3d &relative_pos){
    switch (shape){
        case 's':
            relative_pos << s[id][0],s[id][1],s[id][2];
            break;
        case 'y':
            /* 省略 */
        case 'u':
            /* 省略 */
    }}
```

而对于如何定义这个s,y,u数组，我则把其放到`src/planner/plan_manage/config/normal_hexagon.yaml`文件里面，具体的如下：

```cpp
s0: { x: 0.000, y: 0.00, z: 0.0 }
s1: { x: 0.8025, y: 0.7474, z: 0.0 }
/* ... */
y0: { x: -0.4915, y: 0.5139, z: 0.0 }
/* ... */
u0: { x: -0.9975, y: 0.4788, z: 0.0 }
/* ... */
```

然后再使用`nh.param()`函数循环读取即可。

接着又有另外一个问题，连线问题，根据rviz里面的连线话题进行反向搜索，可以找到一个timer，这个timer每0.01执行一次：

```cpp
swarm_graph_visual_timer_ = nh.createTimer(
        ros::Duration(0.01), &PlanningVisualization::swarmGraphVisulCallback, this);
```

那么这个回调函数其实就是发布连线的函数，也就说修改这个函数里面的`line_begin_`和`line_end_`就可以实现连线互不相同。为此我需要设计一个新的回调函数来确定这个回调的次数，从而确定形状的连接方式，设计一个回调函数用来监听`move_base_simple/goal`的话题，如下：

```cpp
void PlanningVisualization::turnCallback(const geometry_msgs::PoseStampedPtr &msg){
  count++;
  if (count >= 2){shape = 'y';}
  else if (count >= 4){shape = 's';}
  else if (count >= 6){shape = 'u';}
}
```

然后根据这个shape，就可以在timer的回调函数`swarmGraphVisulCallback`里面进行修改，增加一个段落：

```cpp
switch (shape){
  case 's': /* S */
    line_begin_ = {1, 2, 3, 0, 4, 5};
    line_end_ = {2, 3, 0, 4, 5, 6};
    break;
  case 'y': /* Y */
		/* 省略 */
  case 'u': /* U */
    /* 省略 */}
```

那么就可以基本实现了飞出`SYSU`的效果了，那么最后使用了一个python文件启动了一个名字为`swarm_control_node`节点用来检测每个飞机的到达情况，然后只要所有飞机在一定的误差下达到了期望位置，比如说所有飞机都抵达了目标`S`形状，并且所有飞机的欧氏距离小于一个阈值之后，就发布下一个目标点，然后依次类推。其逻辑与上述的` 7 次目标点`思路相同，便不再赘述。

### 附录

- 所有的代码已经提交，具体运行方法在`README.md`里面。
- 示例的飞行视频在`result.gif`里面。
