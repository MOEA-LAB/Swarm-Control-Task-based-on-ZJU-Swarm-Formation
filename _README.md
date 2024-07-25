### 如何做？

首先在normal_hexagon.launch文件中发现是在这里启动了6个`run_in_sim.launch`的无人机节点，因此为了找到单个无人机的位置，那就需要在该文件里面找对应的节点，在advanced_param.xml里面，发现了`drone_$(arg drone_id)_ego_planner_node`，那就说明这里就是主要的节点。经过搜索可以找到该节点的CMakeLists文件在`src/planner/plan_manage/CMakeLists.txt`

<img src="/Users/LYU/Library/Application Support/typora-user-images/image-20240721104227946.png" alt="image-20240721104227946" style="zoom:50%;" />

那么就可以确定，无人机的飞行节点肯定是在这几个cpp文件里面。经过全局搜索，找到了main函数在`ego_planner_node.cpp`文件里面，也就是路径：`src/planner/plan_manage/src/ego_planner_node.cpp`

<img src="/Users/LYU/Library/Application Support/typora-user-images/image-20240721104755504.png" alt="image-20240721104755504" style="zoom:50%;" />

可以看到主要的函数就是这一行

<img src="/Users/LYU/Library/Application Support/typora-user-images/image-20240721104848661.png" alt="image-20240721104848661" style="zoom:50%;" />

那我们继续找这个init的函数，经过全局搜索，在`ego_replan_fsm.cpp`里面找到了它的定义：

<img src="/Users/LYU/Library/Application Support/typora-user-images/image-20240721105134287.png" alt="image-20240721105134287" style="zoom:50%;" />

很明显六角形就在在这里面定义相对位置的，全局搜索变量`swarm_relative_pts_`，可以找到一个叫做`formationWaypointCallback`的函数，然后每次设定终点都会调用这个函数，那么说明在这里改相对位置就可以实现了。但是考虑到SYSU这里有三个字母，那么就需要重新修改一下参数，把SYU三个字母给画出来，那么我就在文件`init`的函数里面里面，多增加了三个double二维矩阵，`swarm_relative_pts_s_`，`swarm_relative_pts_y_`，`swarm_relative_pts_u_`，我用GPT帮我生成了无人机在这些字母上的相对位置的参数，如下所示：

```c++
// 初始化指定元素
swarm_relative_pts_s_[0][0] = 0.000;
swarm_relative_pts_s_[0][1] = 0.00;
swarm_relative_pts_s_[0][2] = 0.0;
swarm_relative_pts_s_[1][0] = 0.8825;
swarm_relative_pts_s_[1][1] = 0.7274;
swarm_relative_pts_s_[1][2] = 0.0;
swarm_relative_pts_s_[2][0] = -0.0104;
swarm_relative_pts_s_[2][1] = 1.5182;
swarm_relative_pts_s_[2][2] = 0.0;
// 后续省略，以及Y和U，参见代码
```

其余的代码就使用原来的代码就可以了，主要是在src/planner/plan_manage/src/ego_replan_fsm.cpp中的将relative_pos修改成上述的就可以了。

```cpp
  if (state == 0 || state == 2)
  {
    relative_pos << swarm_relative_pts_s_[id][0], swarm_relative_pts_s_[id][1],
        swarm_relative_pts_s_[id][2];
  }
  else if (state == 1)
  {
    relative_pos << swarm_relative_pts_y_[id][0], swarm_relative_pts_y_[id][1],
        swarm_relative_pts_y_[id][2];
  }
  else if (state == 3)
  {
    relative_pos << swarm_relative_pts_u_[id][0], swarm_relative_pts_u_[id][1],
        swarm_relative_pts_u_[id][2];
  }
  else
  {
    return;
  }
```

### 主程序

然后为了方便发布指令，不用每次都点击四次，所以我编写了`goal.py`作为主程序，当检测到所有的飞机都快到达指定位置的时候，就发送下一个目标，其实就是往move_base_simple/goal话题里面发送一个ros消息，`geometry_msgs/PoseStamped` 消息类型由两个部分组成：`Header header` 和 `geometry_msgs/Pose pose`。`Header` 包含消息的序列号 (`seq`)、时间戳 (`stamp`) 和坐标系 ID (`frame_id`)；`Pose` 包含位置 (`position`) 和姿态 (`orientation`)，位置使用 x, y, z 坐标表示，姿态使用四元数表示。我们用到的消息就是目标xy和大于0的z。然后利用python发送一个消息之后，无人机就会进行下一次的规划，并且改变队形。

最后我发现改变了编队但是连线并没有修改，那么经过一阵搜索之后，在`src/planner/traj_utils/src/planning_visualization.cpp`找到了连线的代码，我设计了一个`mycallback`函数，订阅`/move_base_simple/goal`的消息，用来每次接收到目标点之后更改连线规则，同理也是要增加一个计数器之后state之后，根据我所给定的连线规则连线，即可实现。

## 实现效果

参见附件中的demo.mp4（已加速4倍）
