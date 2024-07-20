import rospy
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def main():
    rospy.init_node('multi_drone_control_node')

    yaml_path = 'src/planner/plan_manage/config/normal_hexagon.yaml'
    cfg = load_config(yaml_path)

    formation_scale = cfg['global_goal']['swarm_scale']
    rel_pos_s = [cfg['global_goal'][f's{i}'] for i in range(7)]
    rel_pos_y = [cfg['global_goal'][f'y{i}'] for i in range(7)]
    rel_pos_u = [cfg['global_goal'][f'u{i}'] for i in range(7)]
    rel_positions = [rel_pos_s, rel_pos_y, rel_pos_s, rel_pos_u]

    x_coords = [-9, -5, 2, 8, 15, 22, 25]
    formations = [0, 1, 1, 2, 2, 3, 3]
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    odometry_info = [{} for _ in range(7)]
    for i in range(7):
        rospy.Subscriber(f'/drone_{i}_visual_slam/odom', Odometry, handle_odometry, callback_args=(i, odometry_info))

    pcl_data = {'points': []}
    rospy.Subscriber('/map_generator/global_cloud', PointCloud2, handle_pointcloud, callback_args=(pcl_data['points'], pcl_data))

    rate = rospy.Rate(5)
    current_idx = 0

    '''
     第一个目标
    '''
    target_goal = PoseStamped()
    target_goal.header.stamp = rospy.Time.now()
    target_goal.pose.position.x = x_coords[current_idx]
    target_goal.pose.position.y = 0
    target_goal.pose.position.z = 0

    # 检查是否有点被占用
    while True:
        is_occupied_flag = False
        for i in range(7):
            rel_pos = rel_positions[formations[current_idx]][i]
            central_pos = {'x': target_goal.pose.position.x, 'y': 2.5, 'z': 0}
            expected_pos = {
                'x': central_pos['x'] + rel_pos['x'] * formation_scale,
                'y': central_pos['y'] + rel_pos['y'] * formation_scale,
                'z': central_pos['z'] + rel_pos['z'] * formation_scale
            }

            if is_occupied(pcl_data['points'], expected_pos):
                is_occupied_flag = True
                break

        if is_occupied_flag:
            target_goal.pose.position.x += 0.3
            rospy.loginfo(f"目标x因被占用而更新: x = {target_goal.pose.position.x}")
        else:
            break

    goal_pub.publish(target_goal)
    rospy.loginfo(f"发布新目标: x = {target_goal.pose.position.x}")

    while not rospy.is_shutdown() and current_idx < len(x_coords):
        within_threshold = True
        total_dist = 0
        for i in range(7):
            if not odometry_info[i]:
                within_threshold = False
                break

            rel_pos = rel_positions[formations[current_idx]][i]
            central_pos = {'x': target_goal.pose.position.x, 'y': 0, 'z': 0}
            expected_pos = {
                'x': central_pos['x'] + rel_pos['x'] * formation_scale,
                'y': central_pos['y'] + rel_pos['y'] * formation_scale,
                'z': central_pos['z'] + rel_pos['z'] * formation_scale
            }
            total_dist += compute_euclidean_distance(odometry_info[i], expected_pos)

        if total_dist < 4 and total_dist != 0:
            current_idx += 1
            if current_idx < len(x_coords):
                target_goal.header.stamp = rospy.Time.now()
                target_goal.pose.position.x = x_coords[current_idx]
                target_goal.pose.position.y = 0
                target_goal.pose.position.z = 0

                # 检查是否有点被占用
                while True:
                    is_occupied_flag = False
                    for i in range(7):
                        rel_pos = rel_positions[formations[current_idx % 7]][i]
                        central_pos = {'x': target_goal.pose.position.x, 'y': 0, 'z': 0}
                        expected_pos = {
                            'x': central_pos['x'] + rel_pos['x'] * formation_scale,
                            'y': central_pos['y'] + rel_pos['y'] * formation_scale,
                            'z': central_pos['z'] + rel_pos['z'] * formation_scale
                        }

                        if is_occupied(pcl_data['points'], expected_pos):
                            is_occupied_flag = True
                            break
                    if is_occupied_flag:
                        target_goal.pose.position.x += 0.3
                        rospy.loginfo(f"目标x因被占用而更新: x = {target_goal.pose.position.x}")
                    else:
                        break

                goal_pub.publish(target_goal)
                rospy.loginfo(f"发布新目标: x = {target_goal.pose.position.x}")
        else:
            target_goal.header.stamp = rospy.Time.now()
            goal_pub.publish(target_goal)
        
        rate.sleep()

# 加载YAML配置文件
def load_config(yaml_path):
    with open(yaml_path, 'r', encoding='utf-8') as yaml_file:
        cfg = yaml.safe_load(yaml_file)
    return cfg

# 计算三维空间中的欧几里得距离
def compute_euclidean_distance(p1, p2):
    return ((p1['x'] - p2['x'])**2 + (p1['y'] - p2['y'])**2 + (p1['z'] - p2['z'])**2)**0.5

# 处理Odometry消息的回调函数
def handle_odometry(msg, args):
    drone_idx, odom_info = args
    odom_info[drone_idx] = {
        'x': msg.pose.pose.position.x,
        'y': msg.pose.pose.position.y,
        'z': msg.pose.pose.position.z
    }

# 检查某个点是否被占用
def is_occupied(pts, position, distance_threshold=0.05):
    for pt in pts:
        if compute_euclidean_distance({'x': pt[0], 'y': pt[1], 'z': pt[2]}, position) < distance_threshold:
            return True
    return False

# 处理PointCloud2消息的回调函数
def handle_pointcloud(msg, args):
    pts, pcl_data = args
    pts.clear()
    for pt in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        pts.append(pt)
    pcl_data['points'] = pts


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
