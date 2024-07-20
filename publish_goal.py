import rospy
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import time

# 读取YAML配置文件
def read_yaml(file_path):
    with open(file_path, 'r', encoding='utf-8') as file:
        config = yaml.safe_load(file)
    return config

# 计算两点之间的欧氏距离
def calculate_distance(pos1, pos2):
    return ((pos1['x'] - pos2['x'])**2 + (pos1['y'] - pos2['y'])**2 + (pos1['z'] - pos2['z'])**2)**0.5

# 回调函数，处理订阅的Odometry消息
def odometry_callback(msg, args):
    drone_id, odom_data = args
    odom_data[drone_id] = {
        'x': msg.pose.pose.position.x,
        'y': msg.pose.pose.position.y,
        'z': msg.pose.pose.position.z
    }

# 检查点是否被占用
def check_occupied(points, pos, threshold=0.05):
    for point in points:
        if calculate_distance({'x': point[0], 'y': point[1], 'z': point[2]}, pos) < threshold:
            return True
    return False

# 回调函数，处理订阅的PointCloud2消息
def pointcloud_callback(msg, args):
    points, pointcloud_data = args
    points.clear()
    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append(point)
    pointcloud_data['points'] = points

def main():
    rospy.init_node('swarm_control_node')

    file_path = 'src/planner/plan_manage/config/normal_hexagon.yaml'
    config = read_yaml(file_path)

    swarm_scale = config['global_goal']['swarm_scale']
    relative_positions_s = [config['global_goal'][f's{i}'] for i in range(7)]
    relative_positions_y = [config['global_goal'][f'y{i}'] for i in range(7)]
    relative_positions_u = [config['global_goal'][f'u{i}'] for i in range(7)]
    relative_positions = [relative_positions_s, relative_positions_y, relative_positions_s, relative_positions_u]

    x_list = [-9, -5, 2, 8,15,22,25]
    shape = [0,1,1,2,2,3,3]
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    odom_data = [{} for _ in range(7)]
    for i in range(7):
        rospy.Subscriber(f'/drone_{i}_visual_slam/odom', Odometry, odometry_callback, callback_args=(i, odom_data))

    pointcloud_data = {'points': []}
    rospy.Subscriber('/map_generator/global_cloud', PointCloud2, pointcloud_callback, callback_args=(pointcloud_data['points'], pointcloud_data))

    rate = rospy.Rate(5)  # 1 Hz
    current_goal_index = 0

    '''
     FIRST GOAL
    '''
    new_goal = PoseStamped()
    new_goal.header.stamp = rospy.Time.now()
    new_goal.pose.position.x = x_list[current_goal_index]
    new_goal.pose.position.y = 0
    new_goal.pose.position.z = 0

    # 检查是否有相对点被占用
    while True:
        occupied = False
        for i in range(7):
            relative_position = relative_positions[shape[current_goal_index]][i]
            central_position = {'x': new_goal.pose.position.x, 'y': 2.5, 'z': 0}
            expected_position = {
                'x': central_position['x'] + relative_position['x'] * swarm_scale,
                'y': central_position['y'] + relative_position['y'] * swarm_scale,
                'z': central_position['z'] + relative_position['z'] * swarm_scale
            }

            if check_occupied(pointcloud_data['points'], expected_position):
                occupied = True
                break

        if occupied:
            new_goal.pose.position.x += 0.3
            rospy.loginfo(f"Updated goal x due to occupation: x = {new_goal.pose.position.x}")
        else:
            break

    goal_publisher.publish(new_goal)
    rospy.loginfo(f"Published new goal: x = {new_goal.pose.position.x}")

    while not rospy.is_shutdown() and current_goal_index < len(x_list):
        all_within_threshold = True
        all_distance = 0
        for i in range(7):
            if not odom_data[i]: # not null
                all_within_threshold = False
                break

            relative_position = relative_positions[shape[current_goal_index]][i]
            central_position = {'x': new_goal.pose.position.x, 'y': 0, 'z': 0}
            expected_position = {
                'x': central_position['x'] + relative_position['x'] * swarm_scale,
                'y': central_position['y'] + relative_position['y'] * swarm_scale,
                'z': central_position['z'] + relative_position['z'] * swarm_scale
            }
            all_distance += calculate_distance(odom_data[i], expected_position)

            # if calculate_distance(odom_data[i], expected_position) > 0.5:
            #     all_within_threshold = False
            #     print(f"distance: {calculate_distance(odom_data[i], expected_position)}")
            #     break
        print(f"all_distance: {all_distance}")
        if all_distance <4 and all_distance!=0:
            print(f"all_distance: {all_distance}")
            current_goal_index += 1
            if current_goal_index < len(x_list): # less than 4
                new_goal.header.stamp = rospy.Time.now()
                new_goal.pose.position.x = x_list[current_goal_index]
                new_goal.pose.position.y = 0
                new_goal.pose.position.z = 0

                # 检查是否有相对点被占用
                while True:
                    occupied = False
                    for i in range(7):
                        relative_position = relative_positions[shape[current_goal_index%7]][i]
                        central_position = {'x': new_goal.pose.position.x, 'y': 0, 'z': 0}
                        expected_position = {
                            'x': central_position['x'] + relative_position['x'] * swarm_scale,
                            'y': central_position['y'] + relative_position['y'] * swarm_scale,
                            'z': central_position['z'] + relative_position['z'] * swarm_scale
                        }

                        if check_occupied(pointcloud_data['points'], expected_position):
                            occupied = True
                            break
                    if occupied:
                        new_goal.pose.position.x += 0.3
                        rospy.loginfo(f"Updated goal x due to occupation: x = {new_goal.pose.position.x}")
                    else:
                        break

                goal_publisher.publish(new_goal)
                rospy.loginfo(f"Published new goal: x = {new_goal.pose.position.x}")
        else :
            new_goal.header.stamp = rospy.Time.now()
            goal_publisher.publish(new_goal)
        

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
