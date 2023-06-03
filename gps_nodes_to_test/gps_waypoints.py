import utm
import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Path, NavSatFix
from geometry_msgs.msg import PoseStamped


class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')
        self.follow_waypoints_client = self.create_client(FollowWaypoints, 'follow_waypoints')

        while not self.follow_waypoints_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Follow Waypoints service not available, waiting...')

        # get robot gps coordinates
        self.goal_gps_coordinates = self.get_parameter('goal_gps_coordinates').value.split(';')
        for i in range(len(self.goal_gps_coordinates)):
            lon = self.goal_gps_coordinates[i].split(',')[0]
            lat = self.goal_gps_coordinates[i].split(',')[1]

            self.goal_gps_coordinates[i] = (float(lon), float(lat))

        self.waypoints = []
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps_coordinates',
            self.gps_coordinates_callback,
            10
        )
        self.gps_subscription

    
    def gps_coordinates_callback(self, msg):
        robot_utm_coords = utm.from_latlon(msg.longitude, msg.latitude)
        for coords in self.goal_gps_coordinates:
            goal_utm_coords = utm.from_latlon(coords[0], coords[1])
            goal_waypoint = PoseStamped()
            goal_waypoint.pose.position.x = goal_utm_coords[0] - robot_utm_coords[0]
            goal_waypoint.pose.position.y = goal_utm_coords[1] - robot_utm_coords[1]
            goal_waypoint.pose.position.z = 0.0

            self.waypoints.append(goal_waypoint)


    def send_waypoints(self):
        goal_msg = FollowWaypoints.Goal()
        path_msg = Path()
        path_msg.poses = self.waypoints

        goal_msg.path = path_msg

        self.follow_waypoints_client.wait_for_service()
        self.follow_waypoints_client.send_goal_async(goal_msg)


    def shutdown(self):
        self.follow_waypoints_client.destroy()


def main():
    rclpy.init()
    node = WaypointFollowerNode()
    rclpy.spin_once(node)

    try:
        while True:
            # Send the waypoints
            node.send_waypoints()

    except KeyboardInterrupt:
        pass
            
    node.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
