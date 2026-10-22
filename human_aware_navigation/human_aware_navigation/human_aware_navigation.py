from air_simple_sim_msgs.msg import SemanticObservation
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from air_lab_interfaces.msg import Human, Humans


class HumanAwareNavigationNode(Node):
    def __init__(self):
        super().__init__('human_aware_navigation_node')
        self.subscription = self.create_subscription(
            SemanticObservation,
            'semantic_sensor_hf',
            self.semantic_sensor_callback,
            10
        )

        self.humans_publisher = self.create_publisher(
            Humans,
            '/humans',
            10
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.humans = Humans()
    
    def parse_semantic_sensor_message(self, msg):
        uuid = msg.uuid
        klass = msg.klass
        point = msg.point

        point.header.stamp = rclpy.time.Time().to_msg()

        return uuid, klass, point

    def transform_point_to_map(self, point):
        try:
            transformed_point = self.tf_buffer.transform(
                point, "map", timeout=rclpy.duration.Duration(seconds=1.0))
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {point} to map: {ex}')
            return None
        return transformed_point

    def semantic_sensor_callback(self, msg):
        uuid, klass, point = self.parse_semantic_sensor_message(msg)

        if klass != "human":
            return

        transformed_point = self.transform_point_to_map(point)

        if transformed_point == None:
            return
        
        human = Human()
        human.uuid = uuid
        human.position = transformed_point.point
        human.stamp = transformed_point.header.stamp

        # Add/update the human (thanks to uuid) in self.humans
        for i, existing_human in enumerate(self.humans.humans):
            if existing_human.uuid == uuid:
                self.humans.humans[i] = human
                break
        else:
            self.humans.humans.append(human)

        # Print list of humans
        self.get_logger().info(f'Current humans: {[h.uuid for h in self.humans.humans]}')

        # Publish the updated humans list
        self.humans_publisher.publish(self.humans)



def main(args=None):
    rclpy.init(args=args)
    
    human_aware_navigation_node = HumanAwareNavigationNode()

    rclpy.spin(human_aware_navigation_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    human_aware_navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
