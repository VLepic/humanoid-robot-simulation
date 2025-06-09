import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import yarp

class WalkingBridge(Node):
    def __init__(self):
        super().__init__('walking_bridge')

        # YARP init
        yarp.Network.init()

        # Cíl (stream do walking-coordinator)
        self.goal_port = yarp.Port()
        self.goal_port.open("/ros2/walking_bridge/goal:o")
        yarp.Network.connect("/ros2/walking_bridge/goal:o", "/walking-coordinator/goal:i")

        # RPC port pro příkazy
        self.rpc = yarp.RpcClient()
        self.rpc.open("/ros2/walking_bridge/rpc")
        yarp.Network.connect("/ros2/walking_bridge/rpc", "/walking-coordinator/rpc")

        # ROS 2 subs a služby
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_service(Trigger, '/prepare_robot', self.prepare_cb)
        self.create_service(Trigger, '/start_walking', self.start_cb)

        self.get_logger().info("Bridge hotov: přijímá /cmd_vel → posílá goal:x y theta")

    def send_rpc(self, command: str):
        cmd = yarp.Bottle()
        reply = yarp.Bottle()
        cmd.addString(command)
        self.rpc.write(cmd, reply)
        self.get_logger().info(f"[RPC] {command} → {reply.toString()}")
        return reply.toString()

    def prepare_cb(self, request, response):
        result = self.send_rpc("prepareRobot")
        response.success = "ok" in result
        response.message = result
        return response

    def start_cb(self, request, response):
        result = self.send_rpc("startWalking")
        response.success = "ok" in result
        response.message = result
        return response

    def cmd_vel_callback(self, msg: Twist):
        ahead = msg.linear.x
        sides = msg.linear.y
        turn = msg.angular.z

        bottle = yarp.Bottle()
        bottle.clear()
        bottle.addFloat64(ahead)
        bottle.addFloat64(turn)
        bottle.addFloat64(sides)

        self.goal_port.write(bottle)
        self.get_logger().info(f"[setGoal] Sent movement command: ahead={ahead:.2f}, turn={turn:.2f}, side step={sides:.2f}")

    def destroy_node(self):
        self.goal_port.close()
        self.rpc.close()
        yarp.Network.fini()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WalkingBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



