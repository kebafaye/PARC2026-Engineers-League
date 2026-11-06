#!/usr/bin/env python3
"""Republish Nav2's Twist as the TwistStamped the physical Sito-E actually drives on.

This node exists because of one measurement, taken from a rosbag recorded on the real
robot (406 s, 2026-11-06). Two command topics are live on that machine:

    /cmd_vel                      geometry_msgs/Twist          8321 msgs
    /botwheel_explorer/cmd_vel    geometry_msgs/TwistStamped   9989 msgs

Comparing each against /botwheel_explorer/odom 0.3 s later - what the wheels actually
did - settles which one the base obeys:

    /cmd_vel                    mean error 0.5173
    /botwheel_explorer/cmd_vel  mean error 0.0028      <- 180x closer

So the real robot is driven by TwistStamped on /botwheel_explorer/cmd_vel. Nav2 on
Jazzy publishes plain Twist and has no stamped option: `enable_stamped_cmd_vel`
appears in neither the collision_monitor, velocity_smoother nor controller_server
binaries, nor in nav2_bringup's reference params. Remapping the topic name alone
therefore cannot work - the message type is wrong too, and a type mismatch does not
error, it simply never connects.

Left unfixed this is the same silent failure that cost the seed 12 runs in simulation,
where collision_monitor published to a topic with no subscriber and the robot never
moved while Nav2 logged action-server timeouts that read exactly like a saturated
host. The difference is that on the real robot there is no second chance.

Not needed in simulation, where the Gazebo bridge takes plain Twist on /cmd_vel.
"""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class CmdVelStamper(Node):
    def __init__(self):
        super().__init__("cmd_vel_stamper")

        self.declare_parameter("in_topic", "/cmd_vel")
        self.declare_parameter("out_topic", "/botwheel_explorer/cmd_vel")
        # The frame the velocity is expressed in. base_link on the real robot: its
        # URDF, read from the same rosbag, declares exactly five links - base_link,
        # left_wheel, right_wheel and two casters - and no base_footprint at all.
        self.declare_parameter("frame_id", "base_link")

        self.frame_id = self.get_parameter("frame_id").value
        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value

        # Depth 1, reliable: velocity commands are worthless late. Queueing them
        # means the robot executes where it wanted to go a second ago, which on a
        # robot whose centre of mass sits 0.30 m over a 0.17 m half-track is how you
        # end up pushing against furniture.
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE)

        self.pub = self.create_publisher(TwistStamped, out_topic, qos)
        self.create_subscription(Twist, in_topic, self.relay, qos)

        self.count = 0
        self.create_timer(5.0, self.heartbeat)
        self.get_logger().info(f"Relais {in_topic} (Twist) -> {out_topic} (TwistStamped), "
                               f"frame {self.frame_id}.")

    def relay(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)
        self.count += 1

    def heartbeat(self):
        # Says out loud whether commands are flowing. The whole class of bug this
        # node addresses is invisible precisely because nothing reports the silence.
        if self.count == 0:
            self.get_logger().warn(
                "Aucune commande relayee depuis 5 s. Nav2 publie-t-il sur "
                f"{self.get_parameter('in_topic').value} ?")
        self.count = 0


def main():
    rclpy.init()
    node = CmdVelStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
