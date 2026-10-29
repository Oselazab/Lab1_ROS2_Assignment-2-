"""Actions and services share a reentrant group and an eight-thread executor.
Blocking orchestration waits leave threads available for ROS client responses.
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, Trigger
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute
from lab1_interfaces.action import GoToPose, DrawSquare
from lab1_interfaces.msg import TargetPose
from lab1_interfaces.srv import SetSpeedLimits
from .controller import HOME, command, valid_target


class LabServer(Node):
    def __init__(self):
        super().__init__('lab1_server')
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.pose = None
        self.pose_time = 0.0
        self.motion_busy = False
        self.reset_busy = False
        self.square_busy = False
        self.square_reset_pending = False
        self.expected_corner = None
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.add_on_set_parameters_callback(self.validate_parameters)
        for name in ('max_linear_speed', 'max_angular_speed'):
            value = self.get_parameter(name).value
            if not isinstance(value, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be a positive finite double')
        self.velocity = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10, callback_group=self.group)
        self.clear = self.create_client(Empty, '/clear', callback_group=self.group)
        self.pen = self.create_client(SetPen, '/turtle1/set_pen', callback_group=self.group)
        self.teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute', callback_group=self.group)
        self.create_service(Trigger, '/reset_turtle', self.reset_service, callback_group=self.group)
        self.create_service(SetSpeedLimits, '/set_speed_limits', self.set_speeds, callback_group=self.group)
        self.reset_client = self.create_client(Trigger, '/reset_turtle', callback_group=self.group)
        self.goto_client = ActionClient(self, GoToPose, '/go_to_pose', callback_group=self.group)
        self.goto_server = ActionServer(self, GoToPose, '/go_to_pose', self.execute_pose,
            goal_callback=self.accept_pose, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.group)
        self.square_server = ActionServer(self, DrawSquare, '/draw_square', self.execute_square,
            goal_callback=self.accept_square, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.group)

    def on_pose(self, msg):
        self.pose = (msg.x, msg.y, msg.theta)
        self.pose_time = time.monotonic()

    @staticmethod
    def validate_parameters(parameters):
        for p in parameters:
            if p.name in ('max_linear_speed', 'max_angular_speed'):
                if p.type_ != Parameter.Type.DOUBLE or not math.isfinite(p.value) or p.value <= 0:
                    return SetParametersResult(successful=False, reason='Speed limits must be positive finite doubles')
        return SetParametersResult(successful=True)

    def set_speeds(self, request, response):
        result = self.set_parameters_atomically([
            Parameter('max_linear_speed', value=request.max_linear_speed),
            Parameter('max_angular_speed', value=request.max_angular_speed)])
        response.success = result.successful
        response.message = 'Both speed limits updated' if result.successful else result.reason
        return response

    def stop(self):
        self.velocity.publish(Twist())

    def accept_pose(self, request):
        t = request.target
        values = (t.x, t.y, t.yaw)
        with self.lock:
            if (not valid_target(*values) or self.motion_busy or self.reset_busy
                    or (self.square_busy and values != self.expected_corner)):
                return GoalResponse.REJECT
            self.expected_corner = None
            self.motion_busy = True
        return GoalResponse.ACCEPT

    def execute_pose(self, handle):
        result = GoToPose.Result()
        t = handle.request.target
        self.get_logger().info(f'goToPose accepted: ({t.x:.3f}, {t.y:.3f}, {t.yaw:.3f})')
        started = time.monotonic()
        try:
            while rclpy.ok():
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.message = 'Motion canceled'
                    return result
                now = time.monotonic()
                if now-started > 180:
                    raise RuntimeError('Goal timed out after 180 seconds')
                if self.pose is None:
                    if now-started > 5:
                        raise RuntimeError('No turtlesim pose received')
                    time.sleep(0.02)
                    continue
                if now-self.pose_time > 2:
                    raise RuntimeError('Turtlesim pose is stale')
                v, w, distance, yaw_error, done = command(self.pose, (t.x,t.y,t.yaw),
                    self.get_parameter('max_linear_speed').value,
                    self.get_parameter('max_angular_speed').value)
                handle.publish_feedback(GoToPose.Feedback(distance_remaining=distance, yaw_error=yaw_error))
                if done:
                    handle.succeed()
                    result.success = True
                    result.message = 'Target position and yaw reached'
                    return result
                msg = Twist()
                msg.linear.x, msg.angular.z = v, w
                self.velocity.publish(msg)
                time.sleep(0.02)
            raise RuntimeError('ROS shutdown')
        except Exception as exc:
            if handle.is_active:
                handle.abort()
            result.message = str(exc)
            return result
        finally:
            self.stop()
            with self.lock:
                self.motion_busy = False

    @staticmethod
    def wait(future, timeout=10.0):
        deadline = time.monotonic()+timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError('ROS response timed out')
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError('ROS shutdown while waiting')
        return future.result()

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        return self.wait(client.call_async(request))

    def reset_service(self, request, response):
        with self.lock:
            if self.motion_busy or self.reset_busy or (self.square_busy and not self.square_reset_pending):
                response.message = 'Busy: cancel the active action before resetting'
                return response
            self.square_reset_pending = False
            self.reset_busy = True
        pen_off = False
        try:
            self.stop()
            self.call(self.pen, SetPen.Request(r=255,g=255,b=255,width=3,off=1))
            pen_off = True
            self.call(self.teleport, TeleportAbsolute.Request(x=HOME,y=HOME,theta=0.0))
            self.call(self.clear, Empty.Request())
            response.success = True
            response.message = 'Screen cleared; turtle restored to centre, yaw zero'
        except Exception as exc:
            response.message = str(exc)
        finally:
            if pen_off:
                try:
                    self.call(self.pen, SetPen.Request(r=255,g=255,b=255,width=3,off=0))
                except Exception as exc:
                    response.success = False
                    response.message = f'Could not restore pen: {exc}'
            with self.lock:
                self.reset_busy = False
        return response

    def accept_square(self, request):
        side = request.side_length
        with self.lock:
            if (not math.isfinite(side) or side < 0.2 or HOME+side > 10.8
                    or self.square_busy or self.motion_busy or self.reset_busy):
                return GoalResponse.REJECT
            self.square_busy = True
            self.square_reset_pending = True
        return GoalResponse.ACCEPT

    def execute_square(self, handle):
        result = DrawSquare.Result()
        child = None
        try:
            reset = self.call(self.reset_client, Trigger.Request())
            if not reset.success:
                raise RuntimeError(reset.message)
            if not self.goto_client.wait_for_server(timeout_sec=5):
                raise RuntimeError('goToPose server unavailable')
            s = handle.request.side_length
            corners = [(HOME+s,HOME,math.pi/2), (HOME+s,HOME+s,math.pi),
                       (HOME,HOME+s,-math.pi/2), (HOME,HOME,0.0)]
            for index, corner in enumerate(corners, 1):
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.message = 'Square canceled'
                    return result
                handle.publish_feedback(DrawSquare.Feedback(current_side=index, completed_sides=index-1))
                with self.lock:
                    self.expected_corner = corner
                goal = GoToPose.Goal(target=TargetPose(x=corner[0],y=corner[1],yaw=corner[2]))
                child = self.wait(self.goto_client.send_goal_async(goal))
                if not child.accepted:
                    raise RuntimeError(f'Side {index} rejected')
                pending = child.get_result_async()
                while not pending.done() and rclpy.ok():
                    if handle.is_cancel_requested:
                        self.wait(child.cancel_goal_async())
                        self.wait(pending, 10)
                        handle.canceled()
                        result.message = 'Square and active side canceled'
                        return result
                    time.sleep(0.02)
                outcome = self.wait(pending)
                if not outcome.result.success:
                    raise RuntimeError(f'Side {index}: {outcome.result.message}')
            handle.publish_feedback(DrawSquare.Feedback(current_side=4, completed_sides=4))
            handle.succeed()
            result.success = True
            result.message = 'Square complete: exactly four goToPose goals succeeded'
        except Exception as exc:
            if child is not None and child.accepted:
                try:
                    self.wait(child.cancel_goal_async())
                except Exception:
                    pass
            if handle.is_active:
                handle.abort()
            result.message = str(exc)
        finally:
            with self.lock:
                self.expected_corner = None
                self.square_busy = False
                self.square_reset_pending = False
                self.reset_busy = False
        return result


def main(args=None):
    rclpy.init(args=args)
    node = LabServer()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
