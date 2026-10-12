# Copyright (c) 2026 wee733
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Request a cuMotion joint-space plan through MoveIt without executing it."""

import argparse
import json
import math

from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan

import rclpy
from rclpy.utilities import remove_ros_args


def build_request(joints):
    """Build a six-joint cuMotion request using MoveIt's current robot state."""
    if len(joints) != 6 or not all(math.isfinite(value) for value in joints):
        raise ValueError('Expected six finite joint positions in radians')
    request = GetMotionPlan.Request()
    motion = request.motion_plan_request
    motion.group_name = 'manipulator'
    motion.pipeline_id = 'isaac_ros_cumotion'
    motion.start_state.is_diff = True
    motion.allowed_planning_time = 10.0
    motion.num_planning_attempts = 1
    motion.max_velocity_scaling_factor = 0.1
    motion.max_acceleration_scaling_factor = 0.1
    constraints = Constraints(name='joint_target')
    for index, position in enumerate(joints, start=1):
        constraints.joint_constraints.append(JointConstraint(
            joint_name=f'joint{index}',
            position=float(position),
            tolerance_above=0.001,
            tolerance_below=0.001,
            weight=1.0,
        ))
    motion.goal_constraints = [constraints]
    return request


def main(args=None):
    """Print the planner result and return nonzero on errors or timeouts."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joints', nargs=6, required=True, type=float,
                        metavar=('J1', 'J2', 'J3', 'J4', 'J5', 'J6'))
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='Service discovery and response timeout in seconds')
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    try:
        request = build_request(options.joints)
        if not math.isfinite(options.timeout) or options.timeout <= 0.0:
            raise ValueError('--timeout must be finite and positive')
    except ValueError as error:
        parser.error(str(error))

    rclpy.init(args=args)
    node = rclpy.create_node('arx_r5a_plan_joint_target')
    try:
        client = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        if not client.wait_for_service(timeout_sec=options.timeout):
            node.get_logger().error('MoveIt /plan_kinematic_path is unavailable')
            return 1
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=options.timeout)
        if not future.done():
            node.get_logger().error('Timed out waiting for the cuMotion plan')
            return 1
        response = future.result().motion_plan_response
        trajectory = response.trajectory.joint_trajectory
        success = (
            response.error_code.val == MoveItErrorCodes.SUCCESS
            and set(trajectory.joint_names) == {f'joint{i}' for i in range(1, 7)}
            and bool(trajectory.points)
        )
        print(json.dumps({
            'pipeline_id': request.motion_plan_request.pipeline_id,
            'success': success,
            'error_code': response.error_code.val,
            'planning_time': response.planning_time,
            'joint_names': list(trajectory.joint_names),
            'trajectory_points': len(trajectory.points),
            'executed': False,
        }, indent=2))
        return 0 if success else 1
    except (RuntimeError, KeyboardInterrupt) as error:
        node.get_logger().error(f'Planning request interrupted or failed: {error}')
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
