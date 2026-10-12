# Contributing

Keep changes focused on ARX R5A integration with Isaac ROS cuMotion. Preserve
public package names, launch arguments, topics, joint ordering, and controller
actions unless an interface change is deliberate and documented. Perception and
application workflows belong in downstream packages.

## Code and documentation

- Use English for identifiers, comments, docstrings, and the primary README.
  Update `README.zh-CN.md` when commands or supported behavior change.
- Follow ROS 2 ament conventions: four-space Python indentation, two-space C++
  indentation, and C++17. Use `ament_flake8`, `ament_pep257`, and the package's
  C++ ament lint checks. Comments should explain units, constraints, or reasoning.
- Declare runtime and build dependencies in `package.xml`; keep Python package
  metadata consistent with `setup.py`. Do not hard-code workstation paths.
- Keep build outputs, caches, temporary logs, and downloaded SDKs outside tracked
  source. Preserve upstream copyright and license notices.
- Keep MoveIt and cuMotion URDFs consistent. Update joint limits, SRDF, XRDF, and
  controller configuration together when their shared contract changes.

## Validation

Use the target Jazzy / Isaac ROS 4.5 environment and a separate colcon workspace.
After a coherent change, run the checks relevant to the changed behavior:

```bash
colcon build --symlink-install \
  --packages-up-to isaac_ros_manipulation_arx_r5a_driver_utils
source install/setup.bash
colcon test --packages-select \
  isaac_ros_manipulation_arx_r5a_robot_description \
  isaac_ros_manipulation_arx_r5a_ros2_control \
  isaac_ros_manipulation_arx_r5a_driver_utils \
  --event-handlers console_direct+
colcon test-result --verbose
```

For planning changes, also run the README's explicit cuMotion planning request
on a correctly configured runtime. Report build/lint/unit-test results,
planning results, and physical execution separately. Mock services establish
client behavior only. Do not describe a skipped GPU or hardware check as a pass.
Physical tests require an operator, ARX startup procedures, conservative limits,
and an available independent stop mechanism.

## Issues and pull requests

Include the problem, expected and observed behavior, reproduction command,
repository revision, OS/ROS/Isaac ROS versions, GPU/CPU architecture, relevant
launch overrides, and concise logs. State whether the vendor driver, MoveIt,
cuMotion planner, and hardware were running. Remove credentials and private
connection details from logs.

Explain what changed and why, the validation performed, and any remaining
limitations. Avoid unrelated reformatting or generated-file churn.
