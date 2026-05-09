# pick_executor.py — Function Reference

**File:** `grocery_perception/pick_executor.py`  
**Purpose:** ROS 2 node that executes a robot pick sequence using a state machine and MoveIt 2 motion planning.

---

## Enum: `PickState`

Defines the states of the pick state machine:

| State | Description |
|---|---|
| `IDLE` | Waiting for a pick command |
| `MOVE_TO_PRE_GRASP` | Moving arm to the pre-grasp approach pose |
| `MOVE_TO_GRASP` | Moving arm to the final grasp pose |
| `CLOSE_GRIPPER` | Commanding the gripper to close |
| `MOVE_TO_LIFT` | Lifting the arm after grasping |
| `DONE` | Pick sequence completed successfully |
| `ABORTED` | Pick sequence failed and was aborted |

---

## Class: `PickExecutor(Node)`

Main ROS 2 node class. Implements the pick state machine.

---

### `__init__(self)`

Initialises the node. Declares and loads all ROS 2 parameters (MoveIt planning group, EEF link, velocity/acceleration scaling, constraint tolerances, gripper topic, etc.). Creates the MoveGroup action client, subscribes to pose and trigger topics, and advertises the gripper publisher.

**Subscriptions:**
- `/pre_grasp_pose` (PoseStamped)
- `/grasp_pose` (PoseStamped)
- `/lift_pose` (PoseStamped)
- `/start_pick` (Bool)

**Publishers:**
- `<gripper_close_topic>` (Bool) — default `/gripper/close`

---

### `pre_grasp_callback(self, msg: PoseStamped)`

Stores the latest pre-grasp pose received on `/pre_grasp_pose`.

---

### `grasp_callback(self, msg: PoseStamped)`

Stores the latest grasp pose received on `/grasp_pose`.

---

### `lift_callback(self, msg: PoseStamped)`

Stores the latest lift pose received on `/lift_pose`.

---

### `start_pick_callback(self, msg: Bool)`

Triggered by a `True` message on `/start_pick`. Guards against re-entrant calls (`busy` flag) and missing poses. If all preconditions pass, sets `busy=True` and starts the state machine by transitioning to `MOVE_TO_PRE_GRASP`.

---

### `transition_to(self, new_state: PickState)`

Central state machine dispatcher. Updates `self.state` and routes execution to the correct action for each state:

- `MOVE_TO_PRE_GRASP` → calls `send_moveit_goal` with pre-grasp pose
- `MOVE_TO_GRASP` → calls `send_moveit_goal` with grasp pose
- `CLOSE_GRIPPER` → calls `close_gripper()`
- `MOVE_TO_LIFT` → calls `send_moveit_goal` with lift pose
- `DONE` → logs success, clears `busy`
- `ABORTED` → logs failure, clears `busy`

---

### `close_gripper(self)`

Commands the gripper to close by publishing `True` on the gripper topic. If `plan_only=True`, skips the actual publish and schedules a 1-second timer to advance to `MOVE_TO_LIFT`. Otherwise publishes the close command and starts a wait timer (`gripper_close_wait_s`) before advancing.

---

### `_finish_gripper_close_once(self)`

Timer callback fired after `gripper_close_wait_s` seconds. Guards against repeated firings by checking the current state. Assumes the gripper has closed and transitions to `MOVE_TO_LIFT`.

---

### `send_moveit_goal(self, pose: PoseStamped, next_state_on_success: PickState)`

Validates the target pose (not None, non-empty frame_id) and waits up to 5 seconds for the MoveGroup action server. Builds the goal via `make_move_group_goal()` and sends it asynchronously. Registers `goal_response_callback` to handle acceptance/rejection.

---

### `make_move_group_goal(self, target_pose: PoseStamped) -> MoveGroup.Goal`

Constructs a `MoveGroup.Goal` message with:
- **Position constraint:** a tolerance sphere around the target position.
- **Orientation constraint:** fixed orientation with configurable axis tolerances.
- **PlanningOptions:** respects `plan_only`, disables look-around and replanning.

Returns the fully configured goal message.

---

### `goal_response_callback(self, future)`

Called when MoveIt responds to the goal submission. If the goal was rejected, transitions to `ABORTED`. If accepted, registers `result_callback` to handle the execution result.

---

### `result_callback(self, future)`

Called when MoveIt finishes executing (or planning, if `plan_only=True`). Checks the `error_code`:
- `1` (SUCCESS) → schedules a 1-second timer to call `delayed_transition_once` with the next state (avoids transitioning inside the action callback).
- Any other code → transitions to `ABORTED`.

---

### `all_poses_ready(self) -> bool`

Returns `True` only if all three poses (pre-grasp, grasp, lift) have been received and are not `None`.

---

### `report_pose_status(self)`

Logs the readiness of each individual pose to help diagnose why a pick could not start.

---

### `delayed_transition_once(self, next_state)`

One-shot helper used by timers to safely fire a state transition. Cancels and destroys the `next_state_timer` before calling `transition_to`, preventing duplicate transitions if the timer fires more than once.

---

## Function: `main(args=None)`

Entry point. Initialises `rclpy`, creates a `PickExecutor` node, spins until `KeyboardInterrupt`, then cleans up and shuts down.
