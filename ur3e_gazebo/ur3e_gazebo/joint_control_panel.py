"""
joint_control_panel.py
======================
Interactive GUI for manually positioning the UR3e arm in Gazebo.

Use this during Phase 8.3 to dial in the joint angles for each pick-and-place
pose, then copy the values directly into pick_and_place.py.

USAGE
-----
  Launch the simulation first, click Play, wait for home pose, then:

    ros2 run ur3e_gazebo joint_control_panel

FEATURES
--------
  - Sliders for all 6 arm joints and the gripper
  - SEND COMMAND button (or Live Update checkbox for continuous control)
  - Read /joint_states button — snaps sliders to the arm's current position
  - Preset buttons — jump to any pick_and_place.py pose instantly
  - Output box — shows the current values formatted to paste into pick_and_place.py
  - Copy to Clipboard button
"""

import re
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration


# ═══════════════════════════════════════════════════════════════════════════════
#  JOINT DEFINITIONS
# ═══════════════════════════════════════════════════════════════════════════════

# (joint_name, display_label, slider_min, slider_max, home_value)
ARM_JOINTS = [
    ('shoulder_pan_joint',  'shoulder pan',  -6.28, 6.28,  0.0),
    ('shoulder_lift_joint', 'shoulder lift', -6.28, 6.28, -1.57),
    ('elbow_joint',         'elbow',         -3.14, 3.14,  1.57),
    ('wrist_1_joint',       'wrist 1',       -6.28, 6.28, -1.57),
    ('wrist_2_joint',       'wrist 2',       -6.28, 6.28,  0.0),
    ('wrist_3_joint',       'wrist 3',       -6.28, 6.28,  0.0),
]

FINGER_MIN  = -0.03
FINGER_MAX  = 0.05
FINGER_HOME = 0.0


# ═══════════════════════════════════════════════════════════════════════════════
#  PRESET POSES  (match pick_and_place.py — update both if you tune one)
# ═══════════════════════════════════════════════════════════════════════════════

PRESETS = [
    ('Home', {
        'shoulder_pan_joint':  0.0,
        'shoulder_lift_joint': -1.57,
        'elbow_joint':          1.57,
        'wrist_1_joint':       -1.57,
        'wrist_2_joint':        0.0,
        'wrist_3_joint':        0.0,
    }, 0.0),
    ('Approach', {
        'shoulder_pan_joint':  0.0,
        'shoulder_lift_joint': -1.40,
        'elbow_joint':          1.60,
        'wrist_1_joint':       -1.70,
        'wrist_2_joint':        0.0,
        'wrist_3_joint':        0.0,
    }, 0.05),
    ('Grasp', {
        'shoulder_pan_joint':  0.0,
        'shoulder_lift_joint': -1.10,
        'elbow_joint':          1.30,
        'wrist_1_joint':       -1.75,
        'wrist_2_joint':        0.0,
        'wrist_3_joint':        0.0,
    }, 0.05),
    ('Lift', {
        'shoulder_pan_joint':  0.0,
        'shoulder_lift_joint': -1.45,
        'elbow_joint':          1.55,
        'wrist_1_joint':       -1.57,
        'wrist_2_joint':        0.0,
        'wrist_3_joint':        0.0,
    }, 0.0),
    ('Transport', {
        'shoulder_pan_joint':  3.14,
        'shoulder_lift_joint': -1.50,
        'elbow_joint':          1.30,
        'wrist_1_joint':       -1.57,
        'wrist_2_joint':        0.0,
        'wrist_3_joint':        0.0,
    }, 0.0),
    ('Place', {
        'shoulder_pan_joint':  3.14,
        'shoulder_lift_joint': -1.10,
        'elbow_joint':          1.20,
        'wrist_1_joint':       -1.57,
        'wrist_2_joint':        0.0,
        'wrist_3_joint':        0.0,
    }, 0.0),
]


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS 2 NODE  (thin — just pub/sub)
# ═══════════════════════════════════════════════════════════════════════════════

class ControlPanelNode(Node):

    def __init__(self):
        super().__init__('joint_control_panel')

        self._pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )

        self._latest_joint_state: JointState | None = None
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

    def _js_cb(self, msg: JointState):
        self._latest_joint_state = msg

    def send(self, arm_joints: dict, finger_pos: float, duration_sec: float):
        joint_names = list(arm_joints.keys()) + ['left_finger_joint', 'right_finger_joint']
        positions   = list(arm_joints.values()) + [finger_pos, finger_pos]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9),
        )

        msg = JointTrajectory()
        msg.joint_names = joint_names
        msg.points = [point]
        self._pub.publish(msg)

    def latest_joint_state(self):
        return self._latest_joint_state


# ═══════════════════════════════════════════════════════════════════════════════
#  GUI
# ═══════════════════════════════════════════════════════════════════════════════

class JointControlPanel:

    # Minimum seconds between live-update sends (debounce)
    _DEBOUNCE = 0.12

    def __init__(self, node: ControlPanelNode):
        self._node = node
        self._last_send = 0.0

        self._root = tk.Tk()
        self._root.title('UR3e Joint Control Panel')
        self._root.resizable(True, False)
        self._root.minsize(540, 1)

        # One DoubleVar per arm joint (keyed by joint name)
        self._arm_vars: dict[str, tk.DoubleVar] = {}
        # Corresponding formatted StringVar for the value label
        self._arm_display: dict[str, tk.StringVar] = {}

        self._finger_var    = tk.DoubleVar(value=FINGER_HOME)
        self._finger_display = tk.StringVar(value=f'{FINGER_HOME:.4f}')

        self._duration_var  = tk.DoubleVar(value=2.0)
        self._live_var      = tk.BooleanVar(value=False)

        self._build_ui()

    # ------------------------------------------------------------------ #
    #  UI construction                                                     #
    # ------------------------------------------------------------------ #

    def _build_ui(self):
        r = self._root
        PAD = 8

        # ── Header ─────────────────────────────────────────────────────
        hdr = tk.Frame(r, bg='#1a237e', pady=5)
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text='UR3e  Joint Control Panel',
                 bg='#1a237e', fg='white',
                 font=('Helvetica', 13, 'bold')).pack()
        tk.Label(hdr, text='Move sliders → SEND COMMAND   |   or enable Live Update',
                 bg='#1a237e', fg='#90caf9',
                 font=('Helvetica', 8)).pack()

        # ── Arm joint sliders ──────────────────────────────────────────
        arm_frame = tk.LabelFrame(r, text=' ARM JOINTS ', padx=PAD, pady=4)
        arm_frame.pack(fill=tk.X, expand=True, padx=PAD, pady=(PAD, 0))
        arm_frame.columnconfigure(1, weight=1)

        for row, (jname, label, lo, hi, home) in enumerate(ARM_JOINTS):
            dvar = tk.DoubleVar(value=home)
            svar = tk.StringVar(value=f'{home:+.3f}')
            self._arm_vars[jname]    = dvar
            self._arm_display[jname] = svar

            # Keep formatted display in sync
            dvar.trace_add('write', lambda *_, d=dvar, s=svar: s.set(f'{d.get():+.3f}'))

            tk.Label(arm_frame, text=label, width=13, anchor='e',
                     font=('Helvetica', 9)).grid(row=row, column=0, sticky='e', padx=(0, 4))

            tk.Scale(arm_frame, variable=dvar, from_=lo, to=hi,
                     orient=tk.HORIZONTAL, resolution=0.001,
                     width=30, showvalue=False,
                     command=lambda _v, jn=jname: self._slider_moved(jn),
                     ).grid(row=row, column=1, sticky='ew')

            tk.Label(arm_frame, textvariable=svar, width=7, anchor='w',
                     font=('Courier', 9), fg='#333').grid(row=row, column=2, padx=(4, 0))

            tk.Label(arm_frame, text='rad', font=('Helvetica', 8),
                     fg='grey').grid(row=row, column=3, sticky='w')

        # ── Gripper slider ─────────────────────────────────────────────
        grip_frame = tk.LabelFrame(r, text=' GRIPPER ', padx=PAD, pady=4)
        grip_frame.pack(fill=tk.X, expand=True, padx=PAD, pady=(4, 0))
        grip_frame.columnconfigure(1, weight=1)

        self._finger_var.trace_add(
            'write',
            lambda *_: self._finger_display.set(f'{self._finger_var.get():.4f}'),
        )

        tk.Label(grip_frame, text='fingers', width=13, anchor='e',
                 font=('Helvetica', 9)).grid(row=0, column=0, sticky='e', padx=(0, 4))
        tk.Scale(grip_frame, variable=self._finger_var,
                 from_=FINGER_MIN, to=FINGER_MAX,
                 orient=tk.HORIZONTAL, resolution=0.001,
                 width=30, showvalue=False,
                 command=lambda _v: self._slider_moved('fingers'),
                 ).grid(row=0, column=1, sticky='ew')
        tk.Label(grip_frame, textvariable=self._finger_display, width=7, anchor='w',
                 font=('Courier', 9), fg='#333').grid(row=0, column=2, padx=(4, 0))
        tk.Label(grip_frame, text='m', font=('Helvetica', 8),
                 fg='grey').grid(row=0, column=3, sticky='w')

        # Annotation below the finger slider
        ann = tk.Frame(grip_frame)
        ann.grid(row=1, column=1, sticky='ew')
        tk.Label(ann, text='◀ closed (0.00)', font=('Helvetica', 7), fg='grey').pack(side=tk.LEFT)
        tk.Label(ann, text='open (0.05) ▶',  font=('Helvetica', 7), fg='grey').pack(side=tk.RIGHT)

        # ── Controls row ───────────────────────────────────────────────
        ctrl = tk.Frame(r, padx=PAD, pady=6)
        ctrl.pack(fill=tk.X)

        tk.Label(ctrl, text='Duration:', font=('Helvetica', 9)).pack(side=tk.LEFT)
        tk.Spinbox(ctrl, textvariable=self._duration_var,
                   from_=0.5, to=10.0, increment=0.5,
                   width=4, format='%.1f',
                   font=('Helvetica', 9)).pack(side=tk.LEFT, padx=(2, 2))
        tk.Label(ctrl, text='s', font=('Helvetica', 9), fg='grey').pack(side=tk.LEFT, padx=(0, 10))

        tk.Checkbutton(ctrl, text='Live Update', variable=self._live_var,
                       font=('Helvetica', 9)).pack(side=tk.LEFT, padx=(0, 10))

        tk.Button(ctrl, text='Read /joint_states',
                  font=('Helvetica', 9),
                  command=self._read_joint_states).pack(side=tk.LEFT, padx=(0, 6))

        tk.Button(ctrl, text='  SEND COMMAND  ',
                  bg='#2e7d32', fg='white',
                  activebackground='#1b5e20', activeforeground='white',
                  font=('Helvetica', 10, 'bold'),
                  command=self._send_command).pack(side=tk.LEFT)

        # ── Presets ────────────────────────────────────────────────────
        preset_frame = tk.LabelFrame(r, text=' PRESETS ', padx=PAD, pady=4)
        preset_frame.pack(fill=tk.X, padx=PAD, pady=(0, 0))

        for label, joints, finger in PRESETS:
            tk.Button(preset_frame, text=label, width=9,
                      font=('Helvetica', 9),
                      command=lambda j=joints, f=finger: self._load_preset(j, f),
                      ).pack(side=tk.LEFT, padx=3, pady=2)

        # ── Paste & Load ───────────────────────────────────────────────
        paste_frame = tk.LabelFrame(r, text=' Paste & Load Position ', padx=PAD, pady=4)
        paste_frame.pack(fill=tk.X, padx=PAD, pady=(4, 0))

        tk.Label(paste_frame,
                 text='Paste copied output here, then click Load:',
                 font=('Helvetica', 8), fg='grey').pack(anchor='w')

        self._paste_text = tk.Text(paste_frame, height=5, width=52,
                                   font=('Courier', 9),
                                   bg='#fffde7', relief=tk.FLAT, bd=1)
        self._paste_text.pack(fill=tk.X, pady=(2, 4))

        btn_row = tk.Frame(paste_frame)
        btn_row.pack()
        tk.Button(btn_row, text='Paste from Clipboard & Load',
                  font=('Helvetica', 9),
                  command=self._paste_from_clipboard).pack(side=tk.LEFT, padx=(0, 8))
        tk.Button(btn_row, text='Load',
                  font=('Helvetica', 9, 'bold'),
                  command=self._load_from_paste_box).pack(side=tk.LEFT)

        self._paste_status = tk.StringVar(value='')
        tk.Label(paste_frame, textvariable=self._paste_status,
                 font=('Helvetica', 8), fg='#c62828').pack()

        # ── Output box ─────────────────────────────────────────────────
        out_frame = tk.LabelFrame(r, text=' Copy into pick_and_place.py ', padx=PAD, pady=4)
        out_frame.pack(fill=tk.X, padx=PAD, pady=(4, PAD))

        self._out_text = tk.Text(out_frame, height=9, width=52,
                                 font=('Courier', 9), state=tk.DISABLED,
                                 bg='#f5f5f5', relief=tk.FLAT, bd=1)
        self._out_text.pack(fill=tk.X)

        tk.Button(out_frame, text='Copy to Clipboard',
                  font=('Helvetica', 9),
                  command=self._copy_to_clipboard).pack(pady=(4, 0))

        self._refresh_output()

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #

    def _slider_moved(self, _joint_name: str):
        self._refresh_output()
        if self._live_var.get():
            now = time.monotonic()
            if now - self._last_send >= self._DEBOUNCE:
                self._send_command()

    def _send_command(self):
        arm = {jname: var.get() for jname, var in self._arm_vars.items()}
        self._node.send(arm, self._finger_var.get(), self._duration_var.get())
        self._last_send = time.monotonic()

    def _read_joint_states(self):
        msg = self._node.latest_joint_state()
        if msg is None:
            # Flash the button label briefly to indicate nothing received yet
            return

        name_to_pos = dict(zip(msg.name, msg.position))

        for jname, var in self._arm_vars.items():
            if jname in name_to_pos:
                var.set(round(name_to_pos[jname], 3))

        left  = name_to_pos.get('left_finger_joint',  0.0)
        right = name_to_pos.get('right_finger_joint', 0.0)
        self._finger_var.set(round((left + right) / 2.0, 4))

        self._refresh_output()

    def _load_preset(self, joints: dict, finger: float):
        for jname, var in self._arm_vars.items():
            if jname in joints:
                var.set(joints[jname])
        self._finger_var.set(finger)
        self._refresh_output()

    def _paste_from_clipboard(self):
        try:
            text = self._root.clipboard_get()
        except tk.TclError:
            self._paste_status.set('Clipboard is empty.')
            return
        self._paste_text.delete('1.0', tk.END)
        self._paste_text.insert(tk.END, text)
        self._load_from_paste_box()

    def _load_from_paste_box(self):
        text = self._paste_text.get('1.0', tk.END)
        joints, finger = self._parse_pose_text(text)

        if not joints and finger is None:
            self._paste_status.set('Nothing recognised — paste the output block from the copy box.')
            return

        loaded = []
        for jname, var in self._arm_vars.items():
            if jname in joints:
                var.set(round(joints[jname], 3))
                loaded.append(jname)

        if finger is not None:
            self._finger_var.set(round(finger, 4))
            loaded.append('fingers')

        self._paste_status.set(f'Loaded: {", ".join(loaded)}' if loaded else 'No matching joints found.')
        self._refresh_output()

    @staticmethod
    def _parse_pose_text(text: str) -> tuple[dict, float | None]:
        """
        Parse the copy-box format back into joint values.

        Handles lines like:
            'shoulder_pan_joint':  +0.000,
            # left_finger_joint:   0.0500,
        """
        joints: dict[str, float] = {}
        finger: float | None = None

        for line in text.splitlines():
            # Named arm joint:  'joint_name':  value
            m = re.search(r"'(\w+)':\s*([+-]?\d+\.?\d*)", line)
            if m:
                joints[m.group(1)] = float(m.group(2))
                continue

            # Finger joint comment:  # left_finger_joint:   value
            m2 = re.search(r'#\s*left_finger_joint:\s*([+-]?\d+\.?\d*)', line)
            if m2:
                finger = float(m2.group(1))

        return joints, finger

    def _refresh_output(self):
        lines = ['# Paste into pick_and_place.py:']
        for jname, var in self._arm_vars.items():
            lines.append(f"    '{jname}':  {var.get():+.3f},")
        f = self._finger_var.get()
        lines.append(f"    # left_finger_joint:   {f:.4f},  (0=closed, 0.05=open)")
        lines.append(f"    # right_finger_joint:  {f:.4f},")

        self._out_text.configure(state=tk.NORMAL)
        self._out_text.delete('1.0', tk.END)
        self._out_text.insert(tk.END, '\n'.join(lines))
        self._out_text.configure(state=tk.DISABLED)

    def _copy_to_clipboard(self):
        self._root.clipboard_clear()
        self._root.clipboard_append(self._out_text.get('1.0', tk.END).strip())

    # ------------------------------------------------------------------ #
    #  Main loop (integrates rclpy via periodic after() call)             #
    # ------------------------------------------------------------------ #

    def _ros_tick(self):
        """Called every 50 ms by tkinter — drives rclpy without a separate thread."""
        rclpy.spin_once(self._node, timeout_sec=0)
        self._root.after(50, self._ros_tick)

    def run(self):
        self._root.after(50, self._ros_tick)
        self._root.mainloop()


# ═══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = ControlPanelNode()
    panel = JointControlPanel(node)
    try:
        panel.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
