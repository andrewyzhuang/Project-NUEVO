# Manual Test Instructions

Tests for Phases 1–8. Run in order — later phases depend on earlier ones.

---

## Prerequisites

Start the VM Docker container:
```bash
docker compose -f ros2_ws/docker/docker-compose.vm.yml up -d --build --wait
```

Wait for healthy (up to 90 s on first run):
```bash
docker compose -f ros2_ws/docker/docker-compose.vm.yml ps
```

Open a shell in the container:
```bash
docker compose -f ros2_ws/docker/docker-compose.vm.yml exec ros2_runtime bash
source /ros2_ws/install/setup.bash
```

---

## Phase 1 — ROS2 Message Types

```bash
# Inside container:
ros2 interface show bridge_interfaces/msg/FusedPose
ros2 interface show bridge_interfaces/msg/LidarWorldPoints
```

**Expected:**
```
std_msgs/Header header
float32 x
float32 y
float32 theta
bool gps_active
```
```
std_msgs/Header header
float32[] xs
float32[] ys
float32 robot_x
float32 robot_y
float32 robot_theta
```

---

## Phase 2 — lidar_scan.py

```bash
# On Mac (no ROS needed):
cd ros2_ws/src/robot
python3 - <<'EOF'
import sys, math, numpy as np
sys.path.insert(0, '.')
from robot.lidar_scan import LidarConfig, LidarScan

class FakeScan:
    def __init__(self, a, r):
        self.angle_min = a[0]; self.angle_max = a[-1]; self.ranges = r

cfg = LidarConfig(yaw_deg=0.0)
sc = LidarScan(cfg)
pts = sc.process(FakeScan([0.0, 0.1], [1.0, 13.0]))
assert pts.shape == (1, 2) and abs(pts[0,0] - 1000) < 0.5, pts
print("PASS: forward ray 1m →", pts[0])

cfg2 = LidarConfig(yaw_deg=180.0)
pts2 = LidarScan(cfg2).process(FakeScan([0.0, 0.1], [1.0, 13.0]))
assert abs(pts2[0,0] + 1000) < 1.0, pts2
print("PASS: 180° mount →", pts2[0])

pts3 = sc.to_world_frame(np.array([[500.,0.]]), (100., 0., 0.))
assert abs(pts3[0,0] - 600) < 0.1, pts3
print("PASS: world shift →", pts3[0])
EOF
```

**Expected:** three PASS lines.

---

## Phase 3 — APF navigate_to_goal

```bash
# On Mac (no ROS needed):
cd ros2_ws/src/robot
python3 - <<'EOF'
import sys, numpy as np
sys.path.insert(0, '.')
from robot.path_planner import APFPlanner

p = APFPlanner(attraction_gain=1.0, repulsion_gain=500.0, repulsion_range=300.0,
               max_linear=200.0, max_angular=2.0, heading_gain=2.0, goal_tolerance=20.0)
empty = np.empty((0, 2))

lin, ang = p.navigate_to_goal((0,0,0), (500,0), empty)
assert lin > 0 and abs(ang) < 0.05, (lin, ang)
print(f"PASS: straight ahead lin={lin:.1f}")

_, ang2 = p.navigate_to_goal((0,0,0), (0,500), empty)
assert ang2 > 0, ang2
print(f"PASS: left turn ang={ang2:.3f}")

lin4, ang4 = p.navigate_to_goal((495,0,0), (500,0), empty)
assert lin4 == 0.0 == ang4, (lin4, ang4)
print("PASS: at goal → (0,0)")
EOF
```

---

## Phase 4 — /fused_pose topic (needs robot + bridge running on RPi)

```bash
# Inside container with robot running:
ros2 topic echo /fused_pose --once
```

**Expected:** x, y, theta values updating; `gps_active: False` normally, `True` when GPS tag is visible.

For VM-only verification (no RPi):
```bash
# Publish a fake fused_pose and verify bridge forwards it:
ros2 topic pub --once /fused_pose bridge_interfaces/msg/FusedPose \
  "{x: 100.0, y: 200.0, theta: 0.5, gps_active: false}"
# Then check browser console for: fused_pose {x:100, y:200, theta:0.5, gps_active:false}
```

---

## Phase 5 — new_example.py (needs RPi with Arduino + RPLidar)

```bash
# On RPi, inside container:
ros2 run robot new_example
```

**Expected:**
1. Robot starts moving toward GOAL_MM (default 1000, 0)
2. Lidar points appear in web UI world canvas
3. Robot deflects if obstacle placed in path
4. Logs "Reached goal" when within 60 mm of goal
5. `ros2 topic hz /lidar_world_points` shows ~10 Hz

**VM test (mock lidar, no movement):**
```bash
# In container — check that /lidar_world_points is being published:
ros2 topic echo /lidar_world_points --once
```

---

## Phase 6 — Bridge WebSocket relay

Open browser dev console while connected to bridge:

```js
// Paste in browser console to monitor new topics:
const ws = new WebSocket('ws://localhost:8000/ws?token=<your_token>');
ws.onmessage = (e) => {
  const m = JSON.parse(e.data);
  if (['fused_pose','gps_status','lidar_world_points','ros_nodes'].includes(m.topic))
    console.log(m.topic, m.data);
};
```

**Expected within 5 s:**
- `ros_nodes` appears with list of running nodes (~1.5 Hz)
- `fused_pose` appears when robot node is running
- `gps_status.is_detected` → `false` (no camera in VM)
- `lidar_world_points` appears when new_example.py is running

---

## Phase 7 — Zustand store

In browser dev console:
```js
// Import store (works with React devtools or raw access):
const s = window.__ROBOT_STORE__ || null;  // if exposed
// Or use React DevTools → Components → useRobotStore

// Verify state keys exist:
console.log(Object.keys(useRobotStore.getState()));
// Should include: fusedPose, fusedPoseTrail, odometryTrail, gpsStatus, lidarPoints, rosNodes
```

---

## Phase 8 — Frontend UI

Navigate to `http://localhost:8000` and scroll to the **Raspberry Pi** section.

### 8a — World Canvas
- [ ] Canvas appears below Bridge Status
- [ ] Four toggle buttons: Odometry, GPS, Fused, Lidar
- [ ] Grid lines visible with scale label
- [ ] Toggle Odometry off → cyan trail disappears
- [ ] Toggle back on → trail reappears
- [ ] When robot moves (or mock kinematics tick), cyan odometry trail accumulates
- [ ] When `new_example.py` running, red lidar dots visible around robot

### 8b — GPS Status Card
- [ ] Shows "NO" in red when no GPS (normal in VM)
- [ ] On RPi with camera: shows "YES" in green when ArUco tag in view
- [ ] X, Y values grayed out when not detected
- [ ] After 1 s of tag disappearing, reverts to NO

### 8c — ROS Nodes Card
- [ ] "ROS Nodes" header with count badge
- [ ] Click to expand — shows list of nodes
- [ ] Click individual node → shows publishes / subscribes lists
- [ ] Refreshes automatically (~1.5 Hz)
- [ ] Shows "No data yet" before first bridge message

---

## Quick Smoke Test (VM only, no hardware)

```bash
# 1. Start container
docker compose -f ros2_ws/docker/docker-compose.vm.yml up -d --build --wait

# 2. Verify all packages built
docker compose -f ros2_ws/docker/docker-compose.vm.yml exec ros2_runtime bash -c \
  "source /ros2_ws/install/setup.bash && ros2 pkg prefix robot sensors bridge bridge_interfaces"

# 3. Verify mock lidar running
docker compose -f ros2_ws/docker/docker-compose.vm.yml exec ros2_runtime bash -c \
  "source /ros2_ws/install/setup.bash && ros2 topic hz /scan --timeout 5"

# 4. Verify new message types
docker compose -f ros2_ws/docker/docker-compose.vm.yml exec ros2_runtime bash -c \
  "source /ros2_ws/install/setup.bash && ros2 interface show bridge_interfaces/msg/FusedPose"

# 5. Open http://localhost:8000 and verify UI loads with new RPi section panels
```
