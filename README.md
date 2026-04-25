# Lab 4 — Abhishek Tyagi (`tyagi55`)

**Student:** Abhishek Tyagi  
**Purdue Email:** tyagi55@purdue.edu  
**GitHub:** [github.com/AbhishekTyagi404/ECE_Lab4](https://github.com/AbhishekTyagi404/ECE_Lab4)  
**Overleaf Report:** [Link](https://www.overleaf.com/read/ywtcvwdckmjh#7fa008](https://www.overleaf.com/project/69ec3b68d816238aa2abca2c)

---

## Submitted Files

| File | Description |
|---|---|
| `Python/Lab4.py` | Completed Lab 4 Python implementation |
| `Python/Lab4_ECE569.ipynb` | Completed Jupyter notebook |
| `resource/tyagi55.csv` | Lissajous trajectory CSV (Steps 1–4) |
| `resource/tyagi55_bonus.csv` | Bonus heart ♥ trajectory CSV (Step 5) |

---

## Trajectory Summary

**Lissajous (Steps 1–4):**  
`x(t) = 0.12 sin(3t)`,  `y(t) = 0.10 sin(2t)`,  `T = 2π`,  `tfinal = 20 s`  
Average velocity `c = 0.0868 m/s` — well within 0.25 m/s limit

**Bonus (Step 5):**  
Heart ♥ symbol using parametric equation  
`x = 16sin³(t)`,  `y = 13cos(t) − 5cos(2t) − 2cos(3t) − cos(4t)`  
LED on/off feature used — LED=1 during heart stroke, LED=0 during transit moves

---

## Steps 1–3

For steps 1–3, use the MATLAB or Python starter code provided. First, make your own personal copy of the Lab 4 repo with the `Use This Template` button. Then, for MATLAB users, `git clone` the repository onto your personal computer (or download the zip). If using Python, you can clone on your personal machine or on eceprog. If using eceprog, run `git clone` from your home directory.

---

## Step 4 — RViz Verification

### Instructions for non-eceprog users

Download the UR description package to the `ws4/src` folder:

```bash
cd ~/ECE_Lab4/ws4/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
cd Universal_Robots_ROS2_Description
```

Checkout the `humble` or `jazzy` branch depending on your ROS version:

```bash
git checkout origin/humble
```

### Instructions for all users

Edit `ws4/src/msee22_description/urdf/msee22.urdf.xacro` for your ROS version (see lines 89 and 95).

Build all packages:

```bash
cd ~/ECE_Lab4/ws4/
colcon build --symlink-install
source install/setup.bash
```

Verify the robot appears on the table:

```bash
ros2 launch msee22_description view_room.launch.py
```

Check the `py_joint_pub` package works:

```bash
ros2 launch msee22_description move_robot.launch.py
```

### Loading your CSV

1. Copy your CSV file into `ws4/src/py_joint_pub/py_joint_pub/resource/`
2. In `ws4/src/py_joint_pub/py_joint_pub/joint_publisher_csv.py`, update line 18:

```python
filename = 'tyagi55.csv'          # Lissajous trajectory
# filename = 'tyagi55_bonus.csv'  # ← swap for bonus heart trajectory
```

3. Make sure your CSV has **no header row**, column order: `time, j1, j2, j3, j4, j5, j6, led`

> **Tip:** Upload your CSV to this GitHub repo, then run `git pull` on eceprog and move the file to the `resource/` folder. Alternatively, use `scp` to transfer directly from your local machine.

4. Rebuild and source, then run:

```bash
cd ~/ECE_Lab4/ws4/
colcon build --symlink-install
source install/setup.bash
ros2 launch msee22_description move_robot.launch.py
```

5. In RViz, enable the **tool0 trail** and take a screenshot for your report.

6. Upload `tyagi55.csv` to the **Lab4-CSV** assignment on Brightspace.

---

## Step 5 — Bonus

The bonus trajectory draws a **heart ♥** symbol using the robot's LED:

- Generated procedurally in Python using the classic heart parametric equation
- Local `y` offsets map to global `z` (vertical strokes) — key insight for correct orientation
- LED=1 during the heart stroke, LED=0 during pen-up transit moves
- Verified in RViz with no collisions, continuous velocity, and no kinematic singularities (min μ₃ = 0.262)

To run the bonus trajectory, update line 18 in `joint_publisher_csv.py`:

```python
filename = 'tyagi55_bonus.csv'
```

Then rebuild and run as above. Upload `tyagi55_bonus.csv` to the **Lab4-CSV-bonus** assignment on Brightspace.
