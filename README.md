# 6-DOF Planar Robot Simulation

A 6-DOF planar robotic manipulator simulation written in C.  
The system uses OpenGL for real-time visualization and GSL for matrix computation.

Developed and tested on Linux (Ubuntu).

---

## Features

- Forward kinematics
- Analytical Jacobian
- Moore–Penrose pseudoinverse
- Joint-space PD control
- Task-space PD control
- Inverse Jacobian Cartesian control
- Real-time OpenGL visualization

---

## Project Structure

```
.
├── src/
│   ├── planargl.c
│   ├── planar.c
│   └── moore_penrose_pseudoinverse.c
│
├── include/
│   └── serial.h
│
├── Makefile
└── README.md
```

---

## Dependencies (Linux)

```bash
sudo apt install freeglut3-dev libgsl-dev
```

---

## Build

```bash
make
```

## Run

```bash
./planar
```

---

# Controls

## Program
- `ESC` → Exit

---

## Camera

### Rotation
- `X` / `x` → Rotate about X-axis
- `Y` / `y` → Rotate about Y-axis
- `Z` / `z` → Rotate about Z-axis

### Translation
- `w` / `s` → Move along X
- `a` / `d` → Move along Y
- `f` / `r` → Move along Z

### Zoom
- `p` → Zoom in  
- `o` → Zoom out  

---

## Modes

- `-` → Toggle Angle Control
- `=` → Toggle PD Control
- `t` / `T` → Toggle Task-Space Control  
  (Current end-effector position becomes target)

---

## Joint Control (±2.5°)

| Joint | Increase | Decrease |
|--------|----------|----------|
| J1 | `1` | `!` |
| J2 | `2` | `@` |
| J3 | `3` | `#` |
| J4 | `4` | `$` |
| J5 | `5` | `%` |
| J6 | `6` | `^` |

Mode-dependent behavior:
- Direct joint update  
- Target-based control  
- PD tracking  

---

## Cartesian Control (±1 cm)

| Axis | Increase | Decrease |
|------|----------|----------|
| X | `7` | `&` |
| Y | `8` | `*` |
| Z | `9` | `(` |

Uses inverse Jacobian mapping.  
If PD is enabled, motion is PD-corrected.

---

Educational robotics simulation project implemented in C.

