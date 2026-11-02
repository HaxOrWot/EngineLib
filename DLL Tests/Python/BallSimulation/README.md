# 3D Ball Physics Simulation (Pygame + EngineLib DLL)

A real-time, interactive 3D physics demonstration rendering colored spherical rigid bodies in a 3D arena powered by `physics_engine.dll`.

---

## 🎮 How to Run

### Option 1: Double-click `run.bat`
Or from PowerShell / Command Prompt:
```powershell
Set-Location "Z:\VSCode\GameLib\DLL Tests\Python\BallSimulation"
python simulation.py
```

---

## 🕹️ Interactive Controls

| Control | Action |
| :--- | :--- |
| **Left Click + Drag** | Orbit camera around the 3D arena (pitch & yaw) |
| **Mouse Scroll Wheel** | Zoom camera in and out |
| **`[SPACE]`** | Drop a new dynamic sphere from the sky |
| **`[F]`** | Blast all balls with an explosive upward force |
| **`[G]`** | Toggle Earth gravity on / off |
| **`[R]`** | Reset and re-populate the arena |

---

## ⚡ Physics Engine Features Showcased
- **Native Sphere Primitives**: Collision detection via `Sphere-Sphere` radial separation and `Sphere-AABB` surface clamping.
- **Dynamic Ground Shadows**: Projected ellipse shadows scaling and fading with height.
- **Depth Sorting**: Real-time Painter's Algorithm sorting 3D objects from back to front.
- **Coulomb Friction & Restitution**: Elastic rebounding off the floor and arena walls.
