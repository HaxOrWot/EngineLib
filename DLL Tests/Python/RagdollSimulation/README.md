# 3D Ragdoll Physics Simulation (EngineLib DLL + Pygame)

An articulated skeletal ragdoll physics demonstration powered by `physics_engine.dll` featuring distance constraint solvers, staircase tumbling, dynamic shadows, and 3D camera controls.

---

## 🎮 How to Run

### Option 1: Double-click `run.bat`
Or from PowerShell / Command Prompt:
```powershell
Set-Location "Z:\VSCode\GameLib\DLL Tests\Python\RagdollSimulation"
python ragdoll_simulation.py
```

---

## 🕹️ Interactive Controls

| Control | Action |
| :--- | :--- |
| **Left Click + Drag** | Orbit 3D camera around the ragdoll scene |
| **Mouse Scroll Wheel** | Zoom camera in and out |
| **`[SPACE]`** | Drop a new articulated ragdoll from the sky |
| **`[F]`** | **Explosion Blast**: Launches all ragdolls into the air with randomized forces |
| **`[R]`** | Reset and re-populate the arena |

---

## ⚡ Realism & Architecture Highlights
- **XPBD Distance Constraints**: 10 skeletal joints connecting head, chest, pelvis, upper/lower arms, thighs, and shins.
- **Multi-Substep Solver**: Uses 4 sub-steps per frame and 6 constraint solver iterations for stiff, non-stretchy joints.
- **Linear Damping & Muscle Drag**: Limb joints absorb kinetic energy, producing natural floppy ragdoll physics instead of rubbery oscillation.
- **Staircase & Floor Collisions**: Spherical joint collision against static environment geometry with restitution and Coulomb friction.
