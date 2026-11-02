# 🏰 SiegePhysics: Castle Demolition

A physics-driven medieval demolition and siege artillery game powered by **EngineLib (`physics_engine.dll`)**.

---

## ⚡ Native Physics DLL Integration

The game is now fully powered by your high-performance native C/Rust dynamic library:
- **`physics_engine.dll` Backend**: Active by default! All rigid body collisions (AABB vs AABB, Sphere vs AABB, Sphere vs Sphere), sub-stepping, restitution, and Coulomb friction are simulated directly in native machine code.
- **Dynamic Dual-Engine Toggle (`[T]` key)**: You can press `[T]` in real time to instantly switch between the **Native DLL** and the fallback **Pure Python** engine to compare performance and physics behavior side-by-side!

---

## 🎮 Gameplay & Physics Mechanics

- **Structural Collapse Dynamics**: Towers are assembled from real rigid bodies (wood beams, heavy stone masonry, explosive TNT). Knocking out foundational pillars causes upper floors to collapse under gravitational load.
- **Radial TNT Explosions**: Striking TNT caches triggers radial shockwaves and blast damage that send wooden planks and debris flying.
- **Crush Damage**: Enemy targets take damage when struck by high-velocity cannonballs or crushed under falling stone blocks.
- **Trajectory Guidance**: Parabolic dotted arc dynamically calculated using gravity integration.

---

## 🎯 Weapons & Projectile Types

| Key | Ammo Type | Description |
|:---:|:---|:---|
| **`[1]`** | **Cannonball** | Heavy high-density iron ball with punching momentum through wooden beams. |
| **`[2]`** | **Bomb Ball** | Timed/impact explosive that triggers a devastating radial shockwave. |
| **`[3]`** | **Cluster Shot** | Can be split mid-air into 3 projectiles by clicking again during flight! |
| **`[4]`** | **Bouncy Shot** | High restitution ($e = 0.90$) rubber sphere that ricochets through enclosed bunkers. |

---

## 🕹️ Controls

- **Mouse Aiming**: Move the mouse to orient the cannon barrel and trajectory arc.
- **Left Click**: Fire selected projectile (power scales with cursor distance).
- **Left Click mid-air** (with Cluster Shot): Splits projectile into 3 balls.
- **`[T]`**: **Toggle Physics Backend** (Native DLL <-> Pure Python).
- **`[1]` - `[4]`**: Switch ammo types.
- **`[SPACE]`**: Fire / Start / Continue.
- **`[R]`**: Reset current level.
- **`[N]`**: Advance to next level when cleared.

---

## 🚀 How to Run

Double-click `run.bat` or run:
```powershell
cd "Z:\VSCode\GameLib\DLL Tests\Python\GameSim"
python game.py
```
