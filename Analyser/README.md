# 🛡️ EngineLib Physics Engine Analyser & Diagnostics Suite

An automated headless testing, telemetry recording, visual capture, and diagnostic system for `physics_engine.dll`.

The Analyser executes real gameplay and simulation scenarios, captures frame-by-frame physical metrics, records visual keyframe screenshots and animated replay GIFs, automatically detects physical anomalies (such as tunneling, contact jitter, sliding creep, and energy explosions), and prescribes concrete code-level engineering fixes with an interactive HTML dashboard.

---

## 🚀 Quick Start

### One-Click Execution (Windows)
Double-click `run.bat` or run from PowerShell:
```powershell
cd Z:\VSCode\GameLib\Analyser
.\run.bat
```

### CLI Execution
```powershell
# Analyze GameSim Demolition with Native DLL
python run_analysis.py --target gamesim --backend dll --frames 180

# Analyze GameSim Demolition with Pure Python Builtin (for comparison)
python run_analysis.py --target gamesim --backend builtin --frames 180

# Analyze 3D Ball Simulation
python run_analysis.py --target ball --frames 180

# Run full automated suite across scenarios
python run_analysis.py --target all
```

---

## 🏛️ Architecture & Modules

```
Z:\VSCode\GameLib\Analyser\
├── telemetry.py           # Frame-by-frame physical sampling (velocities, energies, penetrations)
├── recorder.py            # Headless visual capture, HUD overlay, keyframes & GIF compiler
├── physics_inspector.py   # Anomaly detection (tunneling, energy spikes, jitter, creep)
├── diagnostics.py         # Health scoring (0-100) & actionable fix prescriptions
├── report_generator.py    # Interactive HTML dashboard & Markdown report builder
├── run_analysis.py        # Automated test scenario runner & CLI
├── run.bat                # Windows launcher with automatic browser opening
└── reports/               # Output directory for HTML reports, screenshots, and replay GIFs
    └── gamesim_dll/
        ├── report.html
        ├── report.md
        ├── simulation_replay.gif
        ├── 01_cannon_launch.png
        ├── 02_projectile_impact.png
        ├── 03_structural_collapse.png
        └── 04_settled_equilibrium.png
```

---

## 🔍 Automated Physical Anomaly Checks

| Anomaly Code | Severity | Physical Phenomenon | Root Cause & Detection Rule |
|---|---|---|---|
| `ERR_TUNNELING_OR_DEEP_SINK` | **CRITICAL** | Penetration through floor/boundaries | Body penetrating > 14px beneath boundary. Needs higher sub-stepping or Baumgarte clamping. |
| `ERR_KINETIC_ENERGY_EXPLOSION` | **CRITICAL** | Artificial energy generation | Kinetic energy spiking > 3.0× without external impulse, indicating solver divergence. |
| `WARN_RESTING_CONTACT_CHATTER` | **WARNING** | Resting micro-jitter & vibration | Resting blocks oscillating velocity direction (> 30% sign flips) instead of settling into sleep. |
| `WARN_PERPETUAL_SLIDING_CREEP` | **WARNING** | Friction failure / continuous sliding | Bodies sliding indefinitely at low velocity without coming to rest; indicates Coulomb friction cutoff defect. |
| `WARN_GRAVITY_ACCELERATION_ERROR` | **WARNING** | Unrealistic free-fall acceleration | Measured downward acceleration deviates > 25% from configured gravity ($g = 9.81\text{ m/s}^2$ or $640\text{ px/s}^2$). |
| `WARN_FLOOR_SINKING` | **WARNING** | Moderate resting contact depression | Resting blocks penetrating 5-14px into ground due to soft Baumgarte factor. |

---

## 📊 Reports & Visual Evidence

Each analysis run automatically generates:
1. **Interactive HTML Dashboard (`report.html`)**:
   - Overall Physics Health Score (`A+`, `B`, `C`, `F`)
   - Run metrics (peak kinetic energy, settling time, max floor penetration)
   - Visual Evidence gallery with keyframe screenshots
   - Dynamic simulation replay GIF
   - Anomaly breakdown table with exact timestamps
   - Step-by-step engineering recommendations
2. **GitHub-Flavored Markdown Report (`report.md`)**:
   - Ready for inclusion in issues, pull requests, and documentation.
3. **Animated Simulation Replay GIF (`simulation_replay.gif`)**:
   - High-fidelity visual replay with real-time HUD telemetry and velocity vectors.
