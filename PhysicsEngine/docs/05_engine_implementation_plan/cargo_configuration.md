# Cargo Configuration & Production Optimization

This document describes the optimal `Cargo.toml` configuration, compiler flags, Link-Time Optimization (LTO), and binary size reduction settings for building a high-performance native dynamic library.

---

## 1. Production `Cargo.toml`

```toml
[package]
name = "physics_engine"
version = "0.1.0"
edition = "2021"
authors = ["GameLib Physics Team"]
description = "Universal cross-language 3D rigid body physics engine dynamic library"
license = "MIT OR Apache-2.0"

[lib]
# "cdylib" produces a dynamic system library (.dll on Windows, .so on Linux, .dylib on macOS)
# "rlib" allows internal Rust integration tests under /tests to link against the crate
crate-type = ["cdylib", "rlib"]

[dependencies]
# Zero runtime dependencies for instant compilation and minimal binary footprint

[profile.release]
# Level 3: Maximum aggressive performance optimizations
opt-level = 3

# Fat LTO performs link-time optimization across all crate compilation units
lto = "fat"

# Single codegen unit allows the LLVM backend to perform global inlining and vectorization
codegen-units = 1

# Immediately abort on panic; removes unwinding landing pads, reducing binary size
# Note: FFI functions should still use catch_unwind defensively if unwinding is enabled.
panic = "abort"

# Strip all debug symbols and symbol tables from the final dynamic binary
strip = true

# Enable CPU auto-vectorization (AVX2 / SSE4.2 when target-cpu=native is specified)
overflow-checks = false
```

---

## 2. Compilation Targets & Commands

### Standard Release Build (Windows DLL)
```powershell
cargo build --release
```
Produces: `target/release/physics_engine.dll`

### Cross-Compilation Targets
Rust enables effortless cross-compilation for all major game engine target platforms:

```powershell
# Windows x86_64 (MinGW / GNU)
cargo build --release --target x86_64-pc-windows-gnu

# Linux x86_64 (Ubuntu / SteamOS)
cargo build --release --target x86_64-unknown-linux-gnu

# macOS Apple Silicon (ARM64)
cargo build --release --target aarch64-apple-darwin

# Android ARM64
cargo build --release --target aarch64-linux-android
```

---

## 3. Compiler Performance Tuning Flags

To enable hardware-level SIMD vectorization on modern CPUs:

Create `.cargo/config.toml`:
```toml
[target.'cfg(all(target_arch = "x86_64", target_os = "windows"))']
rustflags = ["-C", "target-feature=+sse3,+avx2,+fma"]
```
This instructs LLVM to vectorize 3D vector arithmetic into single-cycle SIMD instructions (e.g. `_mm256_fmadd_ps`).
