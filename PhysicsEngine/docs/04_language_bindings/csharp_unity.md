# C# & Unity / Godot Integration Guide

This document details how to integrate the compiled physics dynamic library into modern C# environments, including standalone .NET 8/9 applications, Unity, and Godot Engine.

---

## 1. P/Invoke Native Method Declarations

```csharp
using System;
using System.Runtime.InteropServices;

public static class PhysicsNative
{
    private const string DLL_NAME = "physics_engine";

    [StructLayout(LayoutKind.Sequential)]
    public struct Vector3D
    {
        public float X;
        public float Y;
        public float Z;

        public Vector3D(float x, float y, float z)
        {
            X = x; Y = y; Z = z;
        }

        public override string ToString() => $"({X:F2}, {Y:F2}, {Z:F2})";
    }

    [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern void init_physics_world(float gx, float gy, float gz);

    [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern uint add_rigid_body(
        float px, float py, float pz,
        float sx, float sy, float sz,
        float mass, float restitution
    );

    [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
    [return: MarshalAs(UnmanagedType.I1)]
    public static extern bool get_body_position(uint id, out Vector3D outPos);

    [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern void apply_force(uint id, float fx, float fy, float fz);

    [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern void step_physics(float dt);
}
```

---

## 2. Unity Engine Integration: `PhysicsRigidBody` Component

In Unity, attach this `MonoBehaviour` script to any GameObject to hand off spatial integration and collision resolution to the native Rust engine:

```csharp
using UnityEngine;

public class PhysicsRigidBody : MonoBehaviour
{
    [Header("Physical Parameters")]
    public float mass = 1.0f;
    [Range(0f, 1f)]
    public float restitution = 0.8f;

    private uint _bodyId;
    private static bool _worldInitialized = false;

    private void Awake()
    {
        if (!_worldInitialized)
        {
            // Initialize global gravity matching Unity's physics settings
            PhysicsNative.init_physics_world(0.0f, -9.81f, 0.0f);
            _worldInitialized = true;
        }
    }

    private void Start()
    {
        Vector3 pos = transform.position;
        Vector3 size = transform.localScale;

        _bodyId = PhysicsNative.add_rigid_body(
            pos.x, pos.y, pos.z,
            size.x, size.y, size.z,
            mass, restitution
        );
    }

    private void FixedUpdate()
    {
        // On the primary physics controller:
        // PhysicsNative.step_physics(Time.fixedDeltaTime);

        // Sync position back to Unity Transform
        if (PhysicsNative.get_body_position(_bodyId, out PhysicsNative.Vector3D pos))
        {
            transform.position = new Vector3(pos.X, pos.Y, pos.Z);
        }
    }

    public void AddForce(Vector3 force)
    {
        PhysicsNative.apply_force(_bodyId, force.x, force.y, force.z);
    }
}
```

---

## 3. High Performance with `NativeArray` & Unsafe Pointers

For thousands of simulated objects, C# can pass pinned pointers using `fixed` statements directly to the batch transform API (`sync_all_transforms`), avoiding marshaling allocations on the managed GC heap entirely:

```csharp
[StructLayout(LayoutKind.Sequential)]
public struct BodyTransform
{
    public uint Id;
    public PhysicsNative.Vector3D Position;
}

public unsafe void SyncAllBodies(BodyTransform[] transforms, out uint written)
{
    fixed (BodyTransform* pTransforms = transforms)
    fixed (uint* pWritten = &written)
    {
        NativeBatchMethods.sync_all_transforms(pTransforms, (uint)transforms.Length, pWritten);
    }
}
```
