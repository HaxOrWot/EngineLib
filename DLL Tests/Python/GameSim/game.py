#!/usr/bin/env python3
"""
SiegePhysics: Castle Demolition
A complete, physics-driven demolition siege game built with Pygame.
Now integrated with EngineLib Native C++ Dynamic Library (physics_engine.dll)!
Allows toggling between Native DLL and Pure Python physics with [T].

Controls:
  • Mouse Aim & Left Click (or Hold & Release): Fire Projectile
  • [1]: Heavy Cannonball  |  [2]: Explosive Bomb  |  [3]: Cluster Shot  |  [4]: Bouncy Shot
  • Left Click mid-air with Cluster Shot: Split into 3 balls!
  • [T]: Toggle Physics Backend (Native DLL <-> Pure Python)
  • [R]: Restart Level    |  [N]: Next Level (when cleared)
  • [1-3] on Menu: Select Level
"""

import os
import sys
import math
import random
import pygame

# Physics Engine Backends
from physics_builtin import BuiltinPhysicsWorld, Vector2D as BuiltinVector2D
from physics_dll import DLLPhysicsWorld, Vector2D as DLLVector2D

# ---------------------------------------------------------------------------
# Visual Styles & Color Constants
# ---------------------------------------------------------------------------
COLOR_BG_SKY_TOP = (25, 35, 55)
COLOR_BG_SKY_BOT = (85, 115, 155)
COLOR_HILLS_FAR = (40, 65, 80)
COLOR_HILLS_NEAR = (30, 85, 60)
COLOR_GROUND = (45, 120, 65)
COLOR_GROUND_DIRT = (60, 45, 30)

COLOR_WOOD = (175, 115, 65)
COLOR_WOOD_BORDER = (115, 70, 35)
COLOR_STONE = (135, 145, 155)
COLOR_STONE_BORDER = (85, 95, 105)
COLOR_TNT = (225, 45, 45)
COLOR_TNT_YELLOW = (245, 215, 45)
COLOR_GLASS = (140, 220, 240, 180)
COLOR_ENEMY = (110, 205, 75)
COLOR_ENEMY_BORDER = (65, 135, 45)
COLOR_ENEMY_HELMET = (180, 175, 160)

# Projectile configurations
PROJECTILE_TYPES = {
    1: {"name": "Cannonball", "mass": 25.0, "radius": 14, "restitution": 0.25, "friction": 0.5, "color": (55, 60, 70)},
    2: {"name": "Bomb Ball", "mass": 18.0, "radius": 15, "restitution": 0.35, "friction": 0.5, "color": (210, 40, 40)},
    3: {"name": "Cluster Shot", "mass": 20.0, "radius": 13, "restitution": 0.30, "friction": 0.5, "color": (230, 150, 40)},
    4: {"name": "Bouncy Ball", "mass": 12.0, "radius": 12, "restitution": 0.90, "friction": 0.25, "color": (60, 215, 235)},
}

# ---------------------------------------------------------------------------
# Particle FX System
# ---------------------------------------------------------------------------
class Particle:
    __slots__ = ('x', 'y', 'vx', 'vy', 'color', 'radius', 'life', 'max_life', 'gravity')

    def __init__(self, x, y, vx, vy, color, radius, life, gravity=400.0):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.color = color
        self.radius = radius
        self.life = life
        self.max_life = life
        self.gravity = gravity

    def update(self, dt):
        self.vy += self.gravity * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.life -= dt
        return self.life > 0.0

    def draw(self, surface):
        alpha_factor = max(0.0, min(1.0, self.life / self.max_life))
        r = max(1, int(self.radius * alpha_factor))
        c = self.color
        if len(c) == 4:
            col = (c[0], c[1], c[2], int(c[3] * alpha_factor))
            surf = pygame.Surface((r * 2 + 2, r * 2 + 2), pygame.SRCALPHA)
            pygame.draw.circle(surf, col, (r + 1, r + 1), r)
            surface.blit(surf, (int(self.x - r - 1), int(self.y - r - 1)))
        else:
            pygame.draw.circle(surface, c, (int(self.x), int(self.y)), r)


class FloatingText:
    __slots__ = ('x', 'y', 'text', 'color', 'life', 'max_life', 'size')

    def __init__(self, x, y, text, color=(255, 235, 90), life=1.4, size=20):
        self.x = x
        self.y = y
        self.text = text
        self.color = color
        self.life = life
        self.max_life = life
        self.size = size

    def update(self, dt):
        self.y -= 35.0 * dt
        self.life -= dt
        return self.life > 0.0


# ---------------------------------------------------------------------------
# Game Application
# ---------------------------------------------------------------------------
class SiegePhysicsGame:
    def __init__(self):
        pygame.init()
        pygame.font.init()

        self.width = 1120
        self.height = 680
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        pygame.display.set_caption("SiegePhysics: Castle Demolition (EngineLib Native DLL Active)")

        self.clock = pygame.time.Clock()
        self.font_small = pygame.font.SysFont("Segoe UI", 15)
        self.font_med = pygame.font.SysFont("Segoe UI", 18, bold=True)
        self.font_large = pygame.font.SysFont("Segoe UI", 28, bold=True)
        self.font_title = pygame.font.SysFont("Segoe UI", 42, bold=True)

        # Physics Backend Mode: 'DLL' or 'BUILTIN'
        self.physics_mode = "DLL"
        self.world = None

        # Cannon Setup
        self.cannon_x = 130
        self.cannon_y = 520
        self.aim_angle = -math.pi / 4.0
        self.aim_power = 750.0
        self.min_power = 250.0
        self.max_power = 1100.0

        # Ammo Inventory
        self.selected_ammo = 1
        self.shots_left = 5
        self.active_projectiles = []

        # Particles & Popups
        self.particles = []
        self.floating_texts = []
        self.screen_shake = 0.0

        # Game State
        self.state = "MENU"
        self.current_level = 1
        self.total_score = 0
        self.level_score = 0
        self.targets_remaining = 0

        # Load Initial Level with Native DLL
        self.load_level(1)

    def toggle_physics_backend(self):
        """Swaps in real-time between Native DLL and Pure Python engines"""
        if self.physics_mode == "DLL":
            self.physics_mode = "BUILTIN"
            self.floating_texts.append(FloatingText(self.width / 2, 75, "BACKEND: PURE PYTHON PHYSICS", (130, 220, 255), life=2.2, size=24))
        else:
            self.physics_mode = "DLL"
            self.floating_texts.append(FloatingText(self.width / 2, 75, "BACKEND: NATIVE DLL (physics_engine.dll)", (100, 255, 140), life=2.2, size=24))
        self.load_level(self.current_level)

    # -----------------------------------------------------------------------
    # Level Generation
    # -----------------------------------------------------------------------
    def load_level(self, level_num):
        self.current_level = level_num
        self.active_projectiles.clear()
        self.particles.clear()
        self.floating_texts.clear()
        self.level_score = 0
        self.screen_shake = 0.0

        # Instantiate Selected Physics World
        if self.world is not None and hasattr(self.world, "destroy"):
            self.world.destroy()

        if self.physics_mode == "DLL":
            self.world = DLLPhysicsWorld(gravity_x=0.0, gravity_y=950.0)
        else:
            self.world = BuiltinPhysicsWorld(gravity_x=0.0, gravity_y=950.0)

        # Static Floor & Bedrock
        floor_y = 610
        self.floor = self.world.create_box(
            self.width / 2.0, floor_y + 40, self.width + 100, 80,
            mass=0.0, restitution=0.2, friction=0.7, material="terrain"
        )
        self.floor.color = COLOR_GROUND_DIRT

        # Left border wall
        self.world.create_box(-20, self.height / 2.0, 40, self.height * 2, mass=0.0, restitution=0.4, material="terrain")
        # Right border wall
        self.world.create_box(self.width + 20, self.height / 2.0, 40, self.height * 2, mass=0.0, restitution=0.4, material="terrain")

        # Handcrafted Castle Structures
        if level_num == 1:
            self._build_level_1(floor_y)
            self.shots_left = 5
        elif level_num == 2:
            self._build_level_2(floor_y)
            self.shots_left = 5
        else:
            self._build_level_3(floor_y)
            self.shots_left = 6

        self.targets_remaining = sum(1 for b in self.world.bodies if b.material == "enemy")

    def _build_level_1(self, ground_y):
        """Level 1: The Wooden Watchtower (Simple 3-tier structure with 2 targets)"""
        base_x = 780

        # Ground Foundation Slab
        self._add_block(base_x, ground_y - 10, 180, 20, "stone")

        # First Tier (Two pillars + Crossbar)
        self._add_block(base_x - 60, ground_y - 65, 22, 90, "wood")
        self._add_block(base_x + 60, ground_y - 65, 22, 90, "wood")
        self._add_target(base_x, ground_y - 30, radius=18)
        self._add_block(base_x, ground_y - 120, 170, 20, "wood")

        # Second Tier
        self._add_block(base_x - 50, ground_y - 175, 20, 90, "wood")
        self._add_block(base_x + 50, ground_y - 175, 20, 90, "wood")
        self._add_block(base_x, ground_y - 230, 140, 20, "wood")

        # Top Lookout
        self._add_target(base_x, ground_y - 260, radius=18)
        self._add_block(base_x - 30, ground_y - 280, 16, 60, "wood")
        self._add_block(base_x + 30, ground_y - 280, 16, 60, "wood")
        self._add_block(base_x, ground_y - 320, 90, 18, "stone")

    def _build_level_2(self, ground_y):
        """Level 2: The TNT Fortress (Stone columns with explosive TNT caches & 3 targets)"""
        base_x = 750

        # Outer barricade
        self._add_block(base_x - 170, ground_y - 40, 25, 80, "stone")
        self._add_target(base_x - 130, ground_y - 25, radius=18)

        # Main Castle Pillars
        self._add_block(base_x - 70, ground_y - 60, 28, 120, "stone")
        self._add_tnt(base_x, ground_y - 25, 36, 36)
        self._add_block(base_x + 70, ground_y - 60, 28, 120, "stone")

        # Main floor slab
        self._add_block(base_x, ground_y - 130, 200, 22, "stone")

        # Mid level with target and TNT
        self._add_block(base_x - 60, ground_y - 185, 22, 90, "wood")
        self._add_target(base_x, ground_y - 155, radius=18)
        self._add_tnt(base_x + 25, ground_y - 155, 32, 32)
        self._add_block(base_x + 60, ground_y - 185, 22, 90, "wood")

        # Roof & High Lookout
        self._add_block(base_x, ground_y - 240, 160, 20, "stone")
        self._add_block(base_x, ground_y - 285, 20, 70, "wood")
        self._add_target(base_x + 40, ground_y - 265, radius=18)
        self._add_block(base_x + 40, ground_y - 325, 60, 16, "wood")

    def _build_level_3(self, ground_y):
        """Level 3: The King's Citadel (Reinforced double towers + bridge + 4 armored targets)"""
        left_x = 640
        right_x = 880

        # Left Tower
        self._add_block(left_x - 45, ground_y - 70, 26, 140, "stone")
        self._add_block(left_x + 45, ground_y - 70, 26, 140, "stone")
        self._add_target(left_x, ground_y - 30, radius=18)
        self._add_block(left_x, ground_y - 150, 130, 20, "stone")

        self._add_block(left_x - 35, ground_y - 215, 22, 110, "wood")
        self._add_block(left_x + 35, ground_y - 215, 22, 110, "wood")
        self._add_target(left_x, ground_y - 185, radius=18)
        self._add_block(left_x, ground_y - 280, 110, 20, "stone")

        # Right Tower (Taller)
        self._add_block(right_x - 50, ground_y - 80, 28, 160, "stone")
        self._add_tnt(right_x, ground_y - 30, 40, 40)
        self._add_block(right_x + 50, ground_y - 80, 28, 160, "stone")
        self._add_block(right_x, ground_y - 170, 140, 22, "stone")

        self._add_block(right_x - 40, ground_y - 235, 24, 110, "stone")
        self._add_block(right_x + 40, ground_y - 235, 24, 110, "stone")
        self._add_tnt(right_x, ground_y - 200, 34, 34)
        self._add_block(right_x, ground_y - 300, 120, 20, "wood")

        # Crowned King on top of right tower
        self._add_target(right_x, ground_y - 330, radius=20, is_king=True)

        # High Connecting Bridge between towers
        bridge_x = (left_x + right_x) / 2.0
        self._add_block(bridge_x, ground_y - 160, 130, 16, "wood")
        self._add_target(bridge_x, ground_y - 190, radius=18)

    def _add_block(self, x, y, w, h, material):
        mass = (w * h) * (0.015 if material == "wood" else 0.040)
        restitution = 0.15 if material == "wood" else 0.10
        friction = 0.65 if material == "wood" else 0.75
        block = self.world.create_box(x, y, w, h, mass=mass, restitution=restitution, friction=friction, material=material)
        block.color = COLOR_WOOD if material == "wood" else COLOR_STONE
        return block

    def _add_tnt(self, x, y, w, h):
        block = self.world.create_box(x, y, w, h, mass=6.0, restitution=0.2, friction=0.6, material="tnt")
        block.color = COLOR_TNT
        return block

    def _add_target(self, x, y, radius=18, is_king=False):
        enemy = self.world.create_circle(x, y, radius, mass=4.0, restitution=0.15, friction=0.6, material="enemy")
        enemy.color = COLOR_ENEMY
        enemy.is_king = is_king
        return enemy

    # -----------------------------------------------------------------------
    # Firing Mechanics & Explosions
    # -----------------------------------------------------------------------
    def fire_projectile(self):
        if self.shots_left <= 0:
            return

        conf = PROJECTILE_TYPES[self.selected_ammo]
        barrel_len = 50.0
        spawn_x = self.cannon_x + math.cos(self.aim_angle) * barrel_len
        spawn_y = self.cannon_y + math.sin(self.aim_angle) * barrel_len

        proj = self.world.create_circle(
            spawn_x, spawn_y, conf["radius"],
            mass=conf["mass"], restitution=conf["restitution"], friction=conf["friction"],
            material="cannonball"
        )
        proj.color = conf["color"]
        proj.projectile_type = self.selected_ammo
        proj.timer = 0.0
        proj.split_done = False

        # Compute initial launch velocity vector
        vx = math.cos(self.aim_angle) * self.aim_power
        vy = math.sin(self.aim_angle) * self.aim_power

        # Apply velocity into active physics engine (Native DLL or Python)
        if hasattr(self.world, "set_body_velocity"):
            self.world.set_body_velocity(proj, vx, vy)
        else:
            proj.velocity = BuiltinVector2D(vx, vy)

        self.active_projectiles.append(proj)
        self.shots_left -= 1
        self.screen_shake = 6.0

        # Muzzle Flash particles
        for _ in range(16):
            ang = self.aim_angle + random.uniform(-0.4, 0.4)
            spd = random.uniform(100, 350)
            self.particles.append(Particle(
                spawn_x, spawn_y,
                math.cos(ang) * spd, math.sin(ang) * spd,
                random.choice([(255, 220, 60), (255, 140, 30), (220, 50, 30)]),
                random.uniform(3, 7), random.uniform(0.15, 0.35)
            ))

    def trigger_explosion(self, x, y, radius=140.0, force=35000.0):
        self.screen_shake = 16.0
        hit_bodies = self.world.apply_radial_impulse(x, y, radius, force)

        # Blast particles
        for _ in range(40):
            ang = random.uniform(0, math.pi * 2)
            spd = random.uniform(60, 450)
            col = random.choice([
                (255, 230, 70), (255, 150, 30), (240, 50, 30), (80, 80, 80, 200)
            ])
            self.particles.append(Particle(
                x, y, math.cos(ang) * spd, math.sin(ang) * spd,
                col, random.uniform(4, 12), random.uniform(0.3, 0.7), gravity=200.0
            ))

        self.floating_texts.append(FloatingText(x, y - 25, "BOOM!", (255, 210, 60), size=24))

        for b, died, dmg in hit_bodies:
            if died:
                self._handle_body_death(b)

    def _handle_body_death(self, body):
        if body.material == "enemy":
            score_gain = 500 if getattr(body, "is_king", False) else 250
            self.level_score += score_gain
            self.floating_texts.append(FloatingText(body.position.x, body.position.y, f"+{score_gain} TARGET DOWN!", (120, 255, 120)))
        elif body.material == "tnt":
            self.trigger_explosion(body.position.x, body.position.y)
            self.level_score += 150
        elif body.material in ("wood", "stone"):
            self.level_score += 50

        # Debris fragments
        for _ in range(8):
            ang = random.uniform(0, math.pi * 2)
            spd = random.uniform(40, 220)
            self.particles.append(Particle(
                body.position.x, body.position.y,
                math.cos(ang) * spd, math.sin(ang) * spd,
                body.color, random.uniform(2, 5), random.uniform(0.4, 0.9)
            ))

    # -----------------------------------------------------------------------
    # Main Simulation Step
    # -----------------------------------------------------------------------
    def update(self, dt):
        # 1. Step the active physics engine (Native DLL / Builtin)
        self.world.step(dt)

        # 2. Update Active Projectiles
        rem_projectiles = []
        for p in self.active_projectiles:
            p.timer += dt

            # Bomb ball detonates after 2.2s
            if p.projectile_type == 2 and p.timer > 2.2:
                self.trigger_explosion(p.position.x, p.position.y)
                self.world.remove_body(p)
                continue

            # Smoke trail
            if random.random() < 0.6:
                self.particles.append(Particle(
                    p.position.x, p.position.y,
                    random.uniform(-10, 10), random.uniform(-10, 10),
                    (180, 180, 190, 140), random.uniform(2, 5), 0.35, gravity=-30.0
                ))

            # Despawn off-screen or stopped projectiles
            if p.position.x > self.width + 100 or p.position.x < -100 or p.position.y > self.height + 100:
                self.world.remove_body(p)
            elif p.timer > 7.0 and p.velocity.length() < 25.0:
                self.world.remove_body(p)
            else:
                rem_projectiles.append(p)

        self.active_projectiles = rem_projectiles

        # 3. Clean up dead bodies & check TNT triggers
        dead_bodies = []
        for b in list(self.world.bodies):
            if b.is_dead:
                dead_bodies.append(b)
            elif b.material == "tnt" and b.health < b.max_health:
                b.is_dead = True
                dead_bodies.append(b)

        for b in dead_bodies:
            self._handle_body_death(b)
            self.world.remove_body(b)

        # 4. Check Win / Loss condition
        active_targets = sum(1 for b in self.world.bodies if b.material == "enemy")
        self.targets_remaining = active_targets

        if self.state == "PLAYING":
            if active_targets == 0:
                bonus = self.shots_left * 500
                self.level_score += bonus
                self.total_score += self.level_score
                self.state = "LEVEL_CLEAR"
                self.floating_texts.append(FloatingText(self.width / 2, 220, "STAGE CLEARED!", (255, 240, 80), life=3.0, size=34))
            elif self.shots_left == 0 and len(self.active_projectiles) == 0:
                all_settled = all(b.velocity.length() < 35.0 for b in self.world.bodies if not b.is_static)
                if all_settled:
                    self.state = "GAME_OVER"

        # 5. Update Particles & Floating Text
        self.particles = [p for p in self.particles if p.update(dt)]
        self.floating_texts = [ft for ft in self.floating_texts if ft.update(dt)]

        # 6. Screen Shake decay
        if self.screen_shake > 0.0:
            self.screen_shake = max(0.0, self.screen_shake - dt * 25.0)

    # -----------------------------------------------------------------------
    # Rendering Pipeline
    # -----------------------------------------------------------------------
    def render(self):
        ox, oy = 0, 0
        if self.screen_shake > 0.0:
            ox = int(random.uniform(-self.screen_shake, self.screen_shake))
            oy = int(random.uniform(-self.screen_shake, self.screen_shake))

        # 1. Background Sky & Hills
        self._draw_scenery(ox, oy)

        # 2. Trajectory Prediction Line
        if self.state == "PLAYING" and self.shots_left > 0:
            self._draw_trajectory(ox, oy)

        # 3. Physics Rigid Bodies (Blocks, Enemies, Projectiles)
        for b in self.world.bodies:
            self._draw_body(b, ox, oy)

        # 4. Particles
        for p in self.particles:
            p.draw(self.screen)

        # 5. Cannon Battery
        self._draw_cannon(ox, oy)

        # 6. Floating Text
        for ft in self.floating_texts:
            f_surf = self.font_med.render(ft.text, True, ft.color)
            self.screen.blit(f_surf, (int(ft.x - f_surf.get_width() / 2), int(ft.y)))

        # 7. Modern UI / HUD
        self._draw_hud()

        # 8. State Modals
        if self.state == "MENU":
            self._draw_menu_modal()
        elif self.state == "LEVEL_CLEAR":
            self._draw_level_clear_modal()
        elif self.state == "GAME_OVER":
            self._draw_game_over_modal()

        pygame.display.flip()

    def _draw_scenery(self, ox, oy):
        for y in range(0, self.height, 4):
            factor = y / float(self.height)
            r = int(COLOR_BG_SKY_TOP[0] * (1.0 - factor) + COLOR_BG_SKY_BOT[0] * factor)
            g = int(COLOR_BG_SKY_TOP[1] * (1.0 - factor) + COLOR_BG_SKY_BOT[1] * factor)
            b = int(COLOR_BG_SKY_TOP[2] * (1.0 - factor) + COLOR_BG_SKY_BOT[2] * factor)
            pygame.draw.rect(self.screen, (r, g, b), (0, y, self.width, 4))

        hill_pts_far = [
            (0, 480), (180, 410), (380, 460), (560, 390), (760, 440), (960, 395), (self.width, 450),
            (self.width, self.height), (0, self.height)
        ]
        pygame.draw.polygon(self.screen, COLOR_HILLS_FAR, [(p[0] + ox // 2, p[1] + oy // 2) for p in hill_pts_far])

        hill_pts_near = [
            (0, 540), (140, 490), (320, 530), (520, 480), (710, 520), (910, 475), (self.width, 515),
            (self.width, self.height), (0, self.height)
        ]
        pygame.draw.polygon(self.screen, COLOR_HILLS_NEAR, [(p[0] + ox, p[1] + oy) for p in hill_pts_near])

    def _draw_cannon(self, ox, oy):
        cx = self.cannon_x + ox
        cy = self.cannon_y + oy

        barrel_len = 46.0
        bx = cx + math.cos(self.aim_angle) * barrel_len
        by = cy + math.sin(self.aim_angle) * barrel_len

        pygame.draw.line(self.screen, (20, 24, 30), (cx, cy + 4), (bx, by + 4), 16)
        pygame.draw.line(self.screen, (75, 80, 95), (cx, cy), (bx, by), 14)
        pygame.draw.line(self.screen, (110, 120, 135), (cx, cy - 2), (bx, by - 2), 4)
        pygame.draw.circle(self.screen, (45, 50, 60), (int(bx), int(by)), 8)

        pygame.draw.circle(self.screen, (110, 70, 35), (cx - 10, cy + 18), 20)
        pygame.draw.circle(self.screen, (70, 40, 20), (cx - 10, cy + 18), 12)
        pygame.draw.circle(self.screen, (150, 155, 165), (cx - 10, cy + 18), 5)
        pygame.draw.circle(self.screen, (85, 90, 105), (cx, cy), 12)

    def _draw_trajectory(self, ox, oy):
        p0_x = self.cannon_x + math.cos(self.aim_angle) * 46.0
        p0_y = self.cannon_y + math.sin(self.aim_angle) * 46.0
        v0_x = math.cos(self.aim_angle) * self.aim_power
        v0_y = math.sin(self.aim_angle) * self.aim_power
        gy = self.world.gravity.y

        pts = []
        for i in range(1, 28):
            t = i * 0.04
            px = p0_x + v0_x * t + ox
            py = p0_y + v0_y * t + 0.5 * gy * (t ** 2) + oy
            if py > 610 or px > self.width:
                break
            pts.append((int(px), int(py)))

        for i, pt in enumerate(pts):
            radius = max(2, int(4.0 * (1.0 - i / len(pts))))
            color = (255, 230, 80) if i % 2 == 0 else (255, 140, 40)
            pygame.draw.circle(self.screen, color, pt, radius)

    def _draw_body(self, b, ox, oy):
        px = int(b.position.x + ox)
        py = int(b.position.y + oy)

        if b.shape_type == "circle":
            r = int(b.radius)
            if b.material == "enemy":
                pygame.draw.circle(self.screen, COLOR_ENEMY_BORDER, (px, py), r + 2)
                pygame.draw.circle(self.screen, b.color, (px, py), r)
                pygame.draw.arc(self.screen, COLOR_ENEMY_HELMET, (px - r, py - r - 2, r * 2, r * 2), 0, math.pi, 4)

                if getattr(b, "is_king", False):
                    crown_pts = [
                        (px - 12, py - r + 2),
                        (px - 14, py - r - 12),
                        (px - 6, py - r - 4),
                        (px, py - r - 15),
                        (px + 6, py - r - 4),
                        (px + 14, py - r - 12),
                        (px + 12, py - r + 2),
                    ]
                    pygame.draw.polygon(self.screen, (255, 215, 30), crown_pts)

                eye_y = py - 3
                pygame.draw.circle(self.screen, (255, 255, 255), (px - 5, eye_y), 4)
                pygame.draw.circle(self.screen, (255, 255, 255), (px + 5, eye_y), 4)
                pygame.draw.circle(self.screen, (20, 20, 20), (px - 4, eye_y), 2)
                pygame.draw.circle(self.screen, (20, 20, 20), (px + 6, eye_y), 2)
                pygame.draw.ellipse(self.screen, (85, 175, 55), (px - 7, py + 2, 14, 9))
                pygame.draw.circle(self.screen, (35, 75, 25), (px - 3, py + 6), 2)
                pygame.draw.circle(self.screen, (35, 75, 25), (px + 3, py + 6), 2)
            else:
                pygame.draw.circle(self.screen, (25, 25, 30), (px, py), r + 1)
                pygame.draw.circle(self.screen, b.color, (px, py), r)
                pygame.draw.circle(self.screen, (255, 255, 255), (px - int(r * 0.35), py - int(r * 0.35)), max(1, r // 4))

        elif b.shape_type == "box":
            half_w = b.width / 2.0
            half_h = b.height / 2.0
            rect = pygame.Rect(px - half_w, py - half_h, b.width, b.height)

            if b.material == "terrain":
                pygame.draw.rect(self.screen, COLOR_GROUND_DIRT, rect)
                pygame.draw.rect(self.screen, COLOR_GROUND, (rect.x, rect.y, rect.width, 10))
            elif b.material == "tnt":
                pygame.draw.rect(self.screen, COLOR_TNT, rect)
                pygame.draw.rect(self.screen, (150, 20, 20), rect, 2)
                pygame.draw.rect(self.screen, COLOR_TNT_YELLOW, (rect.x, rect.centery - 6, rect.width, 12))
                tnt_txt = self.font_small.render("TNT", True, (20, 20, 20))
                self.screen.blit(tnt_txt, (rect.centerx - tnt_txt.get_width() / 2, rect.centery - 8))
            elif b.material == "stone":
                pygame.draw.rect(self.screen, COLOR_STONE, rect)
                pygame.draw.rect(self.screen, COLOR_STONE_BORDER, rect, 2)
                if b.height > 30:
                    pygame.draw.line(self.screen, COLOR_STONE_BORDER, (rect.x, rect.centery), (rect.right, rect.centery), 1)
            else:
                pygame.draw.rect(self.screen, COLOR_WOOD, rect)
                pygame.draw.rect(self.screen, COLOR_WOOD_BORDER, rect, 2)
                pygame.draw.line(self.screen, COLOR_WOOD_BORDER, (rect.x + 4, rect.y + 4), (rect.right - 4, rect.y + 4), 1)
                pygame.draw.line(self.screen, COLOR_WOOD_BORDER, (rect.x + 4, rect.bottom - 4), (rect.right - 4, rect.bottom - 4), 1)

    def _draw_hud(self):
        top_surf = pygame.Surface((self.width, 54), pygame.SRCALPHA)
        top_surf.fill((16, 22, 34, 220))
        self.screen.blit(top_surf, (0, 0))

        title = self.font_med.render("SIEGEPHYSICS", True, (255, 225, 80))
        if self.physics_mode == "DLL":
            backend_tag = self.font_small.render("[NATIVE DLL ACTIVE: physics_engine.dll (Press [T] to toggle)]", True, (100, 255, 140))
        else:
            backend_tag = self.font_small.render("[PURE PYTHON BACKEND (Press [T] to toggle)]", True, (130, 220, 255))

        self.screen.blit(title, (20, 8))
        self.screen.blit(backend_tag, (20, 30))

        lvl_txt = self.font_med.render(f"Level: {self.current_level}/3", True, (240, 245, 255))
        targets_txt = self.font_med.render(f"Targets Left: {self.targets_remaining}", True, (120, 255, 140))
        score_txt = self.font_med.render(f"Score: {self.total_score + self.level_score}", True, (255, 215, 60))

        self.screen.blit(lvl_txt, (450, 16))
        self.screen.blit(targets_txt, (570, 16))
        self.screen.blit(score_txt, (750, 16))

        # Ammo Arsenal Selector (Bottom Left)
        panel_y = self.height - 75
        panel_surf = pygame.Surface((360, 65), pygame.SRCALPHA)
        panel_surf.fill((20, 26, 38, 220))
        self.screen.blit(panel_surf, (15, panel_y))

        shots_text = self.font_small.render(f"Shots Left: {self.shots_left}", True, (255, 235, 120))
        self.screen.blit(shots_text, (25, panel_y + 6))

        for k in range(1, 5):
            conf = PROJECTILE_TYPES[k]
            bx = 30 + (k - 1) * 82
            by = panel_y + 26
            is_sel = (self.selected_ammo == k)

            rect = pygame.Rect(bx, by, 72, 30)
            btn_col = (50, 75, 110) if is_sel else (32, 42, 60)
            pygame.draw.rect(self.screen, btn_col, rect, border_radius=4)
            if is_sel:
                pygame.draw.rect(self.screen, (255, 215, 60), rect, 2, border_radius=4)

            pygame.draw.circle(self.screen, conf["color"], (bx + 14, by + 15), 6)
            name_txt = self.font_small.render(f"[{k}]", True, (240, 240, 240))
            self.screen.blit(name_txt, (bx + 26, by + 6))

    def _draw_menu_modal(self):
        overlay = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        overlay.fill((10, 14, 22, 210))
        self.screen.blit(overlay, (0, 0))

        box_w, box_h = 600, 420
        bx = (self.width - box_w) // 2
        by = (self.height - box_h) // 2

        pygame.draw.rect(self.screen, (24, 32, 48), (bx, by, box_w, box_h), border_radius=12)
        pygame.draw.rect(self.screen, (255, 215, 75), (bx, by, box_w, box_h), 2, border_radius=12)

        t_surf = self.font_title.render("SIEGEPHYSICS", True, (255, 225, 70))
        sub_surf = self.font_med.render("Powered by EngineLib (physics_engine.dll)", True, (100, 255, 140))
        self.screen.blit(t_surf, (bx + (box_w - t_surf.get_width()) // 2, by + 26))
        self.screen.blit(sub_surf, (bx + (box_w - sub_surf.get_width()) // 2, by + 78))

        lines = [
            "• Aim with Mouse, Left Click or Drag to Fire",
            "• Use [1-4] to select Heavy, Bomb, Cluster, or Bouncy ammo",
            "• Click Cluster Shot mid-air to split into 3 projectiles!",
            "• [T]: Real-time toggle between Native DLL & Pure Python",
            "• Target wooden beams & TNT to trigger chain collapses!",
            "",
            "Select Level to Begin:",
            "[1] Level 1: Wooden Watchtower",
            "[2] Level 2: TNT Redoubt",
            "[3] Level 3: The King's Citadel",
            "",
            "Press [SPACE] or [1] to Play Level 1",
        ]

        y_off = by + 118
        for line in lines:
            col = (255, 235, 100) if "Play" in line or "Level:" in line or "[T]" in line else (200, 215, 235)
            txt = self.font_small.render(line, True, col)
            self.screen.blit(txt, (bx + 50, y_off))
            y_off += 21

    def _draw_level_clear_modal(self):
        overlay = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        overlay.fill((10, 14, 22, 190))
        self.screen.blit(overlay, (0, 0))

        box_w, box_h = 480, 280
        bx = (self.width - box_w) // 2
        by = (self.height - box_h) // 2

        pygame.draw.rect(self.screen, (26, 36, 52), (bx, by, box_w, box_h), border_radius=12)
        pygame.draw.rect(self.screen, (100, 255, 130), (bx, by, box_w, box_h), 3, border_radius=12)

        t_surf = self.font_large.render("LEVEL COMPLETE!", True, (120, 255, 140))
        self.screen.blit(t_surf, (bx + (box_w - t_surf.get_width()) // 2, by + 30))

        s_surf = self.font_med.render(f"Stage Score: +{self.level_score}", True, (255, 230, 90))
        t_tot = self.font_med.render(f"Total Campaign Score: {self.total_score}", True, (210, 230, 255))
        self.screen.blit(s_surf, (bx + (box_w - s_surf.get_width()) // 2, by + 85))
        self.screen.blit(t_tot, (bx + (box_w - t_tot.get_width()) // 2, by + 120))

        btn_txt = self.font_med.render("Press [N] for Next Level  |  [R] to Replay", True, (255, 255, 255))
        self.screen.blit(btn_txt, (bx + (box_w - btn_txt.get_width()) // 2, by + 200))

    def _draw_game_over_modal(self):
        overlay = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        overlay.fill((15, 10, 10, 200))
        self.screen.blit(overlay, (0, 0))

        box_w, box_h = 460, 250
        bx = (self.width - box_w) // 2
        by = (self.height - box_h) // 2

        pygame.draw.rect(self.screen, (42, 24, 28), (bx, by, box_w, box_h), border_radius=12)
        pygame.draw.rect(self.screen, (245, 60, 60), (bx, by, box_w, box_h), 2, border_radius=12)

        t_surf = self.font_large.render("OUT OF AMMO!", True, (255, 80, 80))
        self.screen.blit(t_surf, (bx + (box_w - t_surf.get_width()) // 2, by + 30))

        sub = self.font_med.render(f"Targets Still Standing: {self.targets_remaining}", True, (220, 200, 200))
        self.screen.blit(sub, (bx + (box_w - sub.get_width()) // 2, by + 90))

        btn = self.font_med.render("Press [R] to Try Again", True, (255, 225, 90))
        self.screen.blit(btn, (bx + (box_w - btn.get_width()) // 2, by + 160))

    # -----------------------------------------------------------------------
    # Main Game Loop
    # -----------------------------------------------------------------------
    def run(self):
        running = True
        while running:
            dt = self.clock.tick(60) / 1000.0
            dt = min(dt, 0.033)

            # Aiming Calculation
            mx, my = pygame.mouse.get_pos()
            dx = mx - self.cannon_x
            dy = my - self.cannon_y
            angle = math.atan2(dy, dx)
            self.aim_angle = max(-math.pi * 0.48, min(-0.05, angle))

            dist = math.hypot(dx, dy)
            self.aim_power = max(self.min_power, min(self.max_power, dist * 2.2))

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.VIDEORESIZE:
                    self.width, self.height = event.w, event.h
                    self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        if self.state == "MENU":
                            self.state = "PLAYING"
                        elif self.state == "PLAYING":
                            # Check if clicking mid-air cluster shot
                            split_triggered = False
                            for p in self.active_projectiles:
                                if p.projectile_type == 3 and not p.split_done:
                                    p.split_done = True
                                    split_triggered = True
                                    base_vx = p.velocity.x
                                    base_vy = p.velocity.y
                                    p.radius = 8
                                    p.mass = 8.0
                                    for spread in [-0.25, 0.25]:
                                        sp_x = p.position.x
                                        sp_y = p.position.y
                                        cos_s = math.cos(spread)
                                        sin_s = math.sin(spread)
                                        vx = base_vx * cos_s - base_vy * sin_s
                                        vy = base_vx * sin_s + base_vy * cos_s
                                        child = self.world.create_circle(sp_x, sp_y, 8, mass=8.0, material="cannonball")
                                        child.color = PROJECTILE_TYPES[3]["color"]
                                        child.projectile_type = 3
                                        if hasattr(self.world, "set_body_velocity"):
                                            self.world.set_body_velocity(child, vx, vy)
                                        else:
                                            child.velocity = BuiltinVector2D(vx, vy)
                                        child.timer = 0.0
                                        child.split_done = True
                                        self.active_projectiles.append(child)
                                    break

                            if not split_triggered and self.shots_left > 0:
                                self.fire_projectile()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        if self.state == "MENU":
                            self.state = "PLAYING"
                        elif self.state == "LEVEL_CLEAR":
                            if self.current_level < 3:
                                self.load_level(self.current_level + 1)
                                self.state = "PLAYING"
                            else:
                                self.load_level(1)
                                self.state = "PLAYING"
                        elif self.state == "GAME_OVER":
                            self.load_level(self.current_level)
                            self.state = "PLAYING"
                        elif self.shots_left > 0:
                            self.fire_projectile()
                    elif event.key == pygame.K_r:
                        self.load_level(self.current_level)
                        self.state = "PLAYING"
                    elif event.key == pygame.K_n and self.state == "LEVEL_CLEAR":
                        next_lvl = (self.current_level % 3) + 1
                        self.load_level(next_lvl)
                        self.state = "PLAYING"
                    elif event.key == pygame.K_t:
                        self.toggle_physics_backend()
                    elif event.key in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4):
                        if self.state == "MENU" and event.key in (pygame.K_1, pygame.K_2, pygame.K_3):
                            self.load_level(event.key - pygame.K_0)
                            self.state = "PLAYING"
                        else:
                            self.selected_ammo = event.key - pygame.K_0

            # Step Physics & Simulation
            if self.state in ("PLAYING", "LEVEL_CLEAR", "GAME_OVER"):
                self.update(dt)

            # Render Screen
            self.render()

        if self.world is not None and hasattr(self.world, "destroy"):
            self.world.destroy()
        pygame.quit()
        sys.exit(0)


if __name__ == "__main__":
    game = SiegePhysicsGame()
    game.run()
