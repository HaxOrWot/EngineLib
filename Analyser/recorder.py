"""
Visual Frame Recorder & GIF Compiler
Captures simulation screenshots, overlays real-time physics vectors & telemetry HUD,
and compiles animated visual replays using Pillow.
"""

import os
import math
import pygame
from PIL import Image

class VisualRecorder:
    def __init__(self, output_dir, max_gif_frames=120):
        self.output_dir = output_dir
        self.max_gif_frames = max_gif_frames
        self.captured_pil_frames = []
        self.saved_screenshots = {}
        self.gif_path = None

        os.makedirs(self.output_dir, exist_ok=True)
        self.font = None

    def _get_font(self):
        if self.font is None:
            if not pygame.font.get_init():
                pygame.font.init()
            self.font = pygame.font.SysFont("Consolas", 14)
        return self.font

    def capture_frame(self, surface, telemetry_info=None, draw_vectors=True, bodies=None):
        """Captures a surface, adds telemetry overlays, and stores for GIF compilation"""
        w, h = surface.get_size()
        # Create a copy surface to draw telemetry onto
        overlay_surf = surface.copy()

        # 1. Draw Body Velocity Vectors & Contact Dots
        if draw_vectors and bodies:
            for b in bodies:
                is_stat = getattr(b, 'is_static', False)
                if is_stat:
                    continue

                pos = getattr(b, 'position', None)
                vel = getattr(b, 'velocity', None)
                if pos and vel:
                    px, py = int(pos.x), int(pos.y)
                    vx, vy = vel.x, vel.y
                else:
                    px, py = int(getattr(b, 'x', 0)), int(getattr(b, 'y', 0))
                    vx, vy = getattr(b, 'vx', 0), getattr(b, 'vy', 0)

                speed = math.hypot(vx, vy)
                if speed > 10.0:
                    scale = 0.08
                    ex = int(px + vx * scale)
                    ey = int(py + vy * scale)
                    color = (255, 60, 60) if speed > 250.0 else (255, 220, 60)
                    pygame.draw.line(overlay_surf, color, (px, py), (ex, ey), 2)
                    pygame.draw.circle(overlay_surf, (255, 255, 255), (ex, ey), 3)

        # 2. Draw Diagnostic Overlay HUD in top-right corner
        if telemetry_info:
            font = self._get_font()
            lines = [
                f"[RECORDER TELEMETRY]",
                f"Time: {telemetry_info.get('time', 0.0):.2f}s | Frame: {telemetry_info.get('frame', 0)}",
                f"Kinetic Energy: {telemetry_info.get('ke', 0.0):.0f} J",
                f"Max Speed: {telemetry_info.get('max_speed', 0.0):.1f} px/s",
                f"Floor Pen: {telemetry_info.get('max_pen', 0.0):.1f} px",
                f"Active/Sleep: {telemetry_info.get('active', 0)}/{telemetry_info.get('sleep', 0)}",
            ]

            panel_w = 260
            panel_h = len(lines) * 18 + 14
            panel = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
            panel.fill((16, 20, 30, 200))
            pygame.draw.rect(panel, (60, 140, 220), (0, 0, panel_w, panel_h), 1)

            for i, line in enumerate(lines):
                col = (100, 220, 255) if i == 0 else (220, 230, 245)
                txt = font.render(line, True, col)
                panel.blit(txt, (10, 6 + i * 18))

            overlay_surf.blit(panel, (w - panel_w - 15, 15))

        # Convert to PIL Image
        raw_str = pygame.image.tostring(overlay_surf, "RGB")
        pil_img = Image.frombytes("RGB", (w, h), raw_str)

        # Add to frame buffer (subsampled if exceeding max frames)
        if len(self.captured_pil_frames) < self.max_gif_frames:
            self.captured_pil_frames.append(pil_img)
        elif len(self.captured_pil_frames) % 2 == 0:
            # Drop every second frame to maintain memory bounds while covering longer simulation
            self.captured_pil_frames.pop(1)
            self.captured_pil_frames.append(pil_img)

        return overlay_surf

    def save_keyframe_screenshot(self, surface, label, telemetry_info=None, bodies=None):
        """Saves a standalone high-resolution PNG screenshot with label"""
        overlay_surf = self.capture_frame(surface, telemetry_info=telemetry_info, draw_vectors=True, bodies=bodies)

        # Strip .png if passed in label
        clean_label = label[:-4] if label.endswith(".png") else label
        filename = f"{clean_label}.png"
        filepath = os.path.join(self.output_dir, filename)
        pygame.image.save(overlay_surf, filepath)
        self.saved_screenshots[clean_label] = filepath
        return filepath

    # Alias for convenience
    def save_screenshot(self, surface, label, telemetry_info=None, bodies=None):
        return self.save_keyframe_screenshot(surface, label, telemetry_info, bodies)

    def export_animated_gif(self, filename="simulation_replay.gif", fps=15, duration_ms=None):
        """Compiles the captured frames into an animated GIF"""
        if not self.captured_pil_frames:
            return None

        filepath = os.path.join(self.output_dir, filename)

        # Resize frames for web & report optimization (max 800px width)
        orig_w, orig_h = self.captured_pil_frames[0].size
        target_w = min(800, orig_w)
        target_h = int(orig_h * (target_w / float(orig_w)))

        optimized_frames = []
        for img in self.captured_pil_frames:
            resized = img.resize((target_w, target_h), Image.Resampling.BILINEAR)
            # Quantize to 128 colors for compact GIF size
            quantized = resized.quantize(colors=128)
            optimized_frames.append(quantized)

        frame_duration = duration_ms if duration_ms is not None else int(1000 / fps)
        optimized_frames[0].save(
            filepath,
            save_all=True,
            append_images=optimized_frames[1:],
            duration=frame_duration,
            loop=0,
            optimize=True
        )

        self.gif_path = filepath
        return filepath

    # Alias for convenience
    def compile_gif(self, filename="simulation_replay.gif", duration_ms=25, fps=15):
        return self.export_animated_gif(filename=filename, fps=fps, duration_ms=duration_ms)

    def get_gif_path(self):
        return self.gif_path
