#!/usr/bin/env python3
"""
LLM-GUI.py - OpenRouter / DeepSeek copilot for the CyBot.

A new version of Final-GUI-V2.py that:
  * keeps the CyBot firmware untouched (same newline-delimited protocol),
  * lets an LLM drive the robot through tool calls,
  * shows the model's reasoning, tool calls, route and corrected pose,
  * runs without hardware in a built-in simulator,
  * corrects poor encoder dead-reckoning by matching every scan against the
    obstacle map it has built so far.

Requirements
------------
Python 3.10+ with Tkinter (standard on Windows).
pyserial is only needed for a UART connection:  py -m pip install pyserial

Environment
-----------
OPENROUTER_API_KEY   API key (can also be pasted into Settings at runtime)
OPENROUTER_MODEL     default: deepseek/deepseek-v4.1-flash-1017
OPENROUTER_BASE_URL  default: https://openrouter.ai/api/v1

Run
---
    py LLM-GUI.py                 # starts in simulated mode
    py LLM-GUI.py --tcp           # pre-fills the TCP fields (192.168.1.1:288)
    py LLM-GUI.py --serial COM3   # pre-fills a UART connection

Protocol facts taken from finalProgram.c / movement.c / dataClean.c:
  m <cm> / t <deg> / s / p / o <n> / f / z, each answered by lines ending in
  "d".  A blocked move answers "b <cm remaining>", "h ..." or "w ..." first.
  Positive turn values rotate clockwise; the GUI exposes CCW-positive turns.
"""

from __future__ import annotations

import argparse
import heapq
import http.client
import json
import math
import os
import queue
import random
import socket
import statistics
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional
from urllib.parse import urlsplit

import tkinter as tk
from tkinter import ttk

try:
    import serial  # type: ignore
except ImportError:  # pyserial is optional; TCP and simulator still work
    serial = None


APP_TITLE = "CyBot LLM Copilot"
DEFAULT_MODEL = "deepseek/deepseek-v4.1-flash-1017"
DEFAULT_BASE_URL = "https://openrouter.ai/api/v1"
DEFAULT_HOST = "192.168.1.1"
DEFAULT_PORT = 288
DEFAULT_BAUD = 115200

SONG_RANGO = 0
SONG_FUNKYTOWN = 1
SONG_NAMES = {SONG_RANGO: "Rango", SONG_FUNKYTOWN: "Funkytown"}

ROBOT_RADIUS = 0.19          # m, CyBot / Create 2
SERVO_RADIUS = 0.14          # m, ping/servo sits ahead of the wheel centre
FIELD_HALF = 3.0             # m, mapped square is +/- 3 m
GRID_SPACING = 0.24          # m, A* lattice pitch
MAP_BOUND = 600              # px
PPM = MAP_BOUND / (2 * FIELD_HALF)  # pixels per metre
SCAN_RANGE_M = 0.65          # firmware ignores IR returns beyond ~63 cm
MAX_MOVE_CM = 500.0
MAX_TURN_DEG = 360.0

C = {
    "bg": "#0b0e14",
    "panel": "#111722",
    "panel2": "#0e131c",
    "line": "#223047",
    "grid": "#172032",
    "grid_major": "#1e2b42",
    "text": "#dbe5f1",
    "dim": "#7f8fa6",
    "faint": "#4b5a70",
    "accent": "#4cc9f0",
    "accent2": "#7ae582",
    "red": "#ef476f",
    "orange": "#f4a261",
    "yellow": "#ffd166",
    "route": "#4cc9f0",
    "trail": "#33506e",
    "obstacle": "#e76f51",
    "hole": "#ff2e63",
    "goal": "#7ae582",
    "robot": "#ff5d73",
    "tool": "#ffd166",
}

FONT_UI = ("Segoe UI", 9)
FONT_UI_BOLD = ("Segoe UI Semibold", 9)
FONT_MONO = ("Consolas", 9)
FONT_SMALL = ("Segoe UI", 8)


# --------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------

def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def wrap180(angle: float) -> float:
    """Fold an angle into (-180, 180]."""
    a = math.fmod(angle + 180.0, 360.0)
    if a <= 0:
        a += 360.0
    return a - 180.0


def wrap360(angle: float) -> float:
    return angle % 360.0


def unit(angle_deg: float) -> tuple[float, float]:
    r = math.radians(angle_deg)
    return math.cos(r), math.sin(r)


def segment_point_distance(ax, ay, bx, by, px, py) -> float:
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    length2 = vx * vx + vy * vy
    t = 0.0 if length2 == 0 else clamp((wx * vx + wy * vy) / length2, 0.0, 1.0)
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def ray_circle(ox, oy, dx, dy, cx, cy, radius) -> Optional[float]:
    """Distance from ray origin to the near intersection with a circle."""
    fx, fy = ox - cx, oy - cy
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return None
    root = math.sqrt(disc)
    for t in ((-b - root) / 2.0, (-b + root) / 2.0):
        if t >= 0.0:
            return t
    return None


def circular_median(values: list[float]) -> float:
    if not values:
        return 0.0
    base = values[0]
    unfolded = [wrap180(v - base) for v in values]
    return wrap180(base + statistics.median(unfolded))


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    heading: float = 90.0  # degrees CCW from +x; 90 = north/up

    def copy(self) -> "Pose":
        return Pose(self.x, self.y, self.heading)

    def xy(self) -> tuple[float, float]:
        return (self.x, self.y)


@dataclass
class Obstacle:
    x: float
    y: float
    r: float
    kind: str = "obstacle"  # obstacle | bump | hole | boundary
    hits: int = 1
    id: int = 0
    added_turn: int = 0
    updated_turn: int = 0


@dataclass
class Detection:
    angle_deg: float
    distance_cm: float
    size_cm: float

    @property
    def distance_m(self) -> float:
        return self.distance_cm / 100.0

    @property
    def radius_m(self) -> float:
        return clamp(self.size_cm / 200.0, 0.03, 0.32)

    def as_dict(self) -> dict[str, float]:
        return {
            "angle_deg": round(self.angle_deg, 1),
            "distance_cm": round(self.distance_cm, 1),
            "size_cm": round(self.size_cm, 1),
        }


def detections_to_landmarks(detections: list[Detection],
                            pose: Pose) -> list[Obstacle]:
    """Project cleaned scan objects into world coordinates for re-localization."""
    sx = pose.x + SERVO_RADIUS * math.cos(math.radians(pose.heading))
    sy = pose.y + SERVO_RADIUS * math.sin(math.radians(pose.heading))
    landmarks: list[Obstacle] = []
    for det in detections:
        if not (0.04 <= det.distance_m <= 2.0):
            continue
        angle = pose.heading + det.angle_deg - 90.0
        r = det.radius_m
        centre = det.distance_m + r
        ox = sx + centre * math.cos(math.radians(angle))
        oy = sy + centre * math.sin(math.radians(angle))
        if abs(ox) > FIELD_HALF or abs(oy) > FIELD_HALF:
            continue
        landmarks.append(Obstacle(ox, oy, r, "obstacle"))
    return landmarks


@dataclass
class Correction:
    matched: int = 0
    inliers: int = 0
    heading_delta: float = 0.0
    dx: float = 0.0
    dy: float = 0.0
    applied: bool = False
    note: str = ""
    source: str = ""

    def summary(self) -> str:
        if not self.applied:
            return self.note or "no correction"
        return (
            f"{self.inliers}/{self.matched} {self.source or 'landmarks'}, "
            f"dtheta {self.heading_delta:+.1f} deg, "
            f"dpos ({self.dx * 100:+.1f}, {self.dy * 100:+.1f}) cm"
        )

    def as_dict(self) -> dict[str, Any]:
        return {
            "applied": self.applied,
            "matched": self.matched,
            "inliers": self.inliers,
            "heading_delta_deg": round(self.heading_delta, 2),
            "dx_cm": round(self.dx * 100.0, 2),
            "dy_cm": round(self.dy * 100.0, 2),
            "note": self.note,
            "source": self.source,
        }


def _robot_frame_points(detections: list[Detection]) -> list[tuple[float, float, float]]:
    """Detection object centres in the robot frame (x forward, y left)."""
    points: list[tuple[float, float, float]] = []
    for det in detections:
        a = math.radians(det.angle_deg - 90.0)
        dist = det.distance_m + det.radius_m
        points.append((
            SERVO_RADIUS + dist * math.cos(a),
            dist * math.sin(a),
            det.size_cm,
        ))
    return points


def estimate_turn_from_scans(previous: list[Detection],
                             current: list[Detection],
                             expected_ccw: float) -> tuple[Optional[float], int, float]:
    """Estimate the actual in-place rotation between two scans.

    Only valid when the robot turned without translating.  Points are matched
    in the robot frame, so the 14 cm servo offset does not bias the angle.
    Returns (actual_ccw_deg, inliers, score) or (None, ...) when ambiguous.
    """
    if len(previous) < 2 or len(current) < 2:
        return None, 0, 0.0
    prev_pts = _robot_frame_points(previous)
    cur_pts = _robot_frame_points(current)
    candidates: list[tuple[float, float, int]] = []
    for step in range(-300, 301):
        turn = expected_ccw + step * 0.05
        ca = math.cos(math.radians(-turn))
        sa = math.sin(math.radians(-turn))
        used: set[int] = set()
        errors: list[float] = []
        for px, py, psize in prev_pts:
            rx = px * ca - py * sa
            ry = px * sa + py * ca
            best_j = None
            best_cost = 1e9
            for j, (cx, cy, csize) in enumerate(cur_pts):
                if j in used:
                    continue
                dist = math.hypot(cx - rx, cy - ry)
                size_err = abs(csize - psize)
                if dist < 0.06 and size_err < 8.0:
                    cost = dist / 0.06 + size_err / 8.0
                    if cost < best_cost:
                        best_cost = cost
                        best_j = j
            if best_j is not None:
                used.add(best_j)
                errors.append(best_cost)
        if len(errors) < 2:
            continue
        score = len(errors) - (sum(errors) / len(errors)) * 0.25
        candidates.append((score, turn, len(errors)))
    if not candidates:
        return None, 0, 0.0
    best = max(candidates, key=lambda c: c[0])
    # Reject only a solution pinned to the edge of the search window.  Scenes
    # with repeated geometry can produce several equally good rotations; the
    # best one is still the one closest to the physical expectation.
    if best[2] >= 2 and abs(best[1] - expected_ccw) < 14.5:
        return best[1], best[2], best[0]
    return None, best[2], best[0]


@dataclass
class Route:
    points: list[tuple[float, float]] = field(default_factory=list)
    distance_m: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        return {
            "waypoints": [[round(x, 3), round(y, 3)] for x, y in self.points],
            "distance_m": round(self.distance_m, 3),
        }


@dataclass
class AppConfig:
    api_key: str = ""
    model: str = DEFAULT_MODEL
    base_url: str = DEFAULT_BASE_URL
    temperature: float = 0.2
    reasoning_effort: str = "default"
    max_agent_steps: int = 10
    scan_correction: bool = True
    correction_every: int = 3      # scan every N path waypoints; 0 disables
    move_scale: float = 1.0        # actual distance / commanded distance
    turn_scale: float = 1.0        # actual angle / commanded angle
    show_reasoning: bool = True
    show_grid_nodes: bool = False
    show_scan_rays: bool = True
    sim_drift: bool = True


# --------------------------------------------------------------------------
# Map, A*, and scan-based localization
# --------------------------------------------------------------------------

class MapModel:
    """Obstacle memory plus an A* lattice."""

    def __init__(self) -> None:
        self.obstacles: list[Obstacle] = []
        self._next_id = 1
        self.nodes: list[tuple[float, float]] = []
        self.ij: list[tuple[int, int]] = []
        self.node_index: dict[tuple[int, int], int] = {}
        self.n = int((2.0 * FIELD_HALF) / GRID_SPACING)
        for iy in range(self.n):
            y = -FIELD_HALF + GRID_SPACING / 2.0 + iy * GRID_SPACING
            for ix in range(self.n):
                x = -FIELD_HALF + GRID_SPACING / 2.0 + ix * GRID_SPACING
                self.node_index[(ix, iy)] = len(self.nodes)
                self.ij.append((ix, iy))
                self.nodes.append((x, y))

    # -- obstacle memory ---------------------------------------------------

    def clear(self) -> None:
        self.obstacles.clear()
        self._next_id = 1

    def _merge(self, new: Obstacle, average: bool = False,
               turn: int = 0) -> bool:
        """Merge into a nearby object; returns True when a new object was added."""
        for o in self.obstacles:
            if o.kind != new.kind:
                continue
            if math.hypot(o.x - new.x, o.y - new.y) < max(0.12, 0.45 * (o.r + new.r)):
                w = 1.0 / (o.hits + 1)
                o.x = o.x * (1.0 - w) + new.x * w
                o.y = o.y * (1.0 - w) + new.y * w
                if average:
                    o.r = clamp(o.r * (1.0 - w) + new.r * w, 0.03, 0.32)
                else:
                    o.r = max(o.r, new.r)
                o.hits += 1
                o.updated_turn = max(o.updated_turn, turn)
                return False
        new.id = self._next_id
        self._next_id += 1
        new.added_turn = turn
        new.updated_turn = turn
        self.obstacles.append(new)
        return True

    def _near_contact(self, landmark: Obstacle) -> bool:
        """Do not duplicate a physical object already recorded by contact."""
        for o in self.obstacles:
            if o.kind == "obstacle":
                continue
            if math.hypot(o.x - landmark.x, o.y - landmark.y) < o.r + landmark.r + 0.06:
                return True
        return False

    def _add_landmarks(self, landmarks: list[Obstacle], turn: int,
                       average: bool) -> int:
        added = 0
        for landmark in landmarks:
            if self._near_contact(landmark):
                continue
            if self._merge(landmark, average=average, turn=turn):
                added += 1
        return added

    def add_detections(self, detections: list[Detection], pose: Pose,
                       turn: int = 0, average: bool = False) -> int:
        """Project cleaned scan objects into the world using the current pose."""
        return self._add_landmarks(
            detections_to_landmarks(detections, pose), turn, average
        )

    def _in_scan_area(self, o: Obstacle, pose: Pose) -> bool:
        sx = pose.x + SERVO_RADIUS * math.cos(math.radians(pose.heading))
        sy = pose.y + SERVO_RADIUS * math.sin(math.radians(pose.heading))
        dx, dy = o.x - sx, o.y - sy
        dist = math.hypot(dx, dy)
        if dist - o.r > SCAN_RANGE_M + 0.18:
            return False
        bearing = math.degrees(math.atan2(dy, dx))
        return abs(wrap180(bearing - pose.heading)) <= 95.0

    def refresh_scan(self, pose: Pose, detections: list[Detection],
                     turn: int, average: bool = False) -> tuple[int, list[int]]:
        """Replace the scan-derived objects in the viewed sector.

        Contact objects (bump/hole/boundary) are never removed by a scan. Existing
        scan objects that are re-seen are merged (position/size averaged when the
        robot has not moved); objects in the sector that are not re-seen are
        deleted as stale. Returns (new_objects, removed_ids).
        """
        landmarks = detections_to_landmarks(detections, pose)
        removed: list[int] = []
        for o in list(self.obstacles):
            if o.kind != "obstacle" or not self._in_scan_area(o, pose):
                continue
            if any(math.hypot(o.x - lm.x, o.y - lm.y)
                   < max(0.12, 0.45 * (o.r + lm.r)) for lm in landmarks):
                continue
            removed.append(o.id)
            self.obstacles.remove(o)
        added = self._add_landmarks(landmarks, turn, average)
        return added, removed

    def remove_near(self, x: float, y: float, radius: float,
                    kinds: tuple[str, ...] = ("obstacle",)) -> list[int]:
        removed: list[int] = []
        for o in list(self.obstacles):
            if o.kind in kinds and math.hypot(o.x - x, o.y - y) <= radius:
                removed.append(o.id)
                self.obstacles.remove(o)
        return removed

    def remove_ids(self, ids: list[int]) -> tuple[list[int], list[int]]:
        wanted: list[int] = []
        for raw in ids:
            try:
                wanted.append(int(raw))
            except (TypeError, ValueError):
                continue
        removed: list[int] = []
        ignored: list[int] = []
        for oid in wanted:
            target = next((o for o in self.obstacles if o.id == oid), None)
            if target is None:
                ignored.append(oid)
            else:
                removed.append(oid)
                self.obstacles.remove(target)
        return removed, ignored

    def add_contact(self, pose: Pose, event: str, turn: int = 0) -> None:
        """A bump/hole/boundary contact is a known obstacle just ahead."""
        angle = pose.heading
        ox = pose.x + (ROBOT_RADIUS + 0.05) * math.cos(math.radians(angle))
        oy = pose.y + (ROBOT_RADIUS + 0.05) * math.sin(math.radians(angle))
        # A contact refreshes the immediate area; scan-derived ghosts and an
        # older contact of the same kind near the bumper are not trustworthy.
        kinds = ["obstacle", "bump"]
        if event == "h":
            kinds.append("hole")
        elif event == "w":
            kinds.append("boundary")
        self.remove_near(ox, oy, 0.40, tuple(kinds))
        if event == "h":
            self._merge(Obstacle(ox, oy, 0.10, "hole"), turn=turn)
        elif event == "w":
            self._merge(Obstacle(ox, oy, 0.12, "boundary"), turn=turn)
        else:
            self._merge(Obstacle(ox, oy, 0.07, "bump"), turn=turn)

    # -- planning ----------------------------------------------------------

    def _blocked_nodes(self) -> set[int]:
        blocked: set[int] = set()
        for i, (x, y) in enumerate(self.nodes):
            for o in self.obstacles:
                clearance = o.r + ROBOT_RADIUS + 0.02
                if o.kind == "hole":
                    clearance = o.r + ROBOT_RADIUS
                if math.hypot(x - o.x, y - o.y) < clearance:
                    blocked.add(i)
                    break
        return blocked

    def _line_clear(self, a: tuple[float, float], b: tuple[float, float]) -> bool:
        if max(abs(a[0]), abs(a[1]), abs(b[0]), abs(b[1])) > FIELD_HALF - 0.02:
            return False
        for o in self.obstacles:
            clearance = o.r + ROBOT_RADIUS * 0.80
            if o.kind == "hole":
                clearance = o.r + ROBOT_RADIUS * 0.60
            if segment_point_distance(a[0], a[1], b[0], b[1], o.x, o.y) < clearance:
                return False
        return True

    def _neighbours(self, idx: int) -> list[int]:
        ix, iy = self.ij[idx]
        out = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            j = self.node_index.get((ix + dx, iy + dy))
            if j is not None:
                out.append(j)
        return out

    def plan(self, start: tuple[float, float], goal: tuple[float, float]) -> Route:
        gx, gy = goal
        if abs(gx) > FIELD_HALF - 0.10 or abs(gy) > FIELD_HALF - 0.10:
            return Route()

        blocked = self._blocked_nodes()

        def nearest_free(x: float, y: float) -> Optional[int]:
            best, best_d = None, 1e9
            for i, (nx, ny) in enumerate(self.nodes):
                if i in blocked:
                    continue
                d = (nx - x) ** 2 + (ny - y) ** 2
                if d < best_d:
                    best, best_d = i, d
            return best

        if self._line_clear(start, goal):
            return Route([goal], math.hypot(goal[0] - start[0], goal[1] - start[1]))

        start_i = nearest_free(*start)
        goal_i = nearest_free(*goal)
        if start_i is None or goal_i is None:
            return Route()

        frontier: list[tuple[float, int, int]] = []
        counter = 0
        heapq.heappush(frontier, (0.0, counter, start_i))
        came: dict[int, int] = {}
        gscore = {start_i: 0.0}
        reached = False

        while frontier:
            _, _, cur = heapq.heappop(frontier)
            if cur == goal_i:
                reached = True
                break
            cx, cy = self.nodes[cur]
            for nb in self._neighbours(cur):
                if nb in blocked:
                    continue
                nx, ny = self.nodes[nb]
                if not self._line_clear((cx, cy), (nx, ny)):
                    continue
                ng = gscore[cur] + math.hypot(nx - cx, ny - cy)
                if ng < gscore.get(nb, 1e9):
                    gscore[nb] = ng
                    came[nb] = cur
                    h = math.hypot(nx - gx, ny - gy)
                    counter += 1
                    heapq.heappush(frontier, (ng + h, counter, nb))

        if not reached:
            return Route()

        chain = [goal_i]
        while chain[-1] != start_i:
            chain.append(came[chain[-1]])
        chain.reverse()
        points = [self.nodes[i] for i in chain]
        points[0] = start

        # String-pull the lattice path into longer visible segments.
        simplified = [points[0]]
        i = 0
        while i < len(points) - 1:
            j = len(points) - 1
            while j > i + 1 and not self._line_clear(points[i], points[j]):
                j -= 1
            simplified.append(points[j])
            i = j
        targets = simplified[1:]
        if targets and self._line_clear(targets[-1], goal):
            if targets[-1] != goal:
                targets.append(goal)
        elif not targets:
            return Route()
        distance = 0.0
        prev = start
        for p in targets:
            distance += math.hypot(p[0] - prev[0], p[1] - prev[1])
            prev = p
        return Route(targets, distance)

    def snapshot(self, limit: int = 60,
                 current_turn: int = 0) -> list[dict[str, Any]]:
        items = sorted(self.obstacles, key=lambda o: o.id)[:limit]
        return [
            {"id": o.id, "x": round(o.x, 2), "y": round(o.y, 2),
             "r": round(o.r, 2), "kind": o.kind, "hits": o.hits,
             "added_turn": o.added_turn,
             "updated_turn": o.updated_turn,
             "age_turns": max(0, current_turn - o.added_turn)}
            for o in items
        ]


class ScanLocalizer:
    """Correct encoder drift by matching a scan to the obstacle map.

    Heading comes from the robust median bearing residual.  Translation comes
    from a 2-D least-squares fit of range residuals.  Both are applied with a
    conservative gain because mapped obstacle centres are themselves noisy.
    """

    def __init__(self, heading_gain: float = 0.50, position_gain: float = 0.50) -> None:
        self.heading_gain = heading_gain
        self.position_gain = position_gain

    def _sensor(self, pose: Pose) -> tuple[float, float]:
        return (
            pose.x + SERVO_RADIUS * math.cos(math.radians(pose.heading)),
            pose.y + SERVO_RADIUS * math.sin(math.radians(pose.heading)),
        )

    def _match(self, pose: Pose, detections: list[Detection],
               obstacles: list[Obstacle]) -> list[tuple[Detection, Obstacle]]:
        sx, sy = self._sensor(pose)
        candidates: list[tuple[float, int, int]] = []
        for di, det in enumerate(detections):
            for oi, o in enumerate(obstacles):
                dx, dy = o.x - sx, o.y - sy
                dist = math.hypot(dx, dy)
                if dist < 0.03:
                    continue
                pred_surface = dist - o.r
                if pred_surface > 2.0:
                    continue
                pred_angle = math.degrees(math.atan2(dy, dx)) - pose.heading + 90.0
                angle_err = abs(wrap180(pred_angle - det.angle_deg))
                range_gate = max(0.16, 0.40 * max(pred_surface, 0.10))
                range_err = abs(pred_surface - det.distance_m)
                if angle_err <= 12.0 and range_err <= range_gate:
                    cost = angle_err / 12.0 + range_err / range_gate
                    candidates.append((cost, di, oi))
        candidates.sort(key=lambda c: c[0])
        used_d: set[int] = set()
        used_o: set[int] = set()
        matches: list[tuple[Detection, Obstacle]] = []
        for _, di, oi in candidates:
            if di in used_d or oi in used_o:
                continue
            used_d.add(di)
            used_o.add(oi)
            matches.append((detections[di], obstacles[oi]))
        return matches

    def correct(self, pose: Pose, detections: list[Detection],
                landmarks: list[Obstacle]) -> tuple[Pose, Correction]:
        corr = Correction()
        obstacles = [o for o in landmarks
                     if o.kind in ("obstacle", "bump", "boundary")]
        if len(obstacles) < 2 or not detections:
            corr.note = "not enough mapped landmarks"
            return pose.copy(), corr

        p = pose.copy()
        matches = self._match(p, detections, obstacles)
        corr.matched = len(matches)
        if len(matches) < 2:
            corr.note = "no landmark matches"
            return p, corr

        for _ in range(2):
            # --- heading: measured world bearing vs map-predicted bearing ---
            sx, sy = self._sensor(p)
            offsets = []
            for det, o in matches:
                predicted = math.degrees(math.atan2(o.y - sy, o.x - sx))
                measured = p.heading + det.angle_deg - 90.0
                offsets.append(wrap180(measured - predicted))
            med = circular_median(offsets)
            mad = statistics.median([abs(wrap180(v - med)) for v in offsets])
            if len(offsets) >= 2 and mad <= 20.0 and abs(med) <= 60.0:
                delta = -med * self.heading_gain
                p.heading = wrap360(p.heading + delta)
                corr.heading_delta += delta
                corr.inliers = max(corr.inliers, len(offsets))

            # --- translation: range residuals along sensor->landmark rays ---
            sx, sy = self._sensor(p)
            rows: list[tuple[float, float, float]] = []
            for det, o in matches:
                dx, dy = o.x - sx, o.y - sy
                centre_dist = math.hypot(dx, dy)
                if centre_dist < o.r + 0.03 or centre_dist > 2.0:
                    continue
                predicted_range = centre_dist - o.r
                residual = det.distance_m - predicted_range
                if abs(residual) > 0.35:
                    continue
                ux, uy = dx / centre_dist, dy / centre_dist
                rows.append((ux, uy, residual))
            if len(rows) >= 2:
                med_e = statistics.median(r[2] for r in rows)
                rows = [r for r in rows if abs(r[2] - med_e) < 0.20]
                if len(rows) >= 2:
                    a11 = sum(r[0] * r[0] for r in rows)
                    a12 = sum(r[0] * r[1] for r in rows)
                    a22 = sum(r[1] * r[1] for r in rows)
                    b1 = -sum(r[0] * r[2] for r in rows)
                    b2 = -sum(r[1] * r[2] for r in rows)
                    det_a = a11 * a22 - a12 * a12
                    if abs(det_a) > 1e-5:
                        ddx = (a22 * b1 - a12 * b2) / det_a
                        ddy = (-a12 * b1 + a11 * b2) / det_a
                        mag = math.hypot(ddx, ddy)
                        if mag > 0.5:
                            ddx *= 0.5 / mag
                            ddy *= 0.5 / mag
                        p.x += self.position_gain * ddx
                        p.y += self.position_gain * ddy
                        corr.dx += self.position_gain * ddx
                        corr.dy += self.position_gain * ddy
                        corr.inliers = max(corr.inliers, len(rows))

        corr.applied = (
            abs(corr.heading_delta) > 0.05
            or math.hypot(corr.dx, corr.dy) > 0.002
        )
        return p, corr


# --------------------------------------------------------------------------
# Robot state
# --------------------------------------------------------------------------

class RobotState:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.pose = Pose()
        self.map = MapModel()
        self.goal: Optional[tuple[float, float]] = None
        self.route = Route()
        self.trail: list[tuple[float, float]] = [(0.0, 0.0)]
        self.last_scan: list[Detection] = []
        self.last_scan_landmarks: list[Obstacle] = []
        self.last_scan_pose: Optional[Pose] = None
        self.last_scan_turn: int = -1
        self.last_correction = Correction()
        self.last_event: Optional[str] = None
        self.connected = False
        self.mode = "disconnected"

    def reset(self) -> None:
        with self.lock:
            self.pose = Pose()
            self.map = MapModel()
            self.goal = None
            self.route = Route()
            self.trail = [(0.0, 0.0)]
            self.last_scan = []
            self.last_scan_landmarks = []
            self.last_scan_pose = None
            self.last_scan_turn = -1
            self.last_correction = Correction()
            self.last_event = None

    def add_trail_point(self) -> None:
        with self.lock:
            if not self.trail or math.hypot(
                self.trail[-1][0] - self.pose.x,
                self.trail[-1][1] - self.pose.y,
            ) > 0.03:
                self.trail.append((self.pose.x, self.pose.y))
                if len(self.trail) > 1200:
                    del self.trail[:200]

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "pose": (self.pose.x, self.pose.y, self.pose.heading),
                "goal": self.goal,
                "route": list(self.route.points),
                "route_distance": self.route.distance_m,
                "obstacles": [(o.x, o.y, o.r, o.kind) for o in self.map.obstacles],
                "last_scan": list(self.last_scan),
                "last_correction": self.last_correction,
                "last_event": self.last_event,
                "connected": self.connected,
                "mode": self.mode,
                "trail": list(self.trail),
            }


# --------------------------------------------------------------------------
# Links: TCP, serial, simulator
# --------------------------------------------------------------------------

class CybotLinkError(RuntimeError):
    pass


class TcpLink:
    mode = "tcp"

    def __init__(self, host: str, port: int,
                 on_log: Optional[Callable[[str, str], None]] = None,
                 read_timeout: float = 90.0) -> None:
        self.host = host
        self.port = port
        self.on_log = on_log
        self.read_timeout = read_timeout
        self.lock = threading.RLock()
        self.sock: Optional[socket.socket] = None
        self.file = None
        self.connected = False

    def connect(self) -> None:
        self.sock = socket.create_connection((self.host, self.port), timeout=5.0)
        self.sock.settimeout(self.read_timeout)
        self.file = self.sock.makefile("rb", buffering=0)
        self.connected = True

    def send(self, command: str) -> str:
        with self.lock:
            if not self.connected or self.sock is None or self.file is None:
                raise CybotLinkError("not connected")
            cmd = command.strip()
            if not cmd:
                return "d"
            if self.on_log:
                self.on_log(">>", cmd)
            try:
                self.sock.sendall((cmd + "\n").encode("ascii", "replace"))
                lines: list[str] = []
                while True:
                    raw = self.file.readline()
                    if not raw:
                        raise CybotLinkError("CyBot closed the connection")
                    line = raw.decode("utf-8", "replace").strip()
                    if line:
                        lines.append(line)
                    if line == "d":
                        break
                text = "\n".join(lines)
                if self.on_log:
                    self.on_log("<<", text)
                return text
            except socket.timeout as exc:
                self.close()
                raise CybotLinkError(
                    f"timed out after {self.read_timeout:.0f}s waiting for 'd'"
                ) from exc
            except OSError as exc:
                self.close()
                raise CybotLinkError(str(exc)) from exc

    def close(self) -> None:
        with self.lock:
            self.connected = False
            try:
                if self.file is not None:
                    self.file.close()
            except OSError:
                pass
            try:
                if self.sock is not None:
                    self.sock.close()
            except OSError:
                pass
            self.file = None
            self.sock = None


class SerialLink:
    mode = "serial"

    def __init__(self, port: str, baud: int,
                 on_log: Optional[Callable[[str, str], None]] = None,
                 read_timeout: float = 90.0) -> None:
        self.port = port
        self.baud = baud
        self.on_log = on_log
        self.read_timeout = read_timeout
        self.lock = threading.RLock()
        self.ser = None
        self.connected = False

    def connect(self) -> None:
        if serial is None:
            raise CybotLinkError(
                "pyserial is not installed: py -m pip install pyserial"
            )
        self.ser = serial.Serial(self.port, self.baud, timeout=self.read_timeout)
        self.ser.reset_input_buffer()
        self.connected = True

    def send(self, command: str) -> str:
        with self.lock:
            if not self.connected or self.ser is None:
                raise CybotLinkError("not connected")
            cmd = command.strip()
            if not cmd:
                return "d"
            if self.on_log:
                self.on_log(">>", cmd)
            try:
                self.ser.write((cmd + "\n").encode("ascii", "replace"))
                lines: list[str] = []
                while True:
                    raw = self.ser.readline()
                    if not raw:
                        raise CybotLinkError("serial read timed out")
                    line = raw.decode("utf-8", "replace").strip()
                    if line:
                        lines.append(line)
                    if line == "d":
                        break
                text = "\n".join(lines)
                if self.on_log:
                    self.on_log("<<", text)
                return text
            except Exception as exc:
                self.close()
                raise CybotLinkError(str(exc)) from exc

    def close(self) -> None:
        with self.lock:
            self.connected = False
            try:
                if self.ser is not None:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None


class SimWorld:
    """Ground truth for the built-in demo.  The GUI only ever sees scans."""

    def __init__(self) -> None:
        self.bounds = FIELD_HALF
        self.obstacles = [
            # Asymmetric near-field landmarks so scan matching has something to
            # lock onto; the real room is rarely a perfectly symmetric ring.
            Obstacle(0.30, 0.32, 0.055, "obstacle"),
            Obstacle(-0.36, 0.30, 0.095, "obstacle"),
            Obstacle(0.12, -0.40, 0.075, "obstacle"),
            Obstacle(-0.32, -0.26, 0.055, "obstacle"),
            Obstacle(0.62, -0.10, 0.110, "obstacle"),
            Obstacle(1.45, 1.05, 0.24, "obstacle"),
            Obstacle(-1.55, 1.55, 0.30, "obstacle"),
            Obstacle(1.9, -1.35, 0.26, "obstacle"),
            Obstacle(-1.35, -1.75, 0.22, "hole"),
            Obstacle(2.25, 1.95, 0.20, "obstacle"),
        ]

    def collision(self, x: float, y: float) -> Optional[str]:
        if abs(x) > self.bounds - ROBOT_RADIUS * 0.8 or abs(y) > self.bounds - ROBOT_RADIUS * 0.8:
            return "w"
        for o in self.obstacles:
            if o.kind == "hole":
                if math.hypot(x - o.x, y - o.y) < o.r + ROBOT_RADIUS * 0.35:
                    return "h"
            elif math.hypot(x - o.x, y - o.y) < o.r + ROBOT_RADIUS * 0.85:
                return "b"
        return None

    def advance(self, pose: Pose, distance_cm: float) -> tuple[Optional[str], float]:
        """Move truth pose, stopping at a contact. Returns (event, travelled cm)."""
        remaining = distance_cm
        travelled = 0.0
        step = 1.0 if distance_cm >= 0 else -1.0
        ux, uy = unit(pose.heading)
        while abs(remaining) >= 0.5:
            nx = pose.x + ux * step / 100.0
            ny = pose.y + uy * step / 100.0
            event = self.collision(nx, ny)
            if event and distance_cm > 0:
                return event, travelled
            pose.x, pose.y = nx, ny
            travelled += step
            remaining -= step
        if abs(remaining) > 0:
            pose.x += ux * remaining / 100.0
            pose.y += uy * remaining / 100.0
            travelled += remaining
        return None, travelled

    def ray_scan(self, pose: Pose) -> list[Detection]:
        sx = pose.x + SERVO_RADIUS * math.cos(math.radians(pose.heading))
        sy = pose.y + SERVO_RADIUS * math.sin(math.radians(pose.heading))
        hits: list[tuple[int, float, Obstacle]] = []
        for angle in range(0, 181, 2):
            world_angle = pose.heading + angle - 90.0
            dx, dy = unit(world_angle)
            best_t, best_o = None, None
            for o in self.obstacles:
                if o.kind == "hole":
                    continue
                t = ray_circle(sx, sy, dx, dy, o.x, o.y, o.r)
                if t is not None and t <= SCAN_RANGE_M and (best_t is None or t < best_t):
                    best_t, best_o = t, o
            if best_o is not None and best_t is not None:
                hits.append((angle, best_t, best_o))

        detections: list[Detection] = []
        groups: list[list[tuple[int, float, Obstacle]]] = []
        for hit in hits:
            if groups:
                prev = groups[-1][-1]
                if hit[2] is prev[2] and hit[0] - prev[0] <= 2 and abs(hit[1] - prev[1]) < 0.08:
                    groups[-1].append(hit)
                    continue
            groups.append([hit])
        for group in groups:
            if len(group) < 2:
                continue
            a0, a1 = group[0][0], group[-1][0]
            mid = (a0 + a1) / 2.0
            world_angle = pose.heading + mid - 90.0
            dx, dy = unit(world_angle)
            o = group[0][2]
            t = ray_circle(sx, sy, dx, dy, o.x, o.y, o.r)
            if t is None:
                continue
            dist = t + random.uniform(-0.006, 0.006)
            width = 2.0 * math.tan(math.radians((a1 - a0) / 2.0)) * dist
            if width < 0.03:
                continue
            detections.append(Detection(mid, max(dist, 0.02) * 100.0, width * 100.0))
        return detections

    def ping(self, pose: Pose) -> float:
        sx = pose.x + SERVO_RADIUS * math.cos(math.radians(pose.heading))
        sy = pose.y + SERVO_RADIUS * math.sin(math.radians(pose.heading))
        dx, dy = unit(pose.heading)
        best = 3.0
        for o in self.obstacles:
            if o.kind == "hole":
                continue
            t = ray_circle(sx, sy, dx, dy, o.x, o.y, o.r)
            if t is not None:
                best = min(best, t)
        return best


class SimLink:
    """Speaks the same protocol as the CyBot; optionally models encoder drift."""

    mode = "simulated"

    def __init__(self, world: Optional[SimWorld] = None, drift: bool = True,
                 on_log: Optional[Callable[[str, str], None]] = None) -> None:
        self.world = world or SimWorld()
        self.truth = Pose()
        self.on_log = on_log
        self.lock = threading.RLock()
        self.connected = True
        self.rng = random.Random(7)
        self.move_scale = 1.035 if drift else 1.0
        self.turn_scale = 1.045 if drift else 1.0
        self.move_sigma = 0.35 if drift else 0.0
        self.turn_sigma = 0.65 if drift else 0.0
        self.speed_cm_s = 38.0
        self.turn_deg_s = 70.0

    def send(self, command: str) -> str:
        with self.lock:
            cmd = command.strip()
            if self.on_log:
                self.on_log(">>", cmd)
            lines = self._handle(cmd)
            text = "\n".join(lines)
            if self.on_log:
                self.on_log("<<", text)
            return text

    def _handle(self, cmd: str) -> list[str]:
        if not cmd:
            return ["d"]
        head = cmd.split(None, 1)
        code = head[0][0]
        arg = head[1] if len(head) > 1 else ""

        if code == "m":
            try:
                requested = float(arg)
            except ValueError:
                requested = 0.0
            actual = requested * self.move_scale + self.rng.gauss(0.0, self.move_sigma)
            event = None
            if actual >= 0:
                event, travelled = self.world.advance(self.truth, actual)
            else:
                travelled = actual
                ux, uy = unit(self.truth.heading)
                self.truth.x += ux * actual / 100.0
                self.truth.y += uy * actual / 100.0
            time.sleep(min(abs(actual) / self.speed_cm_s, 4.0))
            if event:
                requested_travelled = travelled / self.move_scale
                remaining = requested - requested_travelled
                return [f"{event} {remaining:.3f}", "d"]
            return ["d"]

        if code == "t":
            try:
                requested = float(arg)
            except ValueError:
                requested = 0.0
            actual = requested * self.turn_scale + self.rng.gauss(0.0, self.turn_sigma)
            self.truth.heading = wrap360(self.truth.heading - actual)
            time.sleep(min(abs(actual) / self.turn_deg_s, 3.0))
            return ["d"]

        if code == "s":
            time.sleep(0.55)
            dets = self.world.ray_scan(self.truth)
            return [f"{d.angle_deg:.0f} {d.distance_cm:.3f} {d.size_cm:.3f}" for d in dets] + ["d"]

        if code == "p":
            time.sleep(0.15)
            return [f"{self.world.ping(self.truth) * 100.0:.3f}", "d"]

        if code == "o":
            try:
                index = int(float(arg))
            except ValueError:
                index = 0
            if index == SONG_RANGO:
                time.sleep(2.2)
            elif index == SONG_FUNKYTOWN:
                time.sleep(3.0)
            else:
                time.sleep(1.0)
            return ["d"]

        if code == "f":
            return ["f", "d"]

        return ["d"]

    def close(self) -> None:
        self.connected = False

    def truth_pose(self) -> Pose:
        with self.lock:
            return self.truth.copy()


# --------------------------------------------------------------------------
# Controller: protocol -> belief updates
# --------------------------------------------------------------------------

class CybotController:
    def __init__(self, link: Any, state: RobotState, config: AppConfig,
                 emit: Optional[Callable[..., None]] = None,
                 stop_event: Optional[threading.Event] = None) -> None:
        self.link = link
        self.state = state
        self.config = config
        self.emit = emit
        self.stop_event = stop_event
        self.cmd_lock = threading.RLock()
        self.localizer = ScanLocalizer()
        self.last_turn: Optional[dict[str, float]] = None
        self.turn_count = 0
        state.connected = True
        state.mode = getattr(link, "mode", "unknown")

    # -- helpers -----------------------------------------------------------

    def _post(self, kind: str, **data: Any) -> None:
        if self.emit:
            self.emit(kind, **data)

    def _dirty(self) -> None:
        self._post("map_dirty")

    def close(self) -> None:
        try:
            self.link.close()
        finally:
            with self.state.lock:
                self.state.connected = False
                self.state.mode = "disconnected"

    def _event_of(self, response: str) -> tuple[Optional[str], Optional[float]]:
        for line in response.splitlines():
            parts = line.split()
            if parts and parts[0] in ("b", "h", "w"):
                try:
                    return parts[0], float(parts[1])
                except (IndexError, ValueError):
                    return parts[0], None
        return None, None

    def _pose(self) -> Pose:
        with self.state.lock:
            return self.state.pose.copy()

    # -- commands ----------------------------------------------------------

    def move_cm(self, distance_cm: float) -> dict[str, Any]:
        if not math.isfinite(distance_cm):
            return {"ok": False, "error": "distance must be finite"}
        if abs(distance_cm) > MAX_MOVE_CM:
            return {"ok": False, "error": f"distance exceeds {MAX_MOVE_CM} cm"}
        if abs(distance_cm) < 0.05:
            return {"ok": True, "requested_cm": 0.0, "travelled_cm": 0.0,
                    "event": None, "response": "d"}

        with self.cmd_lock:
            self.last_turn = None
            pose = self._pose()
            scale = self.config.move_scale if self.config.move_scale > 0 else 1.0
            sent_cm = distance_cm / scale
            command = f"m {sent_cm:.1f}"
            try:
                response = self.link.send(command)
            except CybotLinkError as exc:
                self._link_failed(str(exc))
                return {"ok": False, "error": str(exc)}

            self.turn_count += 1
            event, remaining = self._event_of(response)
            if event is not None and remaining is not None and distance_cm > 0:
                travelled = (sent_cm - remaining) * scale
            else:
                travelled = distance_cm
            travelled = clamp(travelled, min(0.0, distance_cm), max(0.0, distance_cm))

            ux, uy = unit(pose.heading)
            with self.state.lock:
                self.state.pose.x += ux * travelled / 100.0
                self.state.pose.y += uy * travelled / 100.0
                if event is not None:
                    self.state.map.add_contact(self.state.pose, event,
                                               self.turn_count)
                    self.state.route = Route()
                    self.state.last_event = event
                else:
                    self.state.last_event = None
                self.state.add_trail_point()
                pose_now = self.state.pose.copy()
                obstacles = len(self.state.map.obstacles)
            self._dirty()
            return {
                "ok": True,
                "requested_cm": round(distance_cm, 2),
                "travelled_cm": round(travelled, 2),
                "event": event,
                "remaining_cm": None if remaining is None else round(remaining, 2),
                "pose": {"x": round(pose_now.x, 3), "y": round(pose_now.y, 3),
                         "heading_deg": round(pose_now.heading, 2)},
                "mapped_obstacles": obstacles,
                "turn_count": self.turn_count,
                "response": response.replace("\n", " | "),
            }

    def turn_ccw(self, degrees: float) -> dict[str, Any]:
        if not math.isfinite(degrees):
            return {"ok": False, "error": "angle must be finite"}
        if abs(degrees) > MAX_TURN_DEG:
            return {"ok": False, "error": f"turn exceeds {MAX_TURN_DEG} deg"}
        if abs(degrees) < 0.2:
            return {"ok": True, "requested_deg": 0.0, "applied_deg": 0.0,
                    "response": "d"}

        with self.cmd_lock:
            pose = self._pose()
            scale = self.config.turn_scale if self.config.turn_scale > 0 else 1.0
            # Firmware treats positive t as clockwise; expose CCW-positive.
            sent = int(round(-degrees / scale))
            try:
                response = self.link.send(f"t {sent}")
            except CybotLinkError as exc:
                self._link_failed(str(exc))
                return {"ok": False, "error": str(exc)}
            applied = -sent * scale
            self.turn_count += 1
            with self.state.lock:
                self.state.pose.heading = wrap360(self.state.pose.heading + applied)
                self.state.add_trail_point()
                pose_now = self.state.pose.copy()
            if self.last_turn is None:
                self.last_turn = {"requested": degrees, "applied": applied}
            else:
                self.last_turn["requested"] += degrees
                self.last_turn["applied"] += applied
            self._post("heading_changed")
            self._dirty()
            return {
                "ok": True,
                "requested_deg": round(degrees, 2),
                "applied_deg": round(applied, 2),
                "pose": {"x": round(pose_now.x, 3), "y": round(pose_now.y, 3),
                         "heading_deg": round(pose_now.heading, 2)},
                "turn_count": self.turn_count,
                "response": response.replace("\n", " | "),
            }

    def scan(self) -> dict[str, Any]:
        with self.cmd_lock:
            pose = self._pose()
            turn_info = self.last_turn
            self.last_turn = None
            try:
                response = self.link.send("s")
            except CybotLinkError as exc:
                self._link_failed(str(exc))
                return {"ok": False, "error": str(exc)}

            detections: list[Detection] = []
            for line in response.splitlines():
                parts = line.split()
                if len(parts) < 3:
                    continue
                try:
                    detections.append(Detection(float(parts[0]), float(parts[1]),
                                                float(parts[2])))
                except ValueError:
                    continue

            with self.state.lock:
                previous_landmarks = list(self.state.last_scan_landmarks)
                mapped_landmarks = list(self.state.map.obstacles)
                previous_detections = list(self.state.last_scan)
                last_pose = self.state.last_scan_pose
                same_pose = (
                    last_pose is not None
                    and self.state.last_scan_turn == self.turn_count
                    and math.hypot(pose.x - last_pose.x, pose.y - last_pose.y) < 0.03
                    and abs(wrap180(pose.heading - last_pose.heading)) < 3.0
                )

            # In-place turn calibration: a scan right after a turn gives a pure
            # rotation, so overlapping objects measure the encoder's real angle.
            turn_calibration: Optional[dict[str, Any]] = None
            if (turn_info is not None and self.config.scan_correction
                    and abs(turn_info["applied"]) >= 5.0):
                estimate, inliers, score = estimate_turn_from_scans(
                    previous_detections, detections, turn_info["applied"]
                )
                if estimate is not None:
                    delta = wrap180(estimate - turn_info["applied"])
                    if abs(delta) <= 25.0:
                        pose.heading = wrap360(pose.heading + delta)
                        if abs(turn_info["applied"]) > 1.0:
                            ratio = estimate / turn_info["applied"]
                            self.config.turn_scale = clamp(
                                0.50 * self.config.turn_scale + 0.50 * ratio,
                                0.80, 1.20,
                            )
                        turn_calibration = {
                            "estimated_deg": round(estimate, 2),
                            "applied_deg": round(turn_info["applied"], 2),
                            "delta_deg": round(delta, 2),
                            "inliers": inliers,
                            "score": round(score, 3),
                            "turn_scale": round(self.config.turn_scale, 4),
                        }

            correction = Correction(note="disabled")
            if turn_calibration is not None:
                # The scan was used for the pure-rotation estimate; matching the
                # same landmarks again would apply the correction twice.
                corrected = pose
                correction = Correction(
                    note="turn calibration applied",
                    source="turn",
                    applied=True,
                    matched=int(turn_calibration["inliers"]),
                    inliers=int(turn_calibration["inliers"]),
                    heading_delta=float(turn_calibration["delta_deg"]),
                )
            elif self.config.scan_correction:
                previous_result = None
                mapped_result = None
                if len(previous_landmarks) >= 2 and detections:
                    p, c = self.localizer.correct(pose, detections, previous_landmarks)
                    c.source = "prev scan"
                    previous_result = (p, c)
                if len(mapped_landmarks) >= 2 and detections:
                    p, c = self.localizer.correct(pose, detections, mapped_landmarks)
                    c.source = "map"
                    mapped_result = (p, c)

                # Prefer the recent scan (tight local frame), then the full map.
                if previous_result and previous_result[1].applied and previous_result[1].inliers >= 2:
                    corrected, correction = previous_result
                elif mapped_result and mapped_result[1].applied and mapped_result[1].inliers >= 2:
                    corrected, correction = mapped_result
                elif previous_result and previous_result[1].matched >= 2:
                    corrected, correction = previous_result
                elif mapped_result and mapped_result[1].matched >= 2:
                    corrected, correction = mapped_result
                else:
                    corrected = pose
                    correction = (previous_result or mapped_result or
                                  (pose, Correction()))[1]
                    correction.note = "no landmark matches"
            else:
                corrected = pose

            with self.state.lock:
                self.state.pose = corrected
                added, removed = self.state.map.refresh_scan(
                    corrected, detections, self.turn_count, average=same_pose
                )
                self.state.last_scan = detections
                self.state.last_scan_landmarks = detections_to_landmarks(
                    detections, corrected
                )
                self.state.last_scan_pose = corrected.copy()
                self.state.last_scan_turn = self.turn_count
                self.state.last_correction = correction
                if self.state.goal is not None:
                    self.state.route = self.state.map.plan(
                        (corrected.x, corrected.y), self.state.goal
                    )
                pose_now = self.state.pose.copy()
                total = len(self.state.map.obstacles)
                route = self.state.route.as_dict()
            self._dirty()
            return {
                "ok": True,
                "detections": [d.as_dict() for d in detections[:40]],
                "detection_count": len(detections),
                "new_obstacles": added,
                "removed_obstacles": len(removed),
                "removed_ids": removed,
                "same_pose_merge": same_pose,
                "mapped_obstacles": total,
                "turn_count": self.turn_count,
                "correction": correction.as_dict(),
                "turn_calibration": turn_calibration,
                "turn_scale": round(self.config.turn_scale, 4),
                "pose": {"x": round(pose_now.x, 3), "y": round(pose_now.y, 3),
                         "heading_deg": round(pose_now.heading, 2)},
                "route": route,
            }

    def set_goal(self, x: float, y: float) -> dict[str, Any]:
        if not (math.isfinite(x) and math.isfinite(y)):
            return {"ok": False, "error": "goal must be finite"}
        if abs(x) > FIELD_HALF - 0.10 or abs(y) > FIELD_HALF - 0.10:
            return {"ok": False,
                    "error": f"goal outside mapped field (|x|,|y| <= {FIELD_HALF - 0.1:.1f} m)"}
        with self.cmd_lock:
            with self.state.lock:
                pose = self.state.pose.copy()
                route = self.state.map.plan((pose.x, pose.y), (x, y))
                self.state.goal = (x, y)
                self.state.route = route
                snapshot = route.as_dict()
            self._dirty()
            return {
                "ok": True,
                "reachable": bool(route.points),
                "goal": {"x": round(x, 3), "y": round(y, 3)},
                "route": snapshot,
                "pose": {"x": round(pose.x, 3), "y": round(pose.y, 3),
                         "heading_deg": round(pose.heading, 2)},
                "obstacles": self.state.map.snapshot(20, self.turn_count),
                "turn_count": self.turn_count,
                "error": None if route.points else "no collision-free route with current map",
            }

    def navigate_to(self, x: float, y: float) -> dict[str, Any]:
        """Turn/move one leg. Caller holds cmd_lock."""
        pose = self._pose()
        dx, dy = x - pose.x, y - pose.y
        if math.hypot(dx, dy) < 0.003:
            return {"ok": True, "event": None, "travelled_cm": 0.0}
        da = wrap180(math.degrees(math.atan2(dy, dx)) - pose.heading)
        if abs(da) > 0.75:
            result = self.turn_ccw(da)
            if not result.get("ok"):
                return result
        distance_cm = math.hypot(dx, dy) * 100.0
        return self.move_cm(distance_cm)

    def follow_route(self, max_steps: int = 6) -> dict[str, Any]:
        max_steps = int(clamp(max_steps, 1, 15))
        with self.cmd_lock:
            completed = 0
            events: list[dict[str, Any]] = []
            while completed < max_steps:
                if self.stop_event is not None and self.stop_event.is_set():
                    return {"ok": False, "error": "operator stopped the agent",
                            "completed_steps": completed, "events": events}
                with self.state.lock:
                    if not self.state.route.points:
                        break
                    target = self.state.route.points[0]
                result = self.navigate_to(*target)
                if not result.get("ok"):
                    return {"ok": False, "error": result.get("error"),
                            "completed_steps": completed, "events": events}
                if result.get("event"):
                    events.append({"step": completed, "event": result["event"],
                                   "remaining_cm": result.get("remaining_cm")})
                    with self.state.lock:
                        goal = self.state.goal
                        pose = self.state.pose.copy()
                    return {
                        "ok": True,
                        "completed": False,
                        "blocked": True,
                        "event": result["event"],
                        "remaining_cm": result.get("remaining_cm"),
                        "completed_steps": completed,
                        "goal": None if goal is None else {"x": goal[0], "y": goal[1]},
                        "pose": {"x": round(pose.x, 3), "y": round(pose.y, 3),
                                 "heading_deg": round(pose.heading, 2)},
                        "advice": "Back up 5-10 cm first, then scan and replan; a nearby object can block the scan.",
                    }
                with self.state.lock:
                    if self.state.route.points:
                        self.state.route.points.pop(0)
                    remaining = len(self.state.route.points)
                completed += 1
                if (self.config.correction_every > 0
                        and completed % self.config.correction_every == 0
                        and self.config.scan_correction):
                    scan_result = self.scan()
                    if not scan_result.get("ok"):
                        return {"ok": False, "error": scan_result.get("error"),
                                "completed_steps": completed, "events": events}
            with self.state.lock:
                remaining = len(self.state.route.points)
                pose = self.state.pose.copy()
                goal = self.state.goal
            self._dirty()
            return {
                "ok": True,
                "completed": remaining == 0,
                "blocked": False,
                "completed_steps": completed,
                "remaining_waypoints": remaining,
                "goal": None if goal is None else {"x": goal[0], "y": goal[1]},
                "pose": {"x": round(pose.x, 3), "y": round(pose.y, 3),
                         "heading_deg": round(pose.heading, 2)},
            }

    def raw_command(self, command: str) -> dict[str, Any]:
        command = command.strip()
        if not command:
            return {"ok": False, "error": "empty command"}
        if "\n" in command or "\r" in command:
            return {"ok": False, "error": "one command at a time"}
        if len(command) > 16:
            return {"ok": False,
                    "error": "command too long (firmware buffer is 20 bytes)"}
        code = command[0].lower()
        if code == "z":
            return {"ok": False,
                    "error": "z shuts the firmware command loop down; use Disconnect instead"}
        if code == "m":
            try:
                return self.move_cm(float(command[1:].strip()))
            except ValueError:
                return {"ok": False, "error": "m needs a numeric distance"}
        if code == "t":
            try:
                return self.turn_ccw(-float(command[1:].strip()))
            except ValueError:
                return {"ok": False, "error": "t needs a numeric angle"}
        if code == "s":
            return self.scan()
        with self.cmd_lock:
            try:
                response = self.link.send(command)
            except CybotLinkError as exc:
                self._link_failed(str(exc))
                return {"ok": False, "error": str(exc)}
            self._dirty()
            return {"ok": True, "response": response.replace("\n", " | ")}

    def clear_map(self) -> dict[str, Any]:
        with self.cmd_lock:
            with self.state.lock:
                self.state.map.clear()
                self.state.last_scan = []
                self.state.last_scan_landmarks = []
                self.state.last_scan_pose = None
                self.state.last_scan_turn = -1
                if self.state.goal is not None:
                    pos = self.state.pose
                    self.state.route = self.state.map.plan(
                        (pos.x, pos.y), self.state.goal
                    )
                else:
                    self.state.route = Route()
            self._dirty()
            return {"ok": True, "obstacles": 0,
                    "turn_count": self.turn_count}

    def play_song(self, song: Any = SONG_FUNKYTOWN) -> dict[str, Any]:
        if isinstance(song, str):
            key = song.strip().lower()
            if key in ("rango", "0"):
                index = SONG_RANGO
            elif key in ("funkytown", "funky", "1"):
                index = SONG_FUNKYTOWN
            else:
                return {"ok": False, "error": f"unknown song {song!r}"}
        elif isinstance(song, bool):
            return {"ok": False, "error": "song must be a name or index"}
        else:
            try:
                index = int(song)
            except (TypeError, ValueError):
                return {"ok": False, "error": "song must be a name or index"}
        if index not in SONG_NAMES:
            return {"ok": False, "error": "song index must be 0 or 1"}
        with self.cmd_lock:
            try:
                response = self.link.send(f"o {index}")
            except CybotLinkError as exc:
                self._link_failed(str(exc))
                return {"ok": False, "error": str(exc)}
            return {
                "ok": True,
                "song": SONG_NAMES[index],
                "index": index,
                "response": response.replace("\n", " | "),
            }

    def remove_objects(self, ids: list[int]) -> dict[str, Any]:
        with self.cmd_lock:
            with self.state.lock:
                removed, ignored = self.state.map.remove_ids(ids)
                self.state.last_scan_landmarks = []
                self.state.last_scan = []
                self.state.last_scan_pose = None
                self.state.last_scan_turn = -1
                if self.state.goal is not None:
                    pos = self.state.pose
                    self.state.route = self.state.map.plan(
                        (pos.x, pos.y), self.state.goal
                    )
                remaining = len(self.state.map.obstacles)
            self._dirty()
            return {"ok": True, "removed": removed, "ignored": ignored,
                    "remaining": remaining, "turn_count": self.turn_count}

    def _link_failed(self, message: str) -> None:
        with self.state.lock:
            self.state.connected = False
            self.state.mode = "disconnected"
        self._post("link_error", message=message)

    def status(self) -> dict[str, Any]:
        with self.state.lock:
            pose = self.state.pose
            goal = self.state.goal
            route = self.state.route.as_dict()
            corr = self.state.last_correction
            return {
                "ok": True,
                "connected": self.state.connected,
                "mode": self.state.mode,
                "pose": {"x": round(pose.x, 3), "y": round(pose.y, 3),
                         "heading_deg": round(pose.heading, 2)},
                "goal": None if goal is None else {"x": goal[0], "y": goal[1]},
                "route": route,
                "motion_turn": self.turn_count,
                "obstacles": self.state.map.snapshot(40, self.turn_count),
                "obstacle_count": len(self.state.map.obstacles),
                "last_scan": {
                    "detections": len(self.state.last_scan),
                    "correction": corr.as_dict(),
                },
                "last_event": self.state.last_event,
                "settings": {
                    "scan_correction": self.config.scan_correction,
                    "correction_every": self.config.correction_every,
                    "move_scale": self.config.move_scale,
                    "turn_scale": self.config.turn_scale,
                },
            }


# --------------------------------------------------------------------------
# Tools exposed to the model
# --------------------------------------------------------------------------

TOOL_SCHEMAS: list[dict[str, Any]] = [
    {
        "type": "function",
        "function": {
            "name": "get_status",
            "description": (
                "Read pose, goal, route, motion_turn, and mapped obstacles. Each "
                "obstacle has id, kind, hits, added_turn, updated_turn and "
                "age_turns. age_turns counts moves/turns since it was added; "
                "large ages mean the position is probably stale because of "
                "encoder drift. Call this at the start of a task or after an "
                "unexpected result."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "scan",
            "description": (
                "Run a full 180-degree IR scan. Returns detected objects "
                "(servo angle, surface distance in cm, width in cm), applies "
                "scan-based localization, and refreshes the viewed sector: old "
                "scan-derived obstacles there are replaced, while bump/hole/"
                "boundary contacts are kept. Repeated scans from the same pose "
                "average the position and size of re-seen objects. Back up "
                "5-10 cm after a contact before scanning."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "remove_objects",
            "description": (
                "Delete one or more mapped objects by id (ids come from "
                "get_status). Use this for stale objects whose age_turns is high "
                "or whose position no longer matches the room. Invalid ids are "
                "ignored; the call never crashes. Contact objects (bump/hole/"
                "boundary) can also be removed this way. The route is replanned."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "ids": {
                        "type": "array",
                        "items": {"type": "integer"},
                        "description": "obstacle ids to delete",
                    }
                },
                "required": ["ids"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "clear_map",
            "description": (
                "Delete every mapped object (obstacles, bumps, holes, "
                "boundaries) and replan. Use only when the map is too corrupted "
                "to repair object-by-object. The human operator has the same "
                "'Clear all' button."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "finish_sound",
            "description": (
                "Play the completion jingle (Funkytown) after the task is truly "
                "complete. Call it at most once per task, after the final motion "
                "and before the summary. It does not perform any navigation."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "move",
            "description": (
                "Drive straight. Positive distance_cm = forward, negative = "
                "backward. Firmware stops on bump/hole/boundary and reports the "
                "event and remaining distance. After a contact, back up 5-10 cm "
                "before scanning so the object does not corrupt the scan. Maximum "
                "magnitude 500 cm; prefer short legs (<= 50 cm) while exploring."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "distance_cm": {"type": "number",
                                    "description": "signed distance in centimetres"}
                },
                "required": ["distance_cm"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "turn",
            "description": (
                "Rotate in place. Positive degrees = counter-clockwise (left), "
                "negative = clockwise (right). Maximum magnitude 360 degrees."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "degrees": {"type": "number",
                                "description": "signed angle in degrees, CCW positive"}
                },
                "required": ["degrees"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "set_goal",
            "description": (
                "Set a goal in field coordinates (metres, origin at the start "
                "pose, +x right, +y up). Plans an A* route around mapped "
                "obstacles and draws it. Returns reachable=false if the current "
                "map blocks every route."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                },
                "required": ["x", "y"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "follow_path",
            "description": (
                "Execute up to max_steps waypoints of the current route. Stops "
                "immediately on bump/hole/boundary and records the obstacle. "
                "Scans automatically every few waypoints when correction is on."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "max_steps": {"type": "integer", "minimum": 1, "maximum": 15}
                },
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "raw_command",
            "description": (
                "Send a raw firmware command without the trailing newline, for "
                "example 'p' (ping), 'o 0' (Rango), 'o 1' (Funkytown) or "
                "'f' (flush). m/t/s are "
                "routed through the typed tools so the map stays consistent. "
                "z is blocked because it shuts down the firmware loop."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "command": {"type": "string"}
                },
                "required": ["command"],
            },
        },
    },
]


class ToolRuntime:
    def __init__(self, controller: CybotController,
                 stop_event: Optional[threading.Event] = None) -> None:
        self.controller = controller
        self.stop_event = stop_event

    def schemas(self) -> list[dict[str, Any]]:
        return TOOL_SCHEMAS

    def _stopped(self) -> bool:
        return bool(self.stop_event and self.stop_event.is_set())

    def execute(self, name: str, args: dict[str, Any]) -> dict[str, Any]:
        if self._stopped():
            return {"ok": False, "error": "operator stopped the agent"}
        try:
            if name == "get_status":
                return self.controller.status()
            if name == "scan":
                return self.controller.scan()
            if name == "finish_sound":
                result = self.controller.play_song(SONG_FUNKYTOWN)
                result["finish"] = True
                return result
            if name == "remove_objects":
                raw = args.get("ids")
                if raw is None:
                    raw = args.get("objects")
                if raw is None:
                    raw = args.get("object_ids")
                if raw is None:
                    return {"ok": False, "error": "ids is required"}
                if not isinstance(raw, list):
                    raw = [raw]
                ids: list[int] = []
                for item in raw:
                    if isinstance(item, dict):
                        item = item.get("id")
                    if isinstance(item, bool):
                        continue
                    try:
                        ids.append(int(item))
                    except (TypeError, ValueError):
                        continue
                return self.controller.remove_objects(ids)
            if name == "clear_map":
                return self.controller.clear_map()
            if name == "move":
                distance = args.get("distance_cm")
                if isinstance(distance, bool) or not isinstance(distance, (int, float)):
                    return {"ok": False, "error": "distance_cm must be a number"}
                return self.controller.move_cm(float(distance))
            if name == "turn":
                degrees = args.get("degrees")
                if isinstance(degrees, bool) or not isinstance(degrees, (int, float)):
                    return {"ok": False, "error": "degrees must be a number"}
                return self.controller.turn_ccw(float(degrees))
            if name == "set_goal":
                x, y = args.get("x"), args.get("y")
                if (isinstance(x, bool) or isinstance(y, bool)
                        or not isinstance(x, (int, float))
                        or not isinstance(y, (int, float))):
                    return {"ok": False, "error": "x and y must be numbers"}
                return self.controller.set_goal(float(x), float(y))
            if name == "follow_path":
                steps = args.get("max_steps", 6)
                if isinstance(steps, bool) or not isinstance(steps, (int, float)):
                    return {"ok": False, "error": "max_steps must be an integer"}
                return self.controller.follow_route(int(steps))
            if name == "raw_command":
                command = args.get("command")
                if not isinstance(command, str):
                    return {"ok": False, "error": "command must be a string"}
                return self.controller.raw_command(command)
            return {"ok": False, "error": f"unknown tool {name}"}
        except Exception as exc:  # keep the agent loop alive on unexpected errors
            return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}


# --------------------------------------------------------------------------
# OpenRouter streaming client
# --------------------------------------------------------------------------

class LLMError(RuntimeError):
    pass


class Cancelled(RuntimeError):
    pass


@dataclass
class ToolCall:
    id: str
    name: str
    arguments: str

    def as_openai(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "type": "function",
            "function": {"name": self.name, "arguments": self.arguments},
        }


@dataclass
class ChatResult:
    content: str = ""
    reasoning: str = ""
    tool_calls: list[ToolCall] = field(default_factory=list)
    finish_reason: str = ""


class OpenRouterClient:
    def __init__(self, config: AppConfig,
                 stop_event: Optional[threading.Event] = None,
                 timeout: float = 240.0) -> None:
        self.config = config
        self.stop_event = stop_event
        self.timeout = timeout

    def _endpoint(self) -> tuple[str, str, Optional[int], str]:
        base = (self.config.base_url or DEFAULT_BASE_URL).rstrip("/")
        parsed = urlsplit(base)
        if not parsed.scheme or not parsed.hostname:
            raise LLMError(f"invalid OPENROUTER_BASE_URL: {base!r}")
        path = (parsed.path or "").rstrip("/")
        if not path.endswith("/chat/completions"):
            path += "/chat/completions"
        return parsed.scheme, parsed.hostname, parsed.port, path

    def _payload(self, messages: list[dict[str, Any]],
                 tools: list[dict[str, Any]]) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "model": self.config.model,
            "messages": messages,
            "tools": tools,
            "tool_choice": "auto",
            "temperature": self.config.temperature,
            "stream": True,
            "include_reasoning": True,
        }
        effort = (self.config.reasoning_effort or "default").lower()
        if effort in ("low", "medium", "high"):
            payload["reasoning"] = {"effort": effort}
        return payload

    def stream_chat(self, messages: list[dict[str, Any]],
                    tools: list[dict[str, Any]],
                    on_reasoning: Callable[[str], None],
                    on_content: Callable[[str], None]) -> ChatResult:
        if not self.config.api_key:
            raise LLMError(
                "No OpenRouter API key. Set OPENROUTER_API_KEY or paste one in Settings."
            )
        scheme, host, port, path = self._endpoint()
        payload = json.dumps(self._payload(messages, tools)).encode("utf-8")
        headers = {
            "Authorization": f"Bearer {self.config.api_key}",
            "Content-Type": "application/json",
            "Accept": "text/event-stream",
            "HTTP-Referer": "https://github.com/",
            "X-Title": APP_TITLE,
        }
        cls = http.client.HTTPSConnection if scheme == "https" else http.client.HTTPConnection
        conn = cls(host, port, timeout=self.timeout)
        result = ChatResult()
        calls: dict[int, dict[str, str]] = {}
        try:
            conn.request("POST", path, body=payload, headers=headers)
            response = conn.getresponse()
            if response.status != 200:
                body = response.read().decode("utf-8", "replace")
                detail = body
                try:
                    parsed = json.loads(body)
                    detail = parsed.get("error", {}).get("message", body) if isinstance(parsed, dict) else body
                except (ValueError, AttributeError):
                    pass
                if response.status == 401:
                    raise LLMError("OpenRouter rejected the API key (HTTP 401).")
                raise LLMError(f"OpenRouter HTTP {response.status}: {detail[:800]}")

            ctype = (response.getheader("Content-Type") or "").lower()
            if "text/event-stream" not in ctype:
                body = response.read().decode("utf-8", "replace")
                return self._parse_non_stream(body, on_reasoning, on_content)

            while True:
                if self.stop_event and self.stop_event.is_set():
                    raise Cancelled()
                raw = response.readline()
                if not raw:
                    break
                line = raw.decode("utf-8", "replace").strip()
                if not line or line.startswith(":"):
                    continue
                if not line.startswith("data:"):
                    continue
                data = line[5:].strip()
                if data == "[DONE]":
                    break
                try:
                    obj = json.loads(data)
                except ValueError:
                    continue
                if "error" in obj:
                    err = obj["error"]
                    message = err.get("message", err) if isinstance(err, dict) else err
                    raise LLMError(str(message))
                choices = obj.get("choices") or []
                if not choices:
                    continue
                choice = choices[0]
                delta = choice.get("delta") or {}
                reasoning = delta.get("reasoning") or delta.get("reasoning_content")
                if reasoning:
                    result.reasoning += reasoning
                    on_reasoning(reasoning)
                content = delta.get("content")
                if content:
                    result.content += content
                    on_content(content)
                for tc in delta.get("tool_calls") or []:
                    idx = int(tc.get("index", 0))
                    slot = calls.setdefault(idx, {"id": "", "name": "", "arguments": ""})
                    call_id = tc.get("id")
                    if call_id and not slot["id"]:
                        slot["id"] = call_id
                    fn = tc.get("function") or {}
                    name = fn.get("name")
                    if name:
                        if not slot["name"]:
                            slot["name"] = name
                        elif name != slot["name"] and not slot["name"].endswith(name):
                            slot["name"] += name
                    args = fn.get("arguments")
                    if args:
                        slot["arguments"] += args
                if choice.get("finish_reason"):
                    result.finish_reason = str(choice["finish_reason"])
        finally:
            conn.close()

        for idx in sorted(calls):
            slot = calls[idx]
            if not slot["name"]:
                continue
            result.tool_calls.append(ToolCall(
                id=slot["id"] or f"call_{idx}_{int(time.time() * 1000)}",
                name=slot["name"],
                arguments=slot["arguments"] or "{}",
            ))
        return result

    def _parse_non_stream(self, body: str,
                          on_reasoning: Callable[[str], None],
                          on_content: Callable[[str], None]) -> ChatResult:
        try:
            obj = json.loads(body)
        except ValueError as exc:
            raise LLMError(f"invalid JSON from OpenRouter: {body[:400]}") from exc
        if "error" in obj:
            err = obj["error"]
            message = err.get("message", err) if isinstance(err, dict) else err
            raise LLMError(str(message))
        choices = obj.get("choices") or []
        if not choices:
            raise LLMError("OpenRouter returned no choices")
        message = choices[0].get("message") or {}
        result = ChatResult(
            content=message.get("content") or "",
            reasoning=message.get("reasoning") or message.get("reasoning_content") or "",
            finish_reason=str(choices[0].get("finish_reason") or ""),
        )
        if result.reasoning:
            on_reasoning(result.reasoning)
        if result.content:
            on_content(result.content)
        for idx, tc in enumerate(message.get("tool_calls") or []):
            fn = tc.get("function") or {}
            result.tool_calls.append(ToolCall(
                id=tc.get("id") or f"call_{idx}_{int(time.time() * 1000)}",
                name=fn.get("name") or "",
                arguments=fn.get("arguments") or "{}",
            ))
        return result


# --------------------------------------------------------------------------
# Tool-calling agent
# --------------------------------------------------------------------------

SYSTEM_PROMPT = """\
You are the navigation copilot for a CyBot (iRobot Create 2) in a 6 x 6 metre
indoor field. You drive it ONLY through the provided tools. The human operator
can interrupt or take over at any moment.

COORDINATES AND UNITS
- Origin is the pose where the robot started. +x is east/right, +y is north/up.
- heading is degrees counter-clockwise from +x; start heading is 90 (north).
- Field limits are |x|, |y| <= 2.9 m.
- move: centimetres, positive forward. turn: degrees, positive counter-clockwise.
- Pose is dead-reckoned and noisy. Every scan matches detected landmarks
  against the map and corrects some encoder error, but this is only partial.

MAP AGING AND STALE OBJECTS
- Every successful move or turn increments motion_turn. Every obstacle from
  get_status has: id, kind, hits, added_turn, updated_turn and age_turns.
- The encoders drift. An object that is several movements old (age_turns >= 3)
  is probably no longer drawn where it really is. Do not trust old objects for
  tight routes. Re-scan the area before relying on one, or delete it with
  remove_objects(ids=[...]).
- remove_objects can delete several ids in one call. Invalid ids are ignored and
  never crash. clear_map deletes every object; the human has the same button.
- kind "bump", "hole" and "boundary" are direct contact evidence. A scan never
  erases them; only remove_objects or clear_map can.
- A scan refreshes the 180-degree sector in front of the robot: scan-derived
  objects in that sector that are no longer seen are removed, and new detections
  are added. Repeated scans from the same pose average position and size.
- After bump/hole/boundary, back up 5-10 cm (move a negative distance) before
  scanning. An object right at the bumper blocks or distorts the scan.

HOW TO WORK
1. Call get_status at the start of a task and after any surprise.
2. To travel: set_goal(x, y), then follow_path(). Inspect the returned event.
   On a contact, do NOT repeat the same move: back up 5-10 cm, scan, remove any
   phantom objects that box you in, then set_goal again to replan.
3. While exploring, use short move legs (<= 50 cm) and scans. A scan only sees
   about 65 cm, so absence of detections is not proof that space is clear.
4. follow_path executes known-clear waypoints. It stops immediately on contact.
5. Use raw_command for ping ('p'), songs ('o 0' = Rango, 'o 1' = Funkytown),
   flush ('f') or other firmware commands. m/t/s are routed through typed tools
   so the map stays consistent. 'z' is blocked because it shuts down the
   firmware loop.
6. Never claim to have moved or observed something without a tool result.
7. If a tool reports ok=false, explain the failure and recover safely.
8. When the task is genuinely finished, call finish_sound once (Funkytown),
   then give the final summary. Do not use it to pretend a task succeeded.

STYLE
- Before tools, state the intent in one or two short sentences.
- While navigating, explain meaningful detours and why they are safe.
- Finish with a compact summary: pose, goal, route status, hazards.
- Do not ask the operator to confirm routine safe moves; act. Ask only when the
  instruction is ambiguous or clearly unsafe.
- Be honest about uncertainty and about what the scan did not see.
"""


class AgentRunner:
    def __init__(self, config: AppConfig, messages: list[dict[str, Any]],
                 tools: ToolRuntime, emit: Callable[..., None],
                 stop_event: threading.Event) -> None:
        self.config = config
        self.messages = messages
        self.tools = tools
        self.emit = emit
        self.stop_event = stop_event

    def _trim_history(self) -> None:
        if len(self.messages) <= 70:
            return
        tail = self.messages[-45:]
        while tail and tail[0].get("role") == "tool":
            tail.pop(0)
        self.messages[:] = [self.messages[0]] + tail

    def _telemetry(self) -> str:
        status = self.tools.controller.status()
        compact = {
            "pose": status.get("pose"),
            "goal": status.get("goal"),
            "route_waypoints": len(status.get("route", {}).get("waypoints", [])),
            "route_m": status.get("route", {}).get("distance_m"),
            "obstacles": status.get("obstacle_count"),
            "motion_turn": status.get("motion_turn"),
            "last_event": status.get("last_event"),
            "last_correction": status.get("last_scan", {}).get("correction"),
            "mode": status.get("mode"),
        }
        return json.dumps(compact, separators=(",", ":"))

    def run(self, operator_text: str) -> None:
        if not self.messages or self.messages[0].get("role") != "system":
            self.messages.insert(0, {"role": "system", "content": SYSTEM_PROMPT})

        user_content = (
            f"Operator instruction:\n{operator_text}\n\n"
            f"[auto telemetry] {self._telemetry()}"
        )
        self.messages.append({"role": "user", "content": user_content})
        self._trim_history()
        client = OpenRouterClient(self.config, self.stop_event)
        steps = max(1, min(int(self.config.max_agent_steps), 30))
        finished = False

        try:
            for step in range(steps):
                if self.stop_event.is_set():
                    raise Cancelled()
                self.emit("agent_status", text=f"thinking · step {step + 1}")
                self.emit("assistant_start", step=step + 1)
                result = client.stream_chat(
                    self.messages,
                    self.tools.schemas(),
                    on_reasoning=lambda text: self.emit("reasoning_delta", text=text),
                    on_content=lambda text: self.emit("assistant_delta", text=text),
                )
                assistant_msg: dict[str, Any] = {
                    "role": "assistant",
                    "content": result.content or "",
                }
                if result.tool_calls:
                    assistant_msg["tool_calls"] = [c.as_openai() for c in result.tool_calls]
                self.messages.append(assistant_msg)
                self._trim_history()

                if not result.tool_calls:
                    finished = True
                    break

                for call in result.tool_calls:
                    if self.stop_event.is_set():
                        raise Cancelled()
                    self.emit("agent_status", text=f"tool: {call.name}")
                    self.emit("tool_call", name=call.name,
                              arguments=call.arguments)
                    try:
                        args = json.loads(call.arguments or "{}")
                        if not isinstance(args, dict):
                            raise ValueError("tool arguments must be a JSON object")
                        tool_result = self.tools.execute(call.name, args)
                    except (ValueError, TypeError) as exc:
                        tool_result = {"ok": False,
                                       "error": f"bad tool arguments: {exc}"}
                    serialized = json.dumps(tool_result, ensure_ascii=False)
                    self.emit("tool_result", name=call.name, text=serialized,
                              ok=bool(tool_result.get("ok", True)))
                    self.messages.append({
                        "role": "tool",
                        "tool_call_id": call.id,
                        "content": serialized,
                    })
                    self.emit("map_dirty")
                self._trim_history()
            if not finished and not self.stop_event.is_set():
                self.emit("notice",
                          text=f"Reached the {steps}-step limit for this turn. "
                               "Send another instruction to continue.")
        except Cancelled:
            self.emit("notice", text="Agent stopped by operator.")
        except LLMError as exc:
            self.emit("error", text=str(exc))
        except Exception as exc:
            self.emit("error", text=f"{type(exc).__name__}: {exc}")
        finally:
            self.emit("agent_busy", busy=False)
            self.emit("agent_status", text="idle")


# --------------------------------------------------------------------------
# Tk UI
# --------------------------------------------------------------------------

class CyBotApp:
    def __init__(self, root: tk.Tk, args: argparse.Namespace) -> None:
        self.root = root
        self.args = args
        self.config = AppConfig(
            api_key=os.getenv("OPENROUTER_API_KEY", ""),
            model=args.model or os.getenv("OPENROUTER_MODEL", DEFAULT_MODEL),
            base_url=os.getenv("OPENROUTER_BASE_URL", DEFAULT_BASE_URL),
        )
        self.state = RobotState()
        self.controller: Optional[CybotController] = None
        self.events: "queue.Queue[tuple[str, dict[str, Any]]]" = queue.Queue()
        self.agent_stop = threading.Event()
        self.agent_thread: Optional[threading.Thread] = None
        self.agent_busy = False
        self.manual_busy = False
        self._connecting = False
        self._map_dirty = True
        self._chat_cur: Optional[str] = None
        self._ai_status = "idle"
        self._last_goal_text = "none"
        self.messages: list[dict[str, Any]] = [
            {"role": "system", "content": SYSTEM_PROMPT}
        ]

        self.root.title(APP_TITLE)
        self.root.geometry("1380x880")
        self.root.minsize(1120, 760)
        self.root.configure(bg=C["bg"])
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_style()
        self._build_header()
        self._build_body()
        self._build_statusbar()
        self._bind_keys()
        self._sync_conn_fields()
        self._refresh_connection_labels()
        self._update_control_states()
        self._chat_welcome()

        self.root.after(50, self._poll_events)
        self.root.after(250, self._refresh_status)
        if not args.no_autoconnect and not args.tcp and not args.serial:
            self.root.after(350, self._connect)

    # -- styling -----------------------------------------------------------

    def _build_style(self) -> None:
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TFrame", background=C["bg"])
        style.configure("Panel.TFrame", background=C["panel"])
        style.configure("TLabel", background=C["bg"], foreground=C["text"],
                        font=FONT_UI)
        style.configure("Panel.TLabel", background=C["panel"], foreground=C["text"],
                        font=FONT_UI)
        style.configure("Dim.TLabel", background=C["bg"], foreground=C["dim"],
                        font=FONT_SMALL)
        style.configure("PanelDim.TLabel", background=C["panel"],
                        foreground=C["dim"], font=FONT_SMALL)
        style.configure("Title.TLabel", background=C["bg"], foreground=C["text"],
                        font=("Segoe UI Semibold", 15))
        style.configure("Section.TLabel", background=C["panel"],
                        foreground=C["accent"], font=FONT_UI_BOLD)
        style.configure("TButton", background=C["panel2"], foreground=C["text"],
                        bordercolor=C["line"], focusthickness=1, font=FONT_UI)
        style.map("TButton",
                  background=[("active", C["line"]), ("disabled", C["panel"])],
                  foreground=[("disabled", C["faint"])])
        style.configure("Accent.TButton", background=C["accent"],
                        foreground="#06131b", font=FONT_UI_BOLD)
        style.map("Accent.TButton",
                  background=[("active", "#7ad9f5"), ("disabled", C["panel"])],
                  foreground=[("disabled", C["faint"])])
        style.configure("Stop.TButton", background=C["red"], foreground="white",
                        font=FONT_UI_BOLD)
        style.map("Stop.TButton",
                  background=[("active", "#ff6b8d"), ("disabled", C["panel"])],
                  foreground=[("disabled", C["faint"])])
        style.configure("TNotebook", background=C["bg"], borderwidth=0)
        style.configure("TNotebook.Tab", background=C["panel2"], foreground=C["dim"],
                        padding=(14, 6), font=FONT_UI)
        style.map("TNotebook.Tab",
                  background=[("selected", C["panel"])],
                  foreground=[("selected", C["accent"])])
        style.configure("TEntry", fieldbackground=C["panel2"], foreground=C["text"],
                        insertcolor=C["text"], bordercolor=C["line"])
        style.configure("TCombobox", fieldbackground=C["panel2"], background=C["panel2"],
                        foreground=C["text"], arrowcolor=C["text"],
                        bordercolor=C["line"])
        style.map("TCombobox",
                  fieldbackground=[("readonly", C["panel2"])],
                  foreground=[("readonly", C["text"])])
        style.configure("TSpinbox", fieldbackground=C["panel2"], foreground=C["text"],
                        arrowcolor=C["text"], bordercolor=C["line"])
        style.configure("TCheckbutton", background=C["panel"], foreground=C["text"],
                        font=FONT_UI)
        style.map("TCheckbutton", background=[("active", C["panel"])])
        style.configure("TScale", background=C["panel"], troughcolor=C["panel2"])
        style.configure("Horizontal.TProgressbar", background=C["accent"])

    # -- layout ------------------------------------------------------------

    def _build_header(self) -> None:
        header = tk.Frame(self.root, bg=C["bg"])
        header.grid(row=0, column=0, sticky="ew", padx=12, pady=(10, 6))
        header.columnconfigure(1, weight=1)

        title = ttk.Label(header, text="CYBOT  //  LLM COPILOT", style="Title.TLabel")
        title.grid(row=0, column=0, sticky="w")
        ttk.Label(header, text="OpenRouter / DeepSeek tool-calling driver",
                  style="Dim.TLabel").grid(row=1, column=0, sticky="w")

        conn = ttk.Frame(header)
        conn.grid(row=0, column=1, rowspan=2, sticky="e")

        ttk.Label(conn, text="Link", style="Dim.TLabel").grid(row=0, column=0, padx=(0, 4))
        self.mode_var = tk.StringVar(value="Simulated")
        self.mode_box = ttk.Combobox(conn, textvariable=self.mode_var, width=11,
                                     values=("Simulated", "TCP", "Serial"),
                                     state="readonly")
        self.mode_box.grid(row=0, column=1, padx=(0, 10))
        self.mode_box.bind("<<ComboboxSelected>>", lambda _e: self._sync_conn_fields())

        self.host_var = tk.StringVar(value=self.args.host)
        self.port_var = tk.StringVar(value=str(self.args.port))
        self.serial_var = tk.StringVar(value=self.args.serial or "COM3")
        self.baud_var = tk.StringVar(value=str(self.args.baud))

        self.tcp_fields = ttk.Frame(conn)
        self.tcp_fields.grid(row=0, column=2, padx=(0, 8))
        ttk.Label(self.tcp_fields, text="Host", style="Dim.TLabel").grid(row=0, column=0, padx=(0, 3))
        self.host_entry = ttk.Entry(self.tcp_fields, textvariable=self.host_var, width=14)
        self.host_entry.grid(row=0, column=1, padx=(0, 4))
        ttk.Label(self.tcp_fields, text="Port", style="Dim.TLabel").grid(row=0, column=2, padx=(0, 3))
        self.port_entry = ttk.Entry(self.tcp_fields, textvariable=self.port_var, width=6)
        self.port_entry.grid(row=0, column=3)

        self.serial_fields = ttk.Frame(conn)
        self.serial_fields.grid(row=0, column=3, padx=(0, 8))
        ttk.Label(self.serial_fields, text="Port", style="Dim.TLabel").grid(row=0, column=0, padx=(0, 3))
        self.serial_entry = ttk.Entry(self.serial_fields, textvariable=self.serial_var, width=8)
        self.serial_entry.grid(row=0, column=1, padx=(0, 4))
        ttk.Label(self.serial_fields, text="Baud", style="Dim.TLabel").grid(row=0, column=2, padx=(0, 3))
        self.baud_entry = ttk.Entry(self.serial_fields, textvariable=self.baud_var, width=8)
        self.baud_entry.grid(row=0, column=3)

        self.link_dot = tk.Label(conn, text="●", bg=C["bg"], fg=C["faint"],
                                 font=("Segoe UI", 12))
        self.link_dot.grid(row=0, column=4, padx=(2, 4))
        self.conn_label = ttk.Label(conn, text="disconnected", style="Dim.TLabel")
        self.conn_label.grid(row=0, column=5, padx=(0, 10))

        self.connect_btn = ttk.Button(conn, text="Connect", width=10,
                                      command=self._toggle_connect)
        self.connect_btn.grid(row=0, column=6, padx=(0, 6))
        self.stop_btn = ttk.Button(conn, text="STOP AI", width=10,
                                   style="Stop.TButton", command=self._stop_agent)
        self.stop_btn.grid(row=0, column=7)

    def _build_body(self) -> None:
        body = tk.Frame(self.root, bg=C["bg"])
        body.grid(row=1, column=0, sticky="nsew", padx=12, pady=(0, 6))
        body.rowconfigure(0, weight=1)
        body.columnconfigure(0, weight=0)
        body.columnconfigure(1, weight=1)
        self.root.rowconfigure(1, weight=1)
        self.root.columnconfigure(0, weight=1)

        # ---- map column --------------------------------------------------
        left = tk.Frame(body, bg=C["panel"], highlightbackground=C["line"],
                        highlightthickness=1)
        left.grid(row=0, column=0, sticky="nsw", padx=(0, 10))

        map_head = tk.Frame(left, bg=C["panel"])
        map_head.pack(fill="x", padx=8, pady=(7, 2))
        tk.Label(map_head, text="FIELD MAP", bg=C["panel"], fg=C["text"],
                 font=FONT_UI_BOLD).pack(side="left")
        self.coord_label = tk.Label(map_head, text="", bg=C["panel"], fg=C["dim"],
                                    font=FONT_SMALL)
        self.coord_label.pack(side="right")

        self.map_canvas = tk.Canvas(left, width=MAP_BOUND, height=MAP_BOUND,
                                    bg=C["panel2"], highlightthickness=0,
                                    cursor="crosshair")
        self.map_canvas.pack(padx=8, pady=(0, 4))
        self.map_canvas.bind("<Button-1>", self._on_map_click)
        self.map_canvas.bind("<Motion>", self._on_map_motion)

        goal_row = tk.Frame(left, bg=C["panel"])
        goal_row.pack(fill="x", padx=8, pady=(0, 4))
        self.goal_label = tk.Label(goal_row, text="Goal: none", bg=C["panel"],
                                   fg=C["dim"], font=FONT_SMALL)
        self.goal_label.pack(side="left")
        self.goal_ai_btn = ttk.Button(goal_row, text="AI: go to goal", width=15,
                                      command=self._ask_ai_goal)
        self.goal_ai_btn.pack(side="right")

        drive = tk.Frame(left, bg=C["panel"])
        drive.pack(fill="x", padx=8, pady=(0, 8))
        self.drive_buttons: list[ttk.Button] = []
        for text, cmd, width in (
            ("⟲ 30°", lambda: self._manual_turn(30), 7),
            ("▲ 25cm", lambda: self._manual_move(25), 7),
            ("▼ 25cm", lambda: self._manual_move(-25), 7),
            ("⟳ 30°", lambda: self._manual_turn(-30), 7),
            ("Scan", self._manual_scan, 7),
            ("Flush", self._manual_flush, 7),
            ("Clear all", self._clear_map, 9),
        ):
            b = ttk.Button(drive, text=text, width=width, command=cmd)
            b.pack(side="left", padx=(0, 3))
            self.drive_buttons.append(b)

        music = tk.Frame(left, bg=C["panel"])
        music.pack(fill="x", padx=8, pady=(0, 5))
        tk.Label(music, text="Song", bg=C["panel"], fg=C["dim"],
                 font=FONT_SMALL).pack(side="left")
        self.song_var = tk.StringVar(value=SONG_NAMES[SONG_RANGO])
        self.song_box = ttk.Combobox(
            music, textvariable=self.song_var, width=14, state="readonly",
            values=(SONG_NAMES[SONG_RANGO], SONG_NAMES[SONG_FUNKYTOWN]),
        )
        self.song_box.pack(side="left", padx=(6, 6))
        self.song_btn = ttk.Button(music, text="Play", width=8,
                                   command=self._manual_song)
        self.song_btn.pack(side="left")
        self.drive_buttons.append(self.song_btn)

        tk.Label(left, text="Keys: W/S forward/back · A/D turn · Space scan",
                 bg=C["panel"], fg=C["faint"], font=FONT_SMALL).pack(pady=(0, 6))

        # ---- right notebook ---------------------------------------------
        self.notebook = ttk.Notebook(body)
        self.notebook.grid(row=0, column=1, sticky="nsew")

        self._build_copilot_tab()
        self._build_console_tab()
        self._build_settings_tab()

    def _build_copilot_tab(self) -> None:
        tab = tk.Frame(self.notebook, bg=C["panel"])
        self.notebook.add(tab, text="  Copilot  ")
        tab.rowconfigure(1, weight=1)
        tab.columnconfigure(0, weight=1)

        head = tk.Frame(tab, bg=C["panel"])
        head.grid(row=0, column=0, sticky="ew", padx=10, pady=(8, 2))
        head.columnconfigure(0, weight=1)
        self.model_label = tk.Label(head, text=self.config.model, bg=C["panel"],
                                    fg=C["accent"], font=FONT_SMALL)
        self.model_label.grid(row=0, column=0, sticky="w")
        self.api_status_label = tk.Label(head, text="", bg=C["panel"], fg=C["dim"],
                                         font=FONT_SMALL)
        self.api_status_label.grid(row=0, column=1, sticky="e")
        self._refresh_api_label()

        chat_wrap = tk.Frame(tab, bg=C["panel2"])
        chat_wrap.grid(row=1, column=0, sticky="nsew", padx=10, pady=(0, 6))
        chat_wrap.rowconfigure(0, weight=1)
        chat_wrap.columnconfigure(0, weight=1)
        self.chat_text = tk.Text(chat_wrap, bg=C["panel2"], fg=C["text"],
                                 insertbackground=C["text"], wrap="word",
                                 relief="flat", padx=10, pady=8,
                                 font=FONT_UI, state="disabled",
                                 spacing1=1, spacing3=1)
        chat_scroll = ttk.Scrollbar(chat_wrap, orient="vertical",
                                    command=self.chat_text.yview)
        self.chat_text.configure(yscrollcommand=chat_scroll.set)
        self.chat_text.grid(row=0, column=0, sticky="nsew")
        chat_scroll.grid(row=0, column=1, sticky="ns")
        self.chat_text.tag_configure("label", foreground=C["dim"],
                                     font=FONT_UI_BOLD, spacing1=12)
        self.chat_text.tag_configure("user", foreground=C["accent"])
        self.chat_text.tag_configure("assistant", foreground=C["text"])
        self.chat_text.tag_configure("reasoning", foreground=C["dim"],
                                     font=("Segoe UI", 9, "italic"))
        self.chat_text.tag_configure("tool", foreground=C["tool"], font=FONT_MONO,
                                     spacing1=3)
        self.chat_text.tag_configure("result", foreground=C["faint"], font=FONT_MONO)
        self.chat_text.tag_configure("error", foreground=C["red"],
                                     font=FONT_UI_BOLD)
        self.chat_text.tag_configure("notice", foreground=C["yellow"])

        prompt_wrap = tk.Frame(tab, bg=C["panel"])
        prompt_wrap.grid(row=2, column=0, sticky="ew", padx=10, pady=(0, 10))
        prompt_wrap.columnconfigure(0, weight=1)
        tk.Label(prompt_wrap, text="INSTRUCTION  ·  Ctrl+Enter to send",
                 bg=C["panel"], fg=C["dim"], font=FONT_SMALL).grid(
            row=0, column=0, columnspan=3, sticky="w")
        self.prompt_text = tk.Text(prompt_wrap, height=3, bg=C["panel2"],
                                   fg=C["text"], insertbackground=C["text"],
                                   relief="flat", wrap="word", padx=8, pady=5,
                                   font=FONT_UI)
        self.prompt_text.grid(row=1, column=0, sticky="ew", pady=(2, 5))
        self.prompt_text.bind("<Control-Return>", self._send_from_event)
        self.prompt_text.bind("<Control-KP_Enter>", self._send_from_event)
        self.send_btn = ttk.Button(prompt_wrap, text="Send", width=10,
                                   style="Accent.TButton", command=self._send_prompt)
        self.send_btn.grid(row=1, column=1, padx=(6, 4), sticky="ns")
        self.reset_btn = ttk.Button(prompt_wrap, text="Reset chat", width=10,
                                    command=self._reset_chat)
        self.reset_btn.grid(row=1, column=2, sticky="ns")
        self.agent_state_label = tk.Label(prompt_wrap, text="agent idle",
                                          bg=C["panel"], fg=C["dim"],
                                          font=FONT_SMALL)
        self.agent_state_label.grid(row=2, column=0, columnspan=3, sticky="w")

    def _build_console_tab(self) -> None:
        tab = tk.Frame(self.notebook, bg=C["panel"])
        self.notebook.add(tab, text="  Console  ")
        tab.rowconfigure(2, weight=1)
        tab.columnconfigure(0, weight=1)
        tk.Label(tab, text="RAW PROTOCOL  ·  every command ends in 'd'",
                 bg=C["panel"], fg=C["dim"], font=FONT_SMALL).grid(
            row=0, column=0, sticky="w", padx=10, pady=(8, 4))
        row = tk.Frame(tab, bg=C["panel"])
        row.grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 6))
        row.columnconfigure(0, weight=1)
        self.raw_var = tk.StringVar()
        self.raw_entry = ttk.Entry(row, textvariable=self.raw_var)
        self.raw_entry.grid(row=0, column=0, sticky="ew")
        self.raw_entry.bind("<Return>", lambda _e: self._send_raw())
        ttk.Button(row, text="Send", width=8,
                   command=self._send_raw).grid(row=0, column=1, padx=(6, 0))

        log_wrap = tk.Frame(tab, bg=C["panel2"])
        log_wrap.grid(row=2, column=0, sticky="nsew", padx=10, pady=(0, 10))
        log_wrap.rowconfigure(0, weight=1)
        log_wrap.columnconfigure(0, weight=1)
        self.proto_text = tk.Text(log_wrap, bg=C["panel2"], fg=C["text"],
                                  wrap="none", relief="flat", padx=8, pady=6,
                                  font=FONT_MONO, state="disabled")
        proto_scroll = ttk.Scrollbar(log_wrap, orient="vertical",
                                     command=self.proto_text.yview)
        self.proto_text.configure(yscrollcommand=proto_scroll.set)
        self.proto_text.grid(row=0, column=0, sticky="nsew")
        proto_scroll.grid(row=0, column=1, sticky="ns")
        self.proto_text.tag_configure("tx", foreground=C["accent"])
        self.proto_text.tag_configure("rx", foreground=C["text"])
        self.proto_text.tag_configure("info", foreground=C["dim"])

    def _build_settings_tab(self) -> None:
        tab = tk.Frame(self.notebook, bg=C["panel"])
        self.notebook.add(tab, text="  Settings  ")
        tab.columnconfigure(1, weight=1)

        row = 0

        def section(text: str) -> None:
            nonlocal row
            ttk.Label(tab, text=text, style="Section.TLabel").grid(
                row=row, column=0, columnspan=2, sticky="w", padx=10,
                pady=(12 if row else 10, 4))
            row += 1

        def label(text: str) -> None:
            nonlocal row
            ttk.Label(tab, text=text, style="PanelDim.TLabel").grid(
                row=row, column=0, sticky="w", padx=(18, 8), pady=3)

        section("OPENROUTER")
        label("API key")
        self.key_var = tk.StringVar(value=self.config.api_key)
        self.key_entry = ttk.Entry(tab, textvariable=self.key_var, show="•", width=52)
        self.key_entry.grid(row=row, column=1, sticky="ew", padx=(0, 10), pady=3)
        row += 1
        label("Model")
        self.model_var = tk.StringVar(value=self.config.model)
        ttk.Entry(tab, textvariable=self.model_var, width=52).grid(
            row=row, column=1, sticky="ew", padx=(0, 10), pady=3)
        row += 1
        label("Reasoning")
        self.reasoning_var = tk.StringVar(value=self.config.reasoning_effort)
        ttk.Combobox(tab, textvariable=self.reasoning_var, width=12,
                     values=("default", "low", "medium", "high"),
                     state="readonly").grid(row=row, column=1, sticky="w",
                                            padx=(0, 10), pady=3)
        row += 1
        label("Temperature")
        self.temp_var = tk.DoubleVar(value=self.config.temperature)
        temp_row = tk.Frame(tab, bg=C["panel"])
        temp_row.grid(row=row, column=1, sticky="w", pady=3)
        ttk.Scale(temp_row, from_=0.0, to=1.5, variable=self.temp_var,
                  length=220).pack(side="left")
        self.temp_label = tk.Label(temp_row, text=f"{self.config.temperature:.2f}",
                                   bg=C["panel"], fg=C["dim"], font=FONT_SMALL,
                                   width=5)
        self.temp_label.pack(side="left", padx=(6, 0))
        self.temp_var.trace_add("write", lambda *_: self.temp_label.configure(
            text=f"{self.temp_var.get():.2f}"))
        row += 1
        label("Max steps / turn")
        self.steps_var = tk.StringVar(value=str(self.config.max_agent_steps))
        ttk.Spinbox(tab, from_=1, to=30, textvariable=self.steps_var,
                    width=8).grid(row=row, column=1, sticky="w", pady=3)
        row += 1
        self.show_key_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(tab, text="Show API key", variable=self.show_key_var,
                        command=lambda: self.key_entry.configure(
                            show="" if self.show_key_var.get() else "•")).grid(
            row=row, column=1, sticky="w", padx=(0, 10), pady=(0, 4))
        row += 1

        section("ENCODER ERROR CORRECTION")
        self.correction_var = tk.BooleanVar(value=self.config.scan_correction)
        ttk.Checkbutton(tab, text="Correct pose from scans (landmark matching)",
                        variable=self.correction_var).grid(
            row=row, column=1, sticky="w", padx=(0, 10), pady=3)
        row += 1
        label("Scan every N waypoints")
        self.every_var = tk.StringVar(value=str(self.config.correction_every))
        ttk.Spinbox(tab, from_=0, to=10, textvariable=self.every_var,
                    width=8).grid(row=row, column=1, sticky="w", pady=3)
        row += 1
        label("Move trim (actual/commanded)")
        self.move_scale_var = tk.StringVar(value=f"{self.config.move_scale:.4f}")
        ttk.Spinbox(tab, from_=0.80, to=1.20, increment=0.005,
                    textvariable=self.move_scale_var, width=8,
                    format="%.3f").grid(row=row, column=1, sticky="w", pady=3)
        row += 1
        label("Turn trim (actual/commanded)")
        self.turn_scale_var = tk.StringVar(value=f"{self.config.turn_scale:.4f}")
        ttk.Spinbox(tab, from_=0.80, to=1.20, increment=0.005,
                    textvariable=self.turn_scale_var, width=8,
                    format="%.3f").grid(row=row, column=1, sticky="w", pady=3)
        row += 1

        section("DISPLAY AND SIMULATION")
        self.reasoning_show_var = tk.BooleanVar(value=self.config.show_reasoning)
        ttk.Checkbutton(tab, text="Show model reasoning", variable=self.reasoning_show_var).grid(
            row=row, column=1, sticky="w", padx=(0, 10), pady=3)
        row += 1
        self.nodes_var = tk.BooleanVar(value=self.config.show_grid_nodes)
        ttk.Checkbutton(tab, text="Show A* lattice nodes", variable=self.nodes_var).grid(
            row=row, column=1, sticky="w", padx=(0, 10), pady=3)
        row += 1
        self.rays_var = tk.BooleanVar(value=self.config.show_scan_rays)
        ttk.Checkbutton(tab, text="Show last scan rays", variable=self.rays_var).grid(
            row=row, column=1, sticky="w", padx=(0, 10), pady=3)
        row += 1
        self.drift_var = tk.BooleanVar(value=self.config.sim_drift)
        ttk.Checkbutton(tab, text="Simulate encoder drift (next simulated link)",
                        variable=self.drift_var).grid(
            row=row, column=1, sticky="w", padx=(0, 10), pady=3)
        row += 1

        apply_row = tk.Frame(tab, bg=C["panel"])
        apply_row.grid(row=row, column=0, columnspan=2, sticky="w", padx=10,
                       pady=(14, 6))
        ttk.Button(apply_row, text="Apply settings", style="Accent.TButton",
                   command=self._apply_settings).pack(side="left")
        tk.Label(apply_row, text="  Key is kept in memory only; env vars win at launch.",
                 bg=C["panel"], fg=C["faint"], font=FONT_SMALL).pack(side="left")
        row += 1
        tk.Label(tab, text=(
            "Move/turn trim compensates systematic encoder error: set 0.97 if a "
            "100 cm command travels 97 cm. Scan correction then removes residual "
            "drift by matching detected obstacles to the map."
        ), bg=C["panel"], fg=C["dim"], font=FONT_SMALL, justify="left",
            wraplength=620).grid(row=row, column=0, columnspan=2, sticky="w",
                                 padx=10, pady=(4, 10))
        row += 1
        self.settings_note = tk.Label(tab, text="", bg=C["panel"], fg=C["accent2"],
                                      font=FONT_SMALL)
        self.settings_note.grid(row=row, column=0, columnspan=2, sticky="w",
                                padx=10, pady=(0, 10))

    def _build_statusbar(self) -> None:
        bar = tk.Frame(self.root, bg=C["panel"], highlightbackground=C["line"],
                       highlightthickness=1)
        bar.grid(row=2, column=0, sticky="ew", padx=12, pady=(0, 10))
        self.status_labels: dict[str, tk.Label] = {}
        for key, title in (
            ("link", "LINK"), ("pose", "POSE"), ("goal", "GOAL"),
            ("route", "ROUTE"), ("corr", "CORRECTION"), ("ai", "AI"),
            ("api", "API"),
        ):
            cell = tk.Frame(bar, bg=C["panel"])
            cell.pack(side="left", padx=(10, 16), pady=5)
            tk.Label(cell, text=title, bg=C["panel"], fg=C["faint"],
                     font=("Segoe UI", 7, "bold")).pack(anchor="w")
            value = tk.Label(cell, text="—", bg=C["panel"], fg=C["text"],
                             font=FONT_SMALL)
            value.pack(anchor="w")
            self.status_labels[key] = value

    def _bind_keys(self) -> None:
        self.root.bind("<KeyPress>", self._on_key)
        self.root.bind("<Escape>", lambda _e: self._stop_agent())

    # -- connection --------------------------------------------------------

    def _sync_conn_fields(self) -> None:
        mode = self.mode_var.get()
        self.tcp_fields.grid_remove()
        self.serial_fields.grid_remove()
        if mode == "TCP":
            self.tcp_fields.grid()
        elif mode == "Serial":
            self.serial_fields.grid()

    def _toggle_connect(self) -> None:
        if self.controller is not None:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        if self.controller is not None:
            return
        mode = self.mode_var.get()
        self._connecting = True
        self._update_control_states()
        self.conn_label.configure(text="connecting…")
        self.link_dot.configure(fg=C["yellow"])

        def worker() -> None:
            try:
                if mode == "Simulated":
                    link = SimLink(drift=self.config.sim_drift,
                                   on_log=self._thread_log)
                elif mode == "TCP":
                    link = TcpLink(self.host_var.get().strip(),
                                   int(self.port_var.get().strip()),
                                   on_log=self._thread_log)
                    link.connect()
                else:
                    link = SerialLink(self.serial_var.get().strip(),
                                      int(self.baud_var.get().strip()),
                                      on_log=self._thread_log)
                    link.connect()
                self._emit("connected", link=link, mode=mode)
            except Exception as exc:
                self._emit("connect_failed", message=f"{type(exc).__name__}: {exc}")

        threading.Thread(target=worker, daemon=True).start()

    def _disconnect(self) -> None:
        self._connecting = False
        self.agent_stop.set()
        if self.controller is not None:
            try:
                self.controller.close()
            except Exception:
                pass
        self.controller = None
        with self.state.lock:
            self.state.connected = False
            self.state.mode = "disconnected"
        self._set_ai_status("idle")
        self._refresh_connection_labels()
        self._update_control_states()
        self._map_dirty = True
        self._chat_notice("Disconnected.")

    def _refresh_connection_labels(self) -> None:
        if self.controller is not None:
            mode = self.state.mode
            label = {"simulated": "SIMULATED", "tcp": "TCP",
                     "serial": "SERIAL"}.get(mode, mode.upper())
            self.conn_label.configure(text=label)
            self.link_dot.configure(fg=C["accent2"])
            self.connect_btn.configure(text="Disconnect")
        else:
            self.conn_label.configure(text="disconnected")
            self.link_dot.configure(fg=C["faint"])
            self.connect_btn.configure(text="Connect")

    def _thread_log(self, direction: str, text: str) -> None:
        self._emit("protocol", direction=direction, text=text)

    def _emit(self, kind: str, **data: Any) -> None:
        self.events.put((kind, data))

    # -- event pump --------------------------------------------------------

    def _poll_events(self) -> None:
        try:
            while True:
                kind, data = self.events.get_nowait()
                try:
                    self._handle_event(kind, data)
                except Exception as exc:
                    self._chat_error(f"UI event error ({kind}): {exc}")
        except queue.Empty:
            pass
        if self._map_dirty:
            self._draw_map()
            self._map_dirty = False
        self.root.after(50, self._poll_events)

    def _handle_event(self, kind: str, data: dict[str, Any]) -> None:
        if kind == "protocol":
            self._log_protocol(data.get("direction", ""), data.get("text", ""))
        elif kind == "connected":
            self._on_connected(data["link"], data.get("mode", ""))
        elif kind == "connect_failed":
            self._on_connect_failed(data.get("message", "connect failed"))
        elif kind == "link_error":
            self._on_link_error(data.get("message", "link error"))
        elif kind == "map_dirty":
            self._map_dirty = True
        elif kind == "heading_changed":
            self._map_dirty = True
        elif kind == "assistant_start":
            self._chat_begin_assistant(int(data.get("step", 1)))
        elif kind == "assistant_delta":
            if self._chat_cur != "assistant":
                self._chat_section("assistant")
            self._chat_append(data.get("text", ""), "assistant")
        elif kind == "reasoning_delta":
            if self.config.show_reasoning:
                if self._chat_cur != "reasoning":
                    self._chat_section("reasoning")
                self._chat_append(data.get("text", ""), "reasoning")
        elif kind == "tool_call":
            self._chat_tool_call(data.get("name", "?"), data.get("arguments", ""))
        elif kind == "tool_result":
            self._chat_tool_result(data.get("name", "?"), data.get("text", ""),
                                   bool(data.get("ok", True)))
        elif kind == "error":
            self._chat_error(data.get("text", "error"))
        elif kind == "notice":
            self._chat_notice(data.get("text", ""))
        elif kind == "agent_busy":
            self.agent_busy = bool(data.get("busy", False))
            if not self.agent_busy:
                self.agent_thread = None
            self._update_control_states()
        elif kind == "agent_status":
            self._set_ai_status(data.get("text", "idle"))
        elif kind == "manual_busy":
            self.manual_busy = bool(data.get("busy", False))
            if not self.manual_busy:
                self._map_dirty = True
            self._update_control_states()
        elif kind == "manual_result":
            result = data.get("result") or {}
            if isinstance(result, dict) and result.get("event"):
                self._chat_notice(
                    f"Contact '{result['event']}' after {result.get('travelled_cm')} cm; "
                    "obstacle recorded and route cleared."
                )
        elif kind == "raw_result":
            result = data.get("result") or {}
            if not result.get("ok"):
                self._chat_error(f"Raw command failed: {result.get('error')}")

    def _on_connected(self, link: Any, mode: str) -> None:
        self.state.reset()
        with self.state.lock:
            self.state.connected = True
            self.state.mode = getattr(link, "mode", mode.lower())
        self.controller = CybotController(link, self.state, self.config,
                                          emit=self._emit,
                                          stop_event=self.agent_stop)
        self.agent_stop.clear()
        self._connecting = False
        self._refresh_connection_labels()
        self._update_control_states()
        self._map_dirty = True
        pretty = getattr(link, "mode", mode)
        self._chat_notice(f"Connected ({pretty}). Click the map to set a goal.")
        if not self.config.api_key:
            self._chat_notice(
                "No API key yet: paste one in Settings or set OPENROUTER_API_KEY."
            )

    def _on_connect_failed(self, message: str) -> None:
        self._connecting = False
        self._update_control_states()
        self.conn_label.configure(text="disconnected")
        self.link_dot.configure(fg=C["faint"])
        self._chat_error(f"Connection failed: {message}")

    def _on_link_error(self, message: str) -> None:
        self._chat_error(f"Link lost: {message}")
        self._set_ai_status("link lost")
        if self.controller is not None:
            try:
                self.controller.close()
            except Exception:
                pass
        self.controller = None
        with self.state.lock:
            self.state.connected = False
            self.state.mode = "disconnected"
        self._refresh_connection_labels()
        self._update_control_states()
        self._map_dirty = True

    # -- chat --------------------------------------------------------------

    def _chat_set_enabled(self) -> None:
        self.chat_text.configure(state="normal")

    def _chat_append(self, text: str, tag: str) -> None:
        if not text:
            return
        at_bottom = self._at_bottom()
        self._chat_set_enabled()
        self.chat_text.insert("end", text, tag)
        self._trim_chat()
        self.chat_text.configure(state="disabled")
        if at_bottom:
            self.chat_text.see("end")

    def _chat_section(self, kind: str) -> None:
        self._chat_set_enabled()
        if kind == "reasoning":
            self.chat_text.insert("end", "\n  [thinking] ", "reasoning")
        else:
            self.chat_text.insert("end", "\n", "assistant")
        self._chat_cur = kind
        self.chat_text.configure(state="disabled")

    def _chat_begin_assistant(self, step: int) -> None:
        at_bottom = self._at_bottom()
        self._chat_set_enabled()
        suffix = f" · step {step}" if step > 1 else ""
        self.chat_text.insert("end", f"\nCOPILOT{suffix}\n", "label")
        self._chat_cur = None
        self.chat_text.configure(state="disabled")
        if at_bottom:
            self.chat_text.see("end")

    def _chat_tool_call(self, name: str, args: str) -> None:
        at_bottom = self._at_bottom()
        pretty = args
        try:
            obj = json.loads(args)
            pretty = json.dumps(obj, separators=(",", ":"))
        except ValueError:
            pass
        if len(pretty) > 400:
            pretty = pretty[:400] + "…"
        self._chat_set_enabled()
        self.chat_text.insert("end", f"\n  → {name}({pretty})\n", "tool")
        self._chat_cur = None
        self.chat_text.configure(state="disabled")
        if at_bottom:
            self.chat_text.see("end")

    def _chat_tool_result(self, name: str, text: str, ok: bool) -> None:
        at_bottom = self._at_bottom()
        shown = text
        if len(shown) > 1600:
            shown = shown[:1600] + "…"
        self._chat_set_enabled()
        self.chat_text.insert("end", f"  ← {shown}\n",
                              "result" if ok else "error")
        self._chat_cur = None
        self.chat_text.configure(state="disabled")
        if at_bottom:
            self.chat_text.see("end")

    def _chat_user(self, text: str) -> None:
        at_bottom = self._at_bottom()
        self._chat_set_enabled()
        self.chat_text.insert("end", "\nYOU\n", "label")
        self.chat_text.insert("end", text + "\n", "user")
        self._chat_cur = None
        self.chat_text.configure(state="disabled")
        if at_bottom:
            self.chat_text.see("end")

    def _chat_error(self, text: str) -> None:
        at_bottom = self._at_bottom()
        self._chat_set_enabled()
        self.chat_text.insert("end", f"\n! {text}\n", "error")
        self._chat_cur = None
        self.chat_text.configure(state="disabled")
        if at_bottom:
            self.chat_text.see("end")

    def _chat_notice(self, text: str) -> None:
        at_bottom = self._at_bottom()
        self._chat_set_enabled()
        self.chat_text.insert("end", f"\n· {text}\n", "notice")
        self._chat_cur = None
        self.chat_text.configure(state="disabled")
        if at_bottom:
            self.chat_text.see("end")

    def _chat_welcome(self) -> None:
        self._chat_set_enabled()
        self.chat_text.insert(
            "end",
            "CYBOT LLM COPILOT\n"
            "The model drives through tools; you can stop it any time.\n"
            "• Click the field to set an A* goal, then 'AI: go to goal'.\n"
            "• Manual keys: W/S, A/D, Space. Use Scan to refresh the map.\n"
            "• Every scan refreshes the viewed area and re-localizes the robot.\n"
            "• Stale objects can be removed by the model by id; Clear all wipes the map.\n"
            "• After a bump/hole, back up 5-10 cm before scanning.\n",
            "notice",
        )
        self.chat_text.configure(state="disabled")

    def _trim_chat(self) -> None:
        try:
            lines = int(self.chat_text.index("end-1c").split(".")[0])
        except Exception:
            return
        if lines > 4000:
            self.chat_text.delete("1.0", "800.0")

    def _at_bottom(self) -> bool:
        try:
            return self.chat_text.yview()[1] > 0.97
        except tk.TclError:
            return True

    def _reset_chat(self) -> None:
        if self.agent_busy:
            self._chat_notice("Stop the agent before resetting the conversation.")
            return
        self.messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        self._chat_set_enabled()
        self.chat_text.delete("1.0", "end")
        self.chat_text.configure(state="disabled")
        self._chat_welcome()

    def _log_protocol(self, direction: str, text: str) -> None:
        lines = text.splitlines() or [""]
        if len(lines) > 14:
            shown = lines[:8] + [f"... {len(lines) - 11} lines omitted ..."] + lines[-3:]
        else:
            shown = lines
        tag = "tx" if direction == ">>" else "rx"
        self.proto_text.configure(state="normal")
        for line in shown:
            self.proto_text.insert("end", f"{direction} {line}\n", tag)
        self._trim_proto()
        self.proto_text.configure(state="disabled")
        self.proto_text.see("end")

    def _trim_proto(self) -> None:
        try:
            lines = int(self.proto_text.index("end-1c").split(".")[0])
        except Exception:
            return
        if lines > 3000:
            self.proto_text.delete("1.0", "600.0")

    # -- agent -------------------------------------------------------------

    def _send_from_event(self, _event: tk.Event) -> str:
        self._send_prompt()
        return "break"

    def _send_prompt(self) -> None:
        self._start_agent(self.prompt_text.get("1.0", "end").strip())

    def _ask_ai_goal(self) -> None:
        if self.controller is None:
            self._chat_notice("Connect first.")
            return
        with self.state.lock:
            goal = self.state.goal
        if goal is None:
            self._chat_notice("Click the map to set a goal first.")
            return
        self._start_agent(
            f"Navigate to the goal at ({goal[0]:.2f}, {goal[1]:.2f}) m. "
            "Scan when needed, and replan if the path is blocked."
        )

    def _start_agent(self, text: str) -> None:
        if self.agent_busy:
            self._chat_notice("The agent is already running.")
            return
        if self.controller is None or not self.state.connected:
            self._chat_error("No CyBot link. Connect first.")
            return
        if not text:
            return
        if not self.config.api_key:
            # Pick up a key typed into Settings without pressing Apply.
            self.config.api_key = self.key_var.get().strip()
            self._refresh_api_label()
        if not self.config.api_key:
            self._chat_error(
                "No OpenRouter API key. Paste one in Settings or set "
                "OPENROUTER_API_KEY, then send again."
            )
            self.notebook.select(2)
            self.key_entry.focus_set()
            return

        self.prompt_text.delete("1.0", "end")
        self._chat_user(text)
        self.agent_stop.clear()
        self.agent_busy = True
        self._set_ai_status("thinking")
        self._update_control_states()

        runtime = ToolRuntime(self.controller, self.agent_stop)
        runner = AgentRunner(self.config, self.messages, runtime,
                             self._emit, self.agent_stop)
        self.agent_thread = threading.Thread(
            target=runner.run, args=(text,), daemon=True
        )
        self.agent_thread.start()

    def _stop_agent(self) -> None:
        if not (self.agent_busy or self.manual_busy):
            return
        self.agent_stop.set()
        self._set_ai_status("stopping after current command")
        self._chat_notice(
            "Stop requested. A move already in progress will finish, then the "
            "agent will halt."
        )

    def _set_ai_status(self, text: str) -> None:
        self._ai_status = text
        try:
            self.agent_state_label.configure(text=f"agent: {text}")
        except tk.TclError:
            pass

    # -- manual controls ---------------------------------------------------

    def _manual(self, label: str, fn: Callable[[], dict[str, Any]]) -> None:
        if self.controller is None or not self.state.connected:
            self._chat_notice("Connect first.")
            return
        if self.agent_busy or self.manual_busy:
            return
        self.agent_stop.clear()
        self.manual_busy = True
        self._update_control_states()
        self._set_ai_status(f"manual: {label}")
        controller = self.controller

        def worker() -> None:
            try:
                result = fn()
                self._emit("manual_result", result=result)
                if isinstance(result, dict) and not result.get("ok", True):
                    self._emit("error", text=str(result.get("error")))
            except Exception as exc:
                self._emit("error", text=f"manual {label}: {exc}")
            finally:
                self._emit("manual_busy", busy=False)
                self._emit("agent_status", text="idle")

        threading.Thread(target=worker, daemon=True).start()

    def _manual_move(self, cm: float) -> None:
        if self.controller is not None:
            controller = self.controller
            self._manual("move", lambda: controller.move_cm(cm))

    def _manual_turn(self, deg: float) -> None:
        if self.controller is not None:
            controller = self.controller
            self._manual("turn", lambda: controller.turn_ccw(deg))

    def _manual_scan(self) -> None:
        if self.controller is not None:
            controller = self.controller
            self._manual("scan", controller.scan)

    def _manual_song(self) -> None:
        if self.controller is None:
            self._chat_notice("Connect first.")
            return
        if self.agent_busy or self.manual_busy:
            return
        controller = self.controller
        song = self.song_var.get()
        self._chat_notice(f"Playing {song}.")
        self._manual("song", lambda: controller.play_song(song))

    def _manual_flush(self) -> None:
        if self.controller is not None:
            controller = self.controller
            self._manual("flush", lambda: controller.raw_command("f"))

    def _clear_map(self) -> None:
        if self.controller is None:
            self._chat_notice("Connect first.")
            return
        if self.agent_busy or self.manual_busy:
            self._chat_notice("Stop the agent first.")
            return
        self.controller.clear_map()
        self._map_dirty = True
        self._chat_notice("All mapped objects cleared; pose kept.")

    def _send_raw(self) -> None:
        command = self.raw_var.get().strip()
        if not command or self.controller is None:
            return
        if self.agent_busy or self.manual_busy:
            self._chat_notice("Stop the current operation first.")
            return
        self.raw_var.set("")
        controller = self.controller
        self.manual_busy = True
        self._update_control_states()

        def worker() -> None:
            try:
                result = controller.raw_command(command)
                self._emit("raw_result", result=result)
            except Exception as exc:
                self._emit("error", text=f"raw command: {exc}")
            finally:
                self._emit("manual_busy", busy=False)

        threading.Thread(target=worker, daemon=True).start()

    def _on_key(self, event: tk.Event) -> Optional[str]:
        if self.agent_busy or self.manual_busy or self.controller is None:
            return None
        try:
            widget_class = event.widget.winfo_class()
        except Exception:
            return None
        if widget_class in ("Text", "Entry", "TEntry", "TCombobox", "TSpinbox"):
            return None
        key = event.keysym.lower()
        if key == "w" or key == "up":
            self._manual_move(25)
        elif key == "s" or key == "down":
            self._manual_move(-25)
        elif key == "a" or key == "left":
            self._manual_turn(30)
        elif key == "d" or key == "right":
            self._manual_turn(-30)
        elif key == "space":
            self._manual_scan()
        else:
            return None
        return "break"

    def _update_control_states(self) -> None:
        busy = self.agent_busy or self.manual_busy
        connected = self.controller is not None and self.state.connected
        locked = connected or self._connecting
        if self._connecting or busy:
            self.connect_btn.state(["disabled"])
        else:
            self.connect_btn.state(["!disabled"])
        self.mode_box.configure(state="disabled" if locked else "readonly")
        for entry in (self.host_entry, self.port_entry,
                      self.serial_entry, self.baud_entry):
            entry.state(["disabled"] if locked else ["!disabled"])
        for button in self.drive_buttons:
            if busy or not connected:
                button.state(["disabled"])
            else:
                button.state(["!disabled"])
        self.song_box.configure(
            state="disabled" if (busy or not connected) else "readonly"
        )
        if busy or not connected:
            self.send_btn.state(["disabled"])
            self.raw_entry.state(["disabled"])
            self.goal_ai_btn.state(["disabled"])
        else:
            self.send_btn.state(["!disabled"])
            self.raw_entry.state(["!disabled"])
            self.goal_ai_btn.state(["!disabled"])
        if self.agent_busy or self.manual_busy:
            self.stop_btn.state(["!disabled"])
        else:
            self.stop_btn.state(["disabled"])
        if self.agent_busy:
            self.reset_btn.state(["disabled"])
        else:
            self.reset_btn.state(["!disabled"])

    # -- settings ----------------------------------------------------------

    def _apply_settings(self) -> None:
        try:
            steps = int(float(self.steps_var.get()))
            every = int(float(self.every_var.get()))
            move_scale = float(self.move_scale_var.get())
            turn_scale = float(self.turn_scale_var.get())
            temperature = float(self.temp_var.get())
        except (ValueError, tk.TclError):
            self._chat_error("Settings: numeric field is invalid.")
            return
        if not 0.80 <= move_scale <= 1.20 or not 0.80 <= turn_scale <= 1.20:
            self._chat_error("Settings: trims must be between 0.80 and 1.20.")
            return
        self.config.api_key = self.key_var.get().strip()
        self.config.model = self.model_var.get().strip() or DEFAULT_MODEL
        self.config.reasoning_effort = self.reasoning_var.get()
        self.config.temperature = clamp(temperature, 0.0, 1.5)
        self.config.max_agent_steps = int(clamp(steps, 1, 30))
        self.config.scan_correction = bool(self.correction_var.get())
        self.config.correction_every = int(clamp(every, 0, 10))
        self.config.move_scale = move_scale
        self.config.turn_scale = turn_scale
        self.config.show_reasoning = bool(self.reasoning_show_var.get())
        self.config.show_grid_nodes = bool(self.nodes_var.get())
        self.config.show_scan_rays = bool(self.rays_var.get())
        self.config.sim_drift = bool(self.drift_var.get())
        self.model_label.configure(text=self.config.model)
        self._refresh_api_label()
        self._map_dirty = True
        self.settings_note.configure(
            text=f"Applied. Model={self.config.model}, trim={move_scale:.3f}/"
                 f"{turn_scale:.3f}, correction={'on' if self.config.scan_correction else 'off'}."
        )

    def _refresh_api_label(self) -> None:
        if self.config.api_key:
            self.api_status_label.configure(text="API key set", fg=C["accent2"])
        else:
            self.api_status_label.configure(text="API key missing", fg=C["red"])
        try:
            self.status_labels["api"].configure(
                text="set" if self.config.api_key else "missing",
                fg=C["accent2"] if self.config.api_key else C["red"],
            )
        except (AttributeError, KeyError):
            pass

    # -- map ---------------------------------------------------------------

    def _to_map(self, x: float, y: float) -> tuple[float, float]:
        return (MAP_BOUND / 2.0 + x * PPM, MAP_BOUND / 2.0 - y * PPM)

    def _from_map(self, px: float, py: float) -> tuple[float, float]:
        return ((px - MAP_BOUND / 2.0) / PPM, (MAP_BOUND / 2.0 - py) / PPM)

    def _on_map_motion(self, event: tk.Event) -> None:
        x, y = self._from_map(event.x, event.y)
        self.coord_label.configure(text=f"{x:+.2f}, {y:+.2f} m")

    def _on_map_click(self, event: tk.Event) -> None:
        if self.controller is None or not self.state.connected:
            self._chat_notice("Connect first.")
            return
        if self.agent_busy or self.manual_busy:
            self._chat_notice("Stop the agent before changing the goal.")
            return
        x, y = self._from_map(event.x, event.y)
        result = self.controller.set_goal(x, y)
        self._map_dirty = True
        if not result.get("ok"):
            self._chat_error(str(result.get("error")))
            return
        route = result.get("route", {})
        if result.get("reachable"):
            self._chat_notice(
                f"Goal ({x:+.2f}, {y:+.2f}) m · route "
                f"{route.get('distance_m', 0):.2f} m / "
                f"{len(route.get('waypoints', []))} waypoints."
            )
        else:
            self._chat_notice(
                f"Goal ({x:+.2f}, {y:+.2f}) m has no known clear route. "
                "Explore, scan, or clear the map."
            )

    def _draw_map(self) -> None:
        canvas = self.map_canvas
        canvas.delete("all")
        w = MAP_BOUND
        snap = self.state.snapshot()

        # grid
        for i in range(-6, 7):
            v = i * 0.5
            if abs(v) > FIELD_HALF:
                continue
            x0, y0 = self._to_map(v, -FIELD_HALF)
            x1, y1 = self._to_map(v, FIELD_HALF)
            canvas.create_line(x0, y0, x1, y1,
                               fill=C["grid_major"] if i == 0 else C["grid"],
                               width=1)
            x0, y0 = self._to_map(-FIELD_HALF, v)
            x1, y1 = self._to_map(FIELD_HALF, v)
            canvas.create_line(x0, y0, x1, y1,
                               fill=C["grid_major"] if i == 0 else C["grid"],
                               width=1)
        canvas.create_rectangle(1, 1, w - 1, w - 1, outline=C["line"], width=1)
        for i in range(-3, 4):
            px, py = self._to_map(i, -FIELD_HALF)
            canvas.create_text(px, w - 8, text=f"{i:+d}", fill=C["faint"],
                               font=("Segoe UI", 7))
            px, py = self._to_map(-FIELD_HALF, i)
            canvas.create_text(9, py, text=f"{i:+d}", fill=C["faint"],
                               font=("Segoe UI", 7), anchor="w")

        if self.config.show_grid_nodes:
            for i, (nx, ny) in enumerate(self.state.map.nodes):
                if i % 2:
                    continue
                px, py = self._to_map(nx, ny)
                canvas.create_oval(px - 1, py - 1, px + 1, py + 1,
                                   fill=C["faint"], outline="")

        # obstacles
        for ox, oy, r, kind in snap["obstacles"]:
            px, py = self._to_map(ox, oy)
            rp = max(r * PPM, 2.0)
            if kind == "hole":
                canvas.create_oval(px - rp, py - rp, px + rp, py + rp,
                                   fill="#12050a", outline=C["hole"], width=2,
                                   dash=(3, 2))
                canvas.create_line(px - 4, py - 4, px + 4, py + 4,
                                   fill=C["hole"], width=1)
                canvas.create_line(px - 4, py + 4, px + 4, py - 4,
                                   fill=C["hole"], width=1)
            elif kind == "boundary":
                canvas.create_oval(px - rp, py - rp, px + rp, py + rp,
                                   fill="#2a0710", outline="#ff8098", width=1)
            elif kind == "bump":
                halo = (r + ROBOT_RADIUS) * PPM
                canvas.create_oval(px - halo, py - halo, px + halo, py + halo,
                                   outline="#4a2a12", width=1)
                canvas.create_oval(px - rp, py - rp, px + rp, py + rp,
                                   fill=C["orange"], outline="#ffd6a5", width=1)
            else:
                halo = (r + ROBOT_RADIUS) * PPM
                canvas.create_oval(px - halo, py - halo, px + halo, py + halo,
                                   outline="#3a1c1c", width=1)
                canvas.create_oval(px - rp, py - rp, px + rp, py + rp,
                                   fill=C["obstacle"], outline="#ffb4a2", width=1)

        # travelled trail
        trail = snap["trail"]
        if len(trail) >= 2:
            pts: list[float] = []
            for tx, ty in trail:
                px, py = self._to_map(tx, ty)
                pts.extend((px, py))
            canvas.create_line(*pts, fill=C["trail"], width=2)

        # last scan rays
        if self.config.show_scan_rays and snap["last_scan"]:
            pose_x, pose_y, heading = snap["pose"]
            sx = pose_x + SERVO_RADIUS * math.cos(math.radians(heading))
            sy = pose_y + SERVO_RADIUS * math.sin(math.radians(heading))
            for det in snap["last_scan"]:
                ang = heading + det.angle_deg - 90.0
                ex = sx + det.distance_m * math.cos(math.radians(ang))
                ey = sy + det.distance_m * math.sin(math.radians(ang))
                p0 = self._to_map(sx, sy)
                p1 = self._to_map(ex, ey)
                canvas.create_line(*p0, *p1, fill="#1c3a52", width=1)

        # route
        route = snap["route"]
        if route:
            points = [self._to_map(x, y) for x, y in route]
            start = self._to_map(snap["pose"][0], snap["pose"][1])
            all_points = [start] + points
            for i in range(len(all_points) - 1):
                a, b = all_points[i], all_points[i + 1]
                last = i == len(all_points) - 2
                canvas.create_line(*a, *b, fill=C["route"], width=2,
                                   dash=(5, 4), arrow="last" if last else "none")
            for px, py in points:
                canvas.create_oval(px - 3, py - 3, px + 3, py + 3,
                                   fill=C["route"], outline="")

        # goal
        goal = snap["goal"]
        if goal is not None:
            gx, gy = self._to_map(goal[0], goal[1])
            for radius in (14, 9, 4):
                canvas.create_oval(gx - radius, gy - radius, gx + radius,
                                   gy + radius, outline=C["goal"], width=1)
            canvas.create_line(gx - 18, gy, gx + 18, gy, fill=C["goal"], width=1)
            canvas.create_line(gx, gy - 18, gx, gy + 18, fill=C["goal"], width=1)

        # robot
        rx, ry, heading = snap["pose"]
        rpx, rpy = self._to_map(rx, ry)
        rr = ROBOT_RADIUS * PPM
        canvas.create_oval(rpx - rr - 3, rpy - rr - 3, rpx + rr + 3, rpy + rr + 3,
                           outline="#51212b", width=1)
        canvas.create_oval(rpx - rr, rpy - rr, rpx + rr, rpy + rr,
                           fill=C["robot"], outline="#ffd1d8", width=1)
        hx, hy = unit(heading)
        canvas.create_line(rpx, rpy, rpx + hx * rr * 1.4, rpy - hy * rr * 1.4,
                           fill="white", width=2)
        canvas.create_oval(rpx - 2, rpy - 2, rpx + 2, rpy + 2,
                           fill="white", outline="")
        left = heading + 90.0
        right = heading - 90.0
        for ang in (left, right):
            lx, ly = unit(ang)
            canvas.create_line(rpx, rpy, rpx + lx * rr * 2.4, rpy - ly * rr * 2.4,
                               fill="#6b2a35", width=1, dash=(2, 3))

        # scale bar + legend
        sx0, sy0 = 14, w - 18
        canvas.create_line(sx0, sy0, sx0 + PPM, sy0, fill=C["dim"], width=2)
        canvas.create_text(sx0 + PPM / 2, sy0 - 8, text="1 m", fill=C["dim"],
                           font=FONT_SMALL)
        canvas.create_text(w - 10, 16, anchor="ne",
                           text="● obstacle   ● bump   ○ hole   route ┅   ● robot",
                           fill=C["faint"], font=FONT_SMALL)

        if self.controller is None:
            canvas.create_text(w / 2, 26, text="NOT CONNECTED",
                               fill=C["faint"], font=FONT_UI_BOLD)
        elif not self.state.connected:
            canvas.create_text(w / 2, 26, text="LINK LOST",
                               fill=C["red"], font=FONT_UI_BOLD)

    # -- status ------------------------------------------------------------

    def _refresh_status(self) -> None:
        try:
            if self.controller is None or not self.state.connected:
                self.status_labels["link"].configure(text="disconnected",
                                                     fg=C["faint"])
                self.status_labels["pose"].configure(text="—")
                self.status_labels["goal"].configure(text="—")
                self.status_labels["route"].configure(text="—")
                self.status_labels["corr"].configure(text="—")
                self.status_labels["ai"].configure(text=self._ai_status,
                                                   fg=C["dim"])
            else:
                snap = self.state.snapshot()
                x, y, heading = snap["pose"]
                mode = snap["mode"]
                self.status_labels["link"].configure(
                    text=mode, fg=C["accent2"] if mode != "disconnected" else C["faint"]
                )
                self.status_labels["pose"].configure(
                    text=f"({x:+.2f}, {y:+.2f})  {heading:03.0f}°"
                )
                goal = snap["goal"]
                if goal is None:
                    self.status_labels["goal"].configure(text="none", fg=C["dim"])
                    self.goal_label.configure(text="Goal: none", fg=C["dim"])
                    self._last_goal_text = "none"
                else:
                    text = f"({goal[0]:+.2f}, {goal[1]:+.2f}) m"
                    self.status_labels["goal"].configure(text=text, fg=C["goal"])
                    if text != self._last_goal_text:
                        self.goal_label.configure(text=f"Goal: {text}", fg=C["goal"])
                        self._last_goal_text = text
                route = snap["route"]
                if route:
                    self.status_labels["route"].configure(
                        text=f"{snap['route_distance']:.2f} m · {len(route)} wp"
                    )
                else:
                    self.status_labels["route"].configure(text="none", fg=C["dim"])
                corr = snap["last_correction"]
                if corr.applied:
                    self.status_labels["corr"].configure(
                        text=corr.summary(), fg=C["yellow"]
                    )
                elif corr.matched:
                    self.status_labels["corr"].configure(
                        text=f"{corr.matched} matches, no change", fg=C["dim"]
                    )
                else:
                    self.status_labels["corr"].configure(
                        text="no scan yet" if not snap["last_scan"] else "no landmarks",
                        fg=C["faint"],
                    )
                self.status_labels["ai"].configure(text=self._ai_status,
                                                   fg=C["accent"] if self.agent_busy else C["dim"])
            self.status_labels["api"].configure(
                text="set" if self.config.api_key else "missing",
                fg=C["accent2"] if self.config.api_key else C["red"],
            )
        except tk.TclError:
            return
        self.root.after(250, self._refresh_status)

    # -- close -------------------------------------------------------------

    def _on_close(self) -> None:
        self.agent_stop.set()
        if self.controller is not None:
            try:
                self.controller.close()
            except Exception:
                pass
        self.root.destroy()


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="CyBot OpenRouter LLM copilot")
    parser.add_argument("--tcp", action="store_true",
                        help="select TCP mode and pre-fill host/port")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--serial", metavar="PORT", default=None,
                        help="select Serial mode with this port (e.g. COM3)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--model", default=None,
                        help=f"OpenRouter model id (default {DEFAULT_MODEL})")
    parser.add_argument("--no-autoconnect", action="store_true",
                        help="do not auto-connect the simulator at launch")
    args = parser.parse_args()

    root = tk.Tk()
    app = CyBotApp(root, args)
    if args.tcp:
        app.mode_var.set("TCP")
        app._sync_conn_fields()
        app._refresh_connection_labels()
    elif args.serial:
        app.mode_var.set("Serial")
        app._sync_conn_fields()
        app._refresh_connection_labels()
    else:
        app.mode_var.set("Simulated")
        app._sync_conn_fields()
    root.mainloop()


if __name__ == "__main__":
    main()

