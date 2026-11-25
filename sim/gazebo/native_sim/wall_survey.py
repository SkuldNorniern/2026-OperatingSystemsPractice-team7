"""Trace the room perimeter using only the currently visible wall segments."""
from dataclasses import dataclass
import math

import numpy as np


def cross(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


@dataclass(frozen=True)
class Wall:
    start: np.ndarray
    direction: np.ndarray
    length: float

    @property
    def inward(self):
        return np.array([-self.direction[1], self.direction[0]])

    def station(self, point):
        return float((point - self.start) @ self.direction)

    def position(self, station):
        return self.start + station * self.direction

    def distance(self, point):
        return float(np.linalg.norm(point - self.position(np.clip(self.station(point), 0, self.length))))


def visible_walls(scan, pose):
    ranges = np.asarray(scan.ranges, dtype=float)
    indices = np.flatnonzero(np.isfinite(ranges) & (ranges > max(.18, scan.range_min))
                             & (ranges < scan.range_max - .08))
    if indices.size < 5:
        return []
    position = np.asarray(pose[:2])
    angles = pose[2] + scan.angle_min + indices * scan.angle_step
    points = position + ranges[indices, None] * np.column_stack((np.cos(angles), np.sin(angles)))
    gaps = (np.diff(indices) * scan.angle_step > .09) | (np.linalg.norm(np.diff(points, axis=0), axis=1) > .40)
    pieces = list(np.split(points, np.flatnonzero(gaps) + 1))
    wraps = (len(ranges) * scan.angle_step > 2 * math.pi - .10
             and (len(ranges) + indices[0] - indices[-1]) * scan.angle_step < .09)
    if len(pieces) > 1 and wraps and np.linalg.norm(points[0] - points[-1]) < .40:
        pieces[0] = np.concatenate((pieces.pop(), pieces[0]))

    walls = []
    while pieces:
        part = pieces.pop()
        if len(part) < 5:
            continue
        chord = part[-1] - part[0]
        span = float(np.linalg.norm(chord))
        if span < .70:
            if len(part) >= 10 and np.linalg.norm(np.ptp(part, axis=0)) >= .70:
                middle = len(part) // 2
                pieces.extend((part[:middle + 1], part[middle:]))
            continue
        direction = chord / span
        normal = np.array([-direction[1], direction[0]])
        offsets = np.abs((part - part[0]) @ normal)
        split = int(np.argmax(offsets))
        if offsets[split] > .10:
            pieces.extend((part[:split + 1], part[split:]))
            continue
        center = np.mean(part, axis=0)
        _, _, vectors = np.linalg.svd(part - center, full_matrices=False)
        direction = vectors[0]
        if cross(direction, position - center) < 0:
            direction = -direction
        along = (part - center) @ direction
        walls.append(Wall(center + min(along) * direction, direction, float(np.ptp(along))))
    return walls


@dataclass(frozen=True)
class WallStep:
    target: tuple[float, float]
    next_wall: Wall | None = None
    corner: tuple[float, float] | None = None
    closing: bool = False


class WallSurvey:
    lookahead = 1.6

    def __init__(self, standoff):
        self.standoff = standoff
        self.wall = None
        self.home = self.home_direction = None
        self.corners = []
        self.complete = False

    def step(self, scan, pose):
        position = np.asarray(pose[:2])
        walls = visible_walls(scan, pose)
        if self.wall is None:
            self.wall = min(walls, key=lambda wall: wall.distance(position), default=None)
        else:
            matching = [wall for wall in walls if wall.direction @ self.wall.direction > .93
                        and abs((wall.start - self.wall.start) @ self.wall.inward) < .25]
            self.wall = min(matching, key=lambda wall: wall.distance(position), default=self.wall)
            if not matching:
                return None
        wall = self.wall
        if wall is None:
            return None

        along = wall.station(position)
        if self.home is None:
            point = wall.position(np.clip(along, 0, wall.length)) + self.standoff * wall.inward
            return WallStep(tuple(map(float, point)))

        limit = wall.length
        if len(self.corners) == 4:
            if wall.direction @ self.home_direction < .90:
                return None
            home_along = wall.station(self.home)
            if home_along - along <= self.lookahead:
                return WallStep(self.home, closing=True)
            limit = min(limit, home_along)
        else:
            corner = self.next_corner(wall, walls, along)
            if corner is not None:
                target_along, turn = corner
                if target_along - along <= self.lookahead:
                    return turn
                limit = min(limit, target_along)

        advance = min(limit, along + self.lookahead)
        if advance - along < .35:
            return None
        target = wall.position(advance) + self.standoff * wall.inward
        return WallStep(tuple(map(float, target)))

    def next_corner(self, wall, walls, along):
        turns = []
        for following in walls:
            turn = cross(wall.direction, following.direction)
            if turn < .75:
                continue
            intersection = cross(following.start - wall.start, following.direction) / turn
            if intersection > wall.length + .55 or intersection < along - .35:
                continue
            corner = wall.position(intersection)
            if not -.55 < following.station(corner) < .65:
                continue
            if any(math.dist(corner, previous) < 1.0 for previous in self.corners):
                continue
            bisector = wall.inward + following.inward
            target = corner + self.standoff * bisector / (1 + wall.inward @ following.inward)
            station = wall.station(target)
            turns.append((station, WallStep(tuple(map(float, target)), following, tuple(map(float, corner)))))
        return min(turns, key=lambda pair: pair[0], default=None)

    def reached(self, step):
        if self.home is None:
            self.home = step.target
            self.home_direction = self.wall.direction
        elif step.closing:
            self.complete = True
        elif step.next_wall is not None:
            self.corners.append(step.corner)
            self.wall = step.next_wall

    def bounds(self):
        xs, ys = zip(*self.corners)
        return [min(xs), max(xs), min(ys), max(ys)]
