import time
import numpy as np


def samples(a, b, resolution=0.01):
    count = max(1, int(np.ceil(np.max(np.abs(b - a)) / resolution)))
    return np.linspace(a, b, count + 1)


def plan_joint_route(start, goal, valid, lower, upper, *, pulse=lambda: None,
                     timeout=12.0, iterations=1600):
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    lower, upper = np.asarray(lower, float), np.asarray(upper, float)
    deadline = time.monotonic() + timeout
    checks = 0

    def edge(a, b):
        nonlocal checks
        for q in samples(a, b):
            checks += 1
            if checks % 24 == 0:
                pulse()
            if time.monotonic() > deadline:
                return False
            if not valid(q):
                return False
        return True

    if not valid(start) or not valid(goal):
        return None
    if edge(start, goal):
        return samples(start, goal)
    rng = np.random.default_rng(42)
    # First try small local diversions. This often finds a short route around
    # the deliberately inserted sphere without exploring the entire workspace.
    middle = (start + goal) / 2
    for magnitude in (.18, .35, .6):
        for joint in range(len(start)):
            for sign in (1, -1):
                via = middle.copy()
                via[joint] += sign * magnitude
                via = np.clip(via, lower, upper)
                if edge(start, via) and edge(via, goal):
                    return np.vstack((samples(start, via), samples(via, goal)[1:]))
                shift = np.zeros(len(start))
                shift[joint] = sign * magnitude
                first = np.clip(start + shift, lower, upper)
                second = np.clip(goal + shift, lower, upper)
                if edge(start, first) and edge(first, second) and edge(second, goal):
                    return np.vstack((samples(start, first), samples(first, second)[1:],
                                      samples(second, goal)[1:]))
                if time.monotonic() > deadline:
                    return None

    trees = [([start], [-1]), ([goal], [-1])]

    def extend(tree, target):
        points, parents = tree
        nearest = int(np.argmin(np.linalg.norm(np.asarray(points) - target, axis=1)))
        delta = target - points[nearest]
        scale = min(1.0, .22 / max(np.linalg.norm(delta), 1e-12))
        candidate = points[nearest] + scale * delta
        if not edge(points[nearest], candidate):
            return None
        points.append(candidate)
        parents.append(nearest)
        return len(points) - 1

    def branch(tree, index):
        result = []
        while index != -1:
            result.append(tree[0][index])
            index = tree[1][index]
        return result[::-1]

    for iteration in range(iterations):
        pulse()
        if time.monotonic() > deadline:
            return None
        side = iteration % 2
        first, second = trees[side], trees[1 - side]
        # Local sampling keeps recovery routes near the original task.
        target = (second[0][0] if iteration % 5 == 0 else
                  np.clip(middle + rng.normal(0, .65, len(start)), lower, upper))
        index = extend(first, target)
        if index is None:
            continue
        meet = first[0][index]
        for _ in range(40):
            other = extend(second, meet)
            if other is None:
                break
            if np.linalg.norm(second[0][other] - meet) < 1e-9:
                left, right = branch(first, index), branch(second, other)
                knots = left + right[-2::-1]
                if side:
                    knots.reverse()
                # Shortcut only through fully checked edges.
                result = [knots[0]]
                cursor = 0
                while cursor < len(knots) - 1:
                    following = cursor + 1
                    for candidate in range(len(knots) - 1, cursor + 1, -1):
                        if edge(knots[cursor], knots[candidate]):
                            following = candidate
                            break
                    result.extend(samples(knots[cursor], knots[following])[1:])
                    cursor = following
                # A timeout during shortcutting must never turn a failed check
                # into permission to animate an unchecked edge.
                route = np.asarray(result)
                return route if all(valid(q) for q in route) else None
    return None
