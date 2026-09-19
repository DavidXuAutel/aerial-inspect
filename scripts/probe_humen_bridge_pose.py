#!/usr/bin/env python3
"""Find Humen bridge mesh centroid in AirSim (for correcting search area)."""
from __future__ import annotations

import argparse
import json
import math
import os
from collections import defaultdict
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--host", default=os.environ.get("AIRSIM_HOST", "127.0.0.1"))
    p.add_argument("--port", type=int, default=int(os.environ.get("AIRSIM_PORT", "41463")))
    p.add_argument("--pattern", default=".*HumenCorridor.*")
    p.add_argument("--near-x", type=float, default=-950.0)
    p.add_argument("--near-y", type=float, default=-60.0)
    p.add_argument("--radius-m", type=float, default=800.0)
    p.add_argument("--out", help="write JSON report")
    args = p.parse_args()

    import airsim

    client = airsim.MultirotorClient(ip=args.host, port=args.port)
    client.confirmConnection()
    objs = client.simListSceneObjects(args.pattern)
    clusters: dict[tuple[float, float, float], list[str]] = defaultdict(list)
    near: list[tuple[float, str, float, float, float]] = []

    for name in objs:
        try:
            pos = client.simGetObjectPose(name).position
            x, y, z = float(pos.x_val), float(pos.y_val), -float(pos.z_val)
        except Exception:
            continue
        key = (round(x, 0), round(y, 0), round(z, 0))
        clusters[key].append(name)
        d = math.hypot(x - args.near_x, y - args.near_y)
        if d <= args.radius_m:
            near.append((d, name, x, y, z))

    near.sort(key=lambda t: t[0])
    deckish = [
        t for t in near
        if any(k in t[1].lower() for k in ("deck", "tower", "pylon", "cable", "girder", "main", "span"))
    ]

    report = {
        "n_objects": len(objs),
        "n_clusters": len(clusters),
        "top_clusters": [
            {"pose": k, "count": len(v), "sample": v[0]}
            for k, v in sorted(clusters.items(), key=lambda kv: -len(kv[1]))[:10]
        ],
        "near_spawn": [
            {"dist_m": round(t[0], 1), "name": t[1], "xyz": [round(t[2], 1), round(t[3], 1), round(t[4], 1)]}
            for t in near[:30]
        ],
        "deckish_near_spawn": [
            {"dist_m": round(t[0], 1), "name": t[1], "xyz": [round(t[2], 1), round(t[3], 1), round(t[4], 1)]}
            for t in deckish[:30]
        ],
    }

    if deckish:
        xs = [t[2] for t in deckish]
        ys = [t[3] for t in deckish]
        zs = [t[4] for t in deckish]
        report["suggested_centroid_xyz"] = [
            round(sum(xs) / len(xs), 1),
            round(sum(ys) / len(ys), 1),
            round(sum(zs) / len(zs), 1),
        ]
    elif near:
        t = near[0]
        report["suggested_centroid_xyz"] = [round(t[2], 1), round(t[3], 1), round(t[4], 1)]

    text = json.dumps(report, indent=2, ensure_ascii=False)
    print(text)
    if args.out:
        Path(args.out).write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
