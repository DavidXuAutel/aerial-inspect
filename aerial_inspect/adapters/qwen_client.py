"""Optional Qwen task decompose client (H100 service)."""
from __future__ import annotations

import json
import os
import re
from typing import Any, Dict, Optional

import httpx

from aerial_inspect.mission.schema import MissionSpec, SearchArea, TargetSpec


def decompose_url() -> Optional[str]:
    return os.environ.get("QWEN_DECOMPOSE_URL")


def _extract_json(text: str) -> Dict[str, Any]:
    text = text.strip()
    fence = re.search(r"```(?:json)?\s*([\s\S]*?)```", text)
    if fence:
        text = fence.group(1).strip()
    return json.loads(text)


def instruction_to_mission_fields(instruction: str) -> Dict[str, Any]:
    """Call Qwen /api/decompose or return template on failure."""
    url = decompose_url()
    if not url:
        return {
            "instruction": instruction,
            "target": {"visual_prompt": "bridge", "category": "bridge"},
            "search_area": {"center_xy": [0.0, 0.0], "radius_m": 100.0, "altitude_m": 40.0},
        }
    with httpx.Client(timeout=60.0) as client:
        r = client.post(url, json={"instruction": instruction})
        r.raise_for_status()
        body = r.json()
    if "task_type" in body:
        return body
    if "choices" in body:
        content = body["choices"][0]["message"]["content"]
        return _extract_json(content)
    return body


def merge_decompose_into_spec(base: MissionSpec, decomposed: Dict[str, Any]) -> MissionSpec:
    sa = decomposed.get("search_area") or {}
    tg = decomposed.get("target") or {}
    fc = decomposed.get("follow_config") or {}
    return MissionSpec(
        mission_id=base.mission_id,
        instruction=str(decomposed.get("instruction", base.instruction)),
        search=SearchArea(
            center_xy=tuple(sa.get("center_xy", base.search.center_xy)),
            radius_m=float(sa.get("radius_m", base.search.radius_m)),
            altitude_m=float(sa.get("altitude_m", sa.get("altitude_m", base.search.altitude_m))),
        ),
        target=TargetSpec(
            visual_prompt=str(tg.get("visual_prompt", base.target.visual_prompt)),
            category=str(tg.get("category", base.target.category)),
            standoff_dist_m=float(fc.get("standoff_dist_m", base.target.standoff_dist_m)),
            standoff_height_m=float(fc.get("standoff_height_m", base.target.standoff_height_m)),
        ),
        survey=base.survey,
        capture=base.capture,
        reconstruct=base.reconstruct,
        bridge_centroid_xyz=base.bridge_centroid_xyz,
    )
