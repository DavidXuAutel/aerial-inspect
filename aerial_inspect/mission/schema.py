"""Mission specification types."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple


class MissionPhase(str, Enum):
    SEARCH = "search"
    APPROACH = "approach"
    SURVEY = "survey"
    DONE = "done"


@dataclass
class SearchArea:
    center_xy: Tuple[float, float]
    radius_m: float = 80.0
    altitude_m: float = 40.0


@dataclass
class TargetSpec:
    visual_prompt: str = "bridge"
    category: str = "bridge"
    standoff_dist_m: float = 25.0
    standoff_height_m: float = 15.0


@dataclass
class SurveySpec:
    """Photogrammetry-style orbit around bridge centroid."""

    pattern: str = "horizontal_orbit"  # horizontal_orbit | facade_arc
    radius_m: float = 35.0
    altitude_m: float = 40.0
    num_laps: int = 2
    points_per_lap: int = 24
    heading_overlap: float = 0.75
    facade_arc_deg: float = 120.0
    approach_bearing_deg: float = 90.0
    camera_hfov_deg: float = 80.0


@dataclass
class CaptureSpec:
    native_width: int = 1280
    native_height: int = 720
    jpeg_quality: int = 92
    min_frames: int = 120


@dataclass
class ReconstructSpec:
    backend: str = "colmap"  # colmap | none
    min_track_length: int = 3
    require_gps: bool = False


@dataclass
class MissionSpec:
    mission_id: str
    instruction: str
    search: SearchArea
    target: TargetSpec
    survey: SurveySpec
    capture: CaptureSpec = field(default_factory=CaptureSpec)
    reconstruct: ReconstructSpec = field(default_factory=ReconstructSpec)
    bridge_centroid_xyz: Optional[Tuple[float, float, float]] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MissionSpec":
        sa = data.get("search_area") or data.get("search") or {}
        tg = data.get("target") or {}
        sv = data.get("survey") or {}
        cap = data.get("capture") or {}
        rc = data.get("reconstruct") or {}
        centroid = data.get("bridge_centroid_xyz")
        return cls(
            mission_id=str(data.get("mission_id", "mission_001")),
            instruction=str(data.get("instruction", "")),
            search=SearchArea(
                center_xy=tuple(sa.get("center_xy", [0.0, 0.0])),
                radius_m=float(sa.get("radius_m", 80.0)),
                altitude_m=float(sa.get("altitude_m", 40.0)),
            ),
            target=TargetSpec(
                visual_prompt=str(tg.get("visual_prompt", "bridge")),
                category=str(tg.get("category", "bridge")),
                standoff_dist_m=float(tg.get("standoff_dist_m", 25.0)),
                standoff_height_m=float(tg.get("standoff_height_m", 15.0)),
            ),
            survey=SurveySpec(
                pattern=str(sv.get("pattern", "horizontal_orbit")),
                radius_m=float(sv.get("radius_m", 35.0)),
                altitude_m=float(sv.get("altitude_m", 40.0)),
                num_laps=int(sv.get("num_laps", 2)),
                points_per_lap=int(sv.get("points_per_lap", 24)),
                heading_overlap=float(sv.get("heading_overlap", 0.75)),
                facade_arc_deg=float(sv.get("facade_arc_deg", 120.0)),
                approach_bearing_deg=float(sv.get("approach_bearing_deg", 90.0)),
                camera_hfov_deg=float(sv.get("camera_hfov_deg", 80.0)),
            ),
            capture=CaptureSpec(
                native_width=int(cap.get("native_width", 1280)),
                native_height=int(cap.get("native_height", 720)),
                jpeg_quality=int(cap.get("jpeg_quality", 92)),
                min_frames=int(cap.get("min_frames", 120)),
            ),
            reconstruct=ReconstructSpec(
                backend=str(rc.get("backend", "colmap")),
                min_track_length=int(rc.get("min_track_length", 3)),
                require_gps=bool(rc.get("require_gps", False)),
            ),
            bridge_centroid_xyz=tuple(centroid) if centroid else None,
        )
