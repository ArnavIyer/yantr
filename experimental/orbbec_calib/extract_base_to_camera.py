#!/usr/bin/env python3

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Optional
import yaml
import sys
import numpy as np
from PIL import Image
from pupil_apriltags import Detector


HERE = Path(__file__).resolve().parent
DEFAULT_IMAGE = HERE / "extracted_2026-06-20_17-21-09.588_EDT.png"
DEFAULT_INTRINSICS = HERE / "extracted_2026-06-20_17-21-09.588_EDT_camera_intrinsics.yaml"
TAG_TO_BASE_LINK = np.ndarray(
    [
        [0,0,1,.3175],
        [-1,0,0,.1913],
        [0,-1,0,.832],
        [0,0,0,1]
    ]
)

@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    @property
    def pupil_camera_params(self) -> tuple[float, float, float, float]:
        """pupil_apriltags expects (fx, fy, cx, cy)."""
        return (self.fx, self.fy, self.cx, self.cy)


def load_ros_camera_info_yaml(path: Path) -> CameraIntrinsics:
    """
    Load the ROS camera_info YAML.
    """

    with open(path, "r") as file:
        data = yaml.safe_load(file)

    mat = data["camera_matrix"]["data"]

    return CameraIntrinsics(fx=mat[0], fy=mat[4], cx=mat[2], cy=mat[5])


def load_grayscale_u8(path: Path) -> np.ndarray:
    """Load an RGB/RGBA/grayscale image and return uint8 grayscale."""
    with Image.open(path) as image:
        gray = image.convert("L")
        return np.ascontiguousarray(np.array(gray, dtype=np.uint8))


def detect_one_tag(
    gray: np.ndarray,
    intrinsics: CameraIntrinsics,
    tag_size_m: float,
    tag_family: str,
    expected_tag_id: Optional[int],
):
    """
    Detect one AprilTag and ask the library to estimate its pose.

    The tag detection library follows the following convention. 
    For the camera pose, from the view of the camera,
    +x is right
    +y is down
    +z is forward out of the camera
    
    the april tag is the same, just that the xy plane is the same as the plane of the tag

    - physical tag size: 80mm
    - total size 100mm
    - tag family: tag36h11
    """
    detector = Detector(families=tag_family)
    detections = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=intrinsics.pupil_camera_params,
        tag_size=tag_size_m,
    )

    if expected_tag_id is not None:
        detections = [d for d in detections if d.tag_id == expected_tag_id]

    if len(detections) != 1:
        raise RuntimeError(f"Expected exactly one tag, found {len(detections)}")

    return detections[0]

def solve_base_to_camera(
    image_path: Path,
    intrinsics_path: Path,
    tag_size_m: float,
    tag_family: str,
    expected_tag_id: Optional[int],
) -> np.ndarray:
    intrinsics = load_ros_camera_info_yaml(intrinsics_path)
    gray = load_grayscale_u8(image_path)
    detection = detect_one_tag(gray, intrinsics, tag_size_m, tag_family, expected_tag_id)

    tag_to_camera = np.eye(4)
    tag_to_camera[0:3, 0:3] = detection.pose_R
    tag_to_camera[0:3, 3] = detection.pose_t.flatten()
    
    print(tag_to_camera)

    camera_to_tag = np.linalg.inv(tag_to_camera)

    camera_to_base_link = TAG_TO_BASE_LINK @ camera_to_tag

    return np.linalg.inv(camera_to_base_link)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Skeleton for estimating base_link -> camera_link from an AprilTag image."
    )
    parser.add_argument("--image", type=Path, default=DEFAULT_IMAGE)
    parser.add_argument("--intrinsics", type=Path, default=DEFAULT_INTRINSICS)
    parser.add_argument("--tag-family", default="tag36h11")
    parser.add_argument("--tag-size-m", type=float, required=True)
    parser.add_argument("--tag-id", type=int, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    base_link_to_camera = solve_base_to_camera(
        image_path=args.image,
        intrinsics_path=args.intrinsics,
        tag_size_m=args.tag_size_m,
        tag_family=args.tag_family,
        expected_tag_id=args.tag_id,
    )

    print(base_link_to_camera)


if __name__ == "__main__":
    main()
