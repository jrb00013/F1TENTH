# tests/test_lane_detection.py
import pytest
from src.lane_detection import LaneDetection  # Adjust import based on actual class

def test_lane_detection_initialization():
    lane_detector = LaneDetection()
    assert lane_detector is not None

def test_lane_detection_function():
    lane_detector = LaneDetection()
    # Assuming there’s a detect_lanes() function
    lanes = lane_detector.detect_lanes("image_path.jpg")
    assert lanes is not None
    assert len(lanes) > 0
