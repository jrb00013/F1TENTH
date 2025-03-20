# tests/test_behavior_planner.py
import pytest
from src.behavior_planner import BehaviorPlanner  # Adjust import based on actual class

def test_behavior_planner_initialization():
    planner = BehaviorPlanner()
    assert planner is not None

def test_planning_function():
    planner = BehaviorPlanner()
    # Assuming there's a method called plan() that returns some result
    result = planner.plan()
    assert result is not None
    assert isinstance(result, dict)  # Modify according to expected return type
