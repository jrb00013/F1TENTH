# tests/test_aeb.py
import pytest
from src.aeb import AEBSystem  # Adjust import based on actual class

def test_aeb_activation():
    aeb = AEBSystem()
    # Assuming there's an `activate` method in the AEB system
    assert aeb.activate() is True  # Modify depending on behavior
