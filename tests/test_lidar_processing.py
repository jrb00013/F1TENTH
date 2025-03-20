# tests/test_lidar_processing.py
import pytest
from src.lidar_processing import LidarProcessing  # Adjust import based on actual class

def test_lidar_processing_initialization():
    lidar_processor = LidarProcessing()
    assert lidar_processor is not None

def test_lidar_processing_function():
    lidar_processor = LidarProcessing()
    processed_data = lidar_processor.process_data("lidar_data_file")
    assert processed_data is not None
    assert isinstance(processed_data, list)  # Modify according to the return type
