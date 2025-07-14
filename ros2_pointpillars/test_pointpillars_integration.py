#!/usr/bin/env python3
"""
Test script to verify PointPillars integration with ROS2 package
"""

import sys
import os
import numpy as np
import torch

# Add the pointpillars directory to the path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
pointpillars_path = os.path.join(project_root, 'pointpillars')
sys.path.insert(0, pointpillars_path)
sys.path.insert(0, project_root)  # Add project root to path
print(f"Python path: {sys.path}")

# Debug: Print the path being added
print(f"Adding to Python path: {pointpillars_path}")
print(f"Path exists: {os.path.exists(pointpillars_path)}")
print(f"Project root: {project_root}")

def test_pointpillars_imports():
    """Test if all PointPillars imports work correctly"""
    try:
        from pointpillars.model import PointPillars
        from pointpillars.utils import setup_seed, vis_pc, vis_img_3d
        from pointpillars.utils.process import bbox3d2corners_camera, points_camera2image
        from pointpillars.dataset import point_range_filter
        from pointpillars.ops import Voxelization, nms_cuda
        
        print("✅ All PointPillars imports successful")
        return True
    except ImportError as e:
        print(f"❌ Import error: {e}")
        return False

def test_pointpillars_model_creation():
    """Test if PointPillars model can be created"""
    try:
        from pointpillars.model import PointPillars
        
        # Test model creation with default parameters
        model = PointPillars(nclasses=3)
        print("✅ PointPillars model creation successful")
        
        # Test model creation with custom parameters
        point_cloud_range = [0, -39.68, -3, 69.12, 39.68, 1]
        model = PointPillars(nclasses=3, point_cloud_range=point_cloud_range)
        print("✅ PointPillars model creation with custom parameters successful")
        
        return True
    except Exception as e:
        print(f"❌ Model creation error: {e}")
        return False

def test_pointpillars_inference():
    """Test if PointPillars model can run inference"""
    try:
        from pointpillars.model import PointPillars
        
        # Create model
        model = PointPillars(nclasses=3)
        model.eval()
        
        # Create dummy point cloud data
        num_points = 1000
        points = torch.randn(num_points, 4)  # x, y, z, intensity
        
        # Run inference
        with torch.no_grad():
            results = model(batched_pts=[points], mode='test')
            result = results[0]
        
        print(f"✅ PointPillars inference successful")
        print(f"   - Input points: {num_points}")
        print(f"   - Output detections: {len(result['lidar_bboxes'])}")
        
        return True
    except Exception as e:
        print(f"❌ Inference error: {e}")
        return False

def test_point_range_filter():
    """Test point range filtering"""
    try:
        from pointpillars.dataset import point_range_filter
        
        # Create dummy data
        points = np.random.randn(1000, 4)
        data_dict = {'pts': points}
        point_range = [0, -39.68, -3, 69.12, 39.68, 1]
        
        # Apply filter
        filtered_data = point_range_filter(data_dict, point_range)
        filtered_points = filtered_data['pts']
        
        print(f"✅ Point range filtering successful")
        print(f"   - Original points: {len(points)}")
        print(f"   - Filtered points: {len(filtered_points)}")
        
        return True
    except Exception as e:
        print(f"❌ Point range filtering error: {e}")
        return False

def main():
    """Run all tests"""
    print("Testing PointPillars integration...")
    print("=" * 50)
    
    tests = [
        test_pointpillars_imports,
        test_pointpillars_model_creation,
        test_pointpillars_inference,
        test_point_range_filter
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
        print()
    
    print("=" * 50)
    print(f"Tests passed: {passed}/{total}")
    
    if passed == total:
        print("🎉 All tests passed! PointPillars integration is working correctly.")
        return 0
    else:
        print("❌ Some tests failed. Please check the errors above.")
        return 1

if __name__ == "__main__":
    sys.exit(main()) 