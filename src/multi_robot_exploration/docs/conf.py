# Configuration file for Sphinx documentation builder

import os
import sys

# Add the package path
sys.path.insert(0, os.path.abspath('../../multi_robot_exploration'))

# Mock imports for packages that aren't available
autodoc_mock_imports = [
    'rclpy',
    'rclpy.node',
    'rclpy.qos',
    'rclpy.action',
    'rclpy.callback_groups',
    'rclpy.executors',
    'sensor_msgs',
    'sensor_msgs.msg',
    'geometry_msgs',
    'geometry_msgs.msg',
    'nav_msgs',
    'nav_msgs.msg',
    'nav2_msgs',
    'nav2_msgs.action',
    'std_msgs',
    'std_msgs.msg',
    'tf2_ros',
    'tf_transformations',
    'cv_bridge',
    'numpy',
    'scipy',
    'sklearn',
    'cv2',
    'visualization_msgs',
    'visualization_msgs.msg',
    'sensor_msgs_py',
    'sensor_msgs_py.point_cloud2',
    'open3d',
]

# Project information
project = 'Multi Robot Exploration'
copyright = '2026'
author = 'VA50'

# Extensions
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
]

# Theme
html_theme = 'sphinx_rtd_theme'

# Autodoc settings
autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'special-members': '__init__',
    'undoc-members': True,
    'exclude-members': '__weakref__'
}
