"""
ros2_config.py

This script will:
1. analyze our ros2 topics
2. build configurations for what is supported
3. give options for other elements
4. produce sample message conversions?
"""
import argparse


def parse():
    parser = argparse.ArgumentParser(description='Viam configurator for ROS2')
    parser.add_argument('-n', '--namespace', default='', type=str)


def main():
    pass


if __name__ == '__main__':
    main()
