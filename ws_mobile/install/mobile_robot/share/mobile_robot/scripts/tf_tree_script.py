#!/usr/bin/env python3

"""
Script to generate TF tree visualization for the mobile robot
Run this after launching the robot simulation
"""

import os
import subprocess
import time

def generate_tf_tree():
    """Generate TF tree PDF and view frames"""
    
    print("🤖 Generating TF Tree for Mobile Robot...")
    print("=" * 50)
    
    # Wait a moment for all transforms to be published
    print("⏳ Waiting for TF transforms to stabilize...")
    time.sleep(3)
    
    try:
        # Generate the TF tree
        print("📊 Creating TF tree visualization...")
        subprocess.run(['ros2', 'run', 'tf2_tools', 'view_frames'], 
                      check=True, capture_output=False)
        
        # Check if frames.pdf was created
        if os.path.exists('frames.pdf'):
            print("✅ TF tree successfully generated as 'frames.pdf'")
            
            # Try to open the PDF (works on most Linux systems)
            try:
                subprocess.run(['xdg-open', 'frames.pdf'], check=True)
                print("🖼️  Opening TF tree visualization...")
            except subprocess.CalledProcessError:
                print("📁 PDF created but couldn't auto-open. Please open 'frames.pdf' manually.")
        else:
            print("❌ Failed to generate frames.pdf")
            
    except subprocess.CalledProcessError as e:
        print(f"❌ Error generating TF tree: {e}")
        
    # Also show current TF list
    print("\n🔗 Current TF Frames:")
    print("-" * 30)
    try:
        result = subprocess.run(['ros2', 'run', 'tf2_ros', 'tf2_echo', '--all'], 
                              capture_output=True, text=True, timeout=5)
        if result.stdout:
            print(result.stdout)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        print("⚠️  Could not retrieve current frames (this is normal if robot isn't running)")

def check_robot_status():
    """Check if robot simulation is running"""
    
    print("\n🔍 Checking Robot Simulation Status:")
    print("-" * 40)
    
    # Check for robot_description parameter
    try:
        result = subprocess.run(['ros2', 'param', 'list'], 
                              capture_output=True, text=True, timeout=5)
        if 'robot_description' in result.stdout:
            print("✅ Robot description parameter found")
        else:
            print("❌ Robot description parameter not found")
    except:
        print("⚠️  Could not check parameters")
    
    # Check for important topics
    topics_to_check = ['/tf', '/tf_static', '/joint_states', '/scan', '/camera/image_raw', '/imu/data']
    
    try:
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, text=True, timeout=5)
        available_topics = result.stdout.split('\n')
        
        print("\n📡 Topic Status:")
        for topic in topics_to_check:
            if topic in available_topics:
                print(f"  ✅ {topic}")
            else:
                print(f"  ❌ {topic} (not published)")
                
    except:
        print("⚠️  Could not check topics")

def main():
    """Main function to run diagnostics and generate TF tree"""
    
    print("🚀 Mobile Robot TF Tree Generator")
    print("=" * 50)
    
    # Check if ROS is sourced
    if 'ROS_DISTRO' not in os.environ:
        print("❌ ROS environment not sourced!")
        print("Please run: source /opt/ros/<distro>/setup.bash")
        print("And: source ~/your_workspace/install/setup.bash")
        return
    
    print(f"🟢 ROS {os.environ.get('ROS_DISTRO', 'Unknown')} environment detected")
    
    # Check robot status
    check_robot_status()
    
    # Generate TF tree
    generate_tf_tree()
    
    print("\n" + "=" * 50)
    print("🎯 Instructions for Screenshots:")
    print("1. Open RViz2 and enable all sensor displays")
    print("2. Take screenshots showing:")
    print("   - Robot model with TF frames visible")
    print("   - LiDAR scan data (red points)")
    print("   - Camera feed in separate window")
    print("   - IMU orientation arrows")
    print("   - Odometry trail")
    print("3. The TF tree PDF shows all coordinate relationships")
    
if __name__ == "__main__":
    main()