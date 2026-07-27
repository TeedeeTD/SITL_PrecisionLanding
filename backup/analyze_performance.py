#!/usr/bin/env python3
import sys
import os
import re
import time
import subprocess
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from dib_msgs.msg import LandingTarget6D

class PerformanceAnalyzer(Node):
    def __init__(self):
        super().__init__('performance_analyzer')
        
        # Lists to store topic metrics
        self.image_latencies = []
        self.image_times = []
        self.detection_times = []
        self.detection_distances = []
        self.detection_states = []
        
        # Subscriptions
        self.img_sub = self.create_subscription(
            Image,
            '/siyi/image_raw',
            self.image_callback,
            10
        )
        self.target_sub = self.create_subscription(
            LandingTarget6D,
            '/siyi/landing_target',
            self.target_callback,
            10
        )
        
        # Tegrastats metrics
        self.cpu_usages = []
        self.gpu_usages = [] # GR3D
        self.vic_usages = [] # VIC
        self.nvdec_freqs = [] # NVDEC
        self.tegrastats_running = True
        
        self.get_logger().info("Performance Analyzer started. Ready to measure.")

    def image_callback(self, msg):
        now = self.get_clock().now()
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        latency_ms = (now - stamp).nanoseconds / 1e6
        self.image_latencies.append(latency_ms)
        self.image_times.append(time.time())

    def target_callback(self, msg):
        self.detection_times.append(time.time())
        if msg.tag_id >= 0:
            dist = np.sqrt(msg.x**2 + msg.y**2 + msg.z**2)
            self.detection_distances.append(dist)
            self.detection_states.append(True)
        else:
            self.detection_states.append(False)

    def run_tegrastats(self):
        # Run tegrastats with sudo to get VIC and NVDEC metrics
        try:
            proc = subprocess.Popen(['sudo', 'tegrastats', '--interval', '1000'], 
                                    stdout=subprocess.PIPE, 
                                    stderr=subprocess.DEVNULL,
                                    text=True)
        except FileNotFoundError:
            self.get_logger().error("tegrastats/sudo utility not found!")
            return

        cpu_pattern = re.compile(r'CPU \[([^\]]+)\]')
        gpu_pattern = re.compile(r'GR3D_FREQ (\d+)%|GR3D (\d+)%')
        vic_pattern = re.compile(r'VIC_FREQ (\d+)%|VIC (\d+)%')
        nvdec_pattern = re.compile(r'NVDEC_FREQ (\d+)%|NVDEC (\d+)')

        while self.tegrastats_running and proc.poll() is None:
            line = proc.stdout.readline()
            if not line:
                break
            
            # Parse CPU
            cpu_match = cpu_pattern.search(line)
            if cpu_match:
                cores_str = cpu_match.group(1)
                cores = re.findall(r'(\d+)%', cores_str)
                if cores:
                    avg_cpu = sum(int(c) for c in cores) / len(cores)
                    self.cpu_usages.append(avg_cpu)
            
            # Parse GPU (GR3D)
            gpu_match = gpu_pattern.search(line)
            if gpu_match:
                val = gpu_match.group(1) or gpu_match.group(2)
                if val is not None:
                    self.gpu_usages.append(float(val))
                    
            # Parse VIC
            vic_match = vic_pattern.search(line)
            if vic_match:
                val = vic_match.group(1) or vic_match.group(2)
                if val is not None:
                    self.vic_usages.append(float(val))
                    
            # Parse NVDEC
            nvdec_match = nvdec_pattern.search(line)
            if nvdec_match:
                val = nvdec_match.group(1) or nvdec_match.group(2)
                if val is not None:
                    self.nvdec_freqs.append(float(val))
                    
        if proc.poll() is None:
            proc.terminate()
            # Clean up sudo tegrastats background process
            subprocess.run(['sudo', 'killall', 'tegrastats'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def main():
    print("[*] Running script as regular user to receive ROS 2 topics.")
    print("[*] Script will use 'sudo' internally to capture GPU/VIC/NVDEC stats.\n")

    rclpy.init(args=None)
    node = PerformanceAnalyzer()
    
    # Run tegrastats in a background thread
    tegrastats_thread = threading.Thread(target=node.run_tegrastats, daemon=True)
    tegrastats_thread.start()
    
    duration = 60
    print(f"[*] Starting 60 seconds measurement window...")
    
    start_time = time.time()
    
    # Spin node
    try:
        while rclpy.ok() and (time.time() - start_time) < duration:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        print("[!] Interrupted by user.")
    
    node.tegrastats_running = False
    
    total_time = time.time() - start_time
    
    # Calculate stats
    avg_cpu = np.mean(node.cpu_usages) if node.cpu_usages else 0.0
    avg_gpu = np.mean(node.gpu_usages) if node.gpu_usages else 0.0
    avg_vic = np.mean(node.vic_usages) if node.vic_usages else 0.0
    
    avg_nvdec = np.mean(node.nvdec_freqs) if node.nvdec_freqs else 0.0
    nvdec_str = "N/A"
    if avg_nvdec > 0:
        if avg_nvdec < 100:
            nvdec_str = f"{avg_nvdec:.1f}%"
        else:
            mhz = avg_nvdec / 1e6 if avg_nvdec > 1000 else avg_nvdec
            pct = (mhz / 524.8) * 100.0 if mhz > 100 else mhz
            nvdec_str = f"{pct:.1f}% ({mhz:.1f} MHz)"

    # FPS
    num_images = len(node.image_times)
    fps = num_images / total_time if total_time > 0 else 0.0
    
    # Det. Rate
    detections_found = sum(1 for d in node.detection_states if d)
    det_rate = (detections_found / num_images * 100.0) if num_images > 0 else 0.0
    
    # E2E Latency
    avg_e2e = np.mean(node.image_latencies) if node.image_latencies else 0.0
    
    # Distance
    avg_dist = np.mean(node.detection_distances) if node.detection_distances else 0.0
    
    # Accuracy
    accuracy_str = "N/A"
    if len(node.detection_distances) > 1:
        accuracy_str = f"±{np.std(node.detection_distances):.3f}m"
    
    print("\n" + "="*50)
    print("MEASUREMENT RESULTS (60 Seconds)")
    print("="*50)
    print(f"Total Frames Received : {num_images}")
    print(f"Average CPU Usage    : {avg_cpu:.1f}%")
    print(f"Average GPU Usage    : {avg_gpu:.1f}% (GR3D)")
    print(f"Average VIC Usage    : {avg_vic:.1f}% (Video Image Coprocessor)")
    print(f"Average NVDEC Usage  : {nvdec_str}")
    print(f"Average FPS          : {fps:.2f}")
    print(f"Detection Rate       : {det_rate:.1f}% ({detections_found}/{num_images})")
    print(f"Average Distance     : {avg_dist:.2f}m" if avg_dist > 0 else "Average Distance     : N/A")
    print(f"Accuracy (Std Dev)   : {accuracy_str}")
    print(f"Average E2E Latency  : {avg_e2e:.1f}ms")
    print("="*50)
    
    # Print the requested Markdown Table Row
    print("\n### Copy the table below:")
    print("| Config | Resolution | Tag size | Distance | %CPU | %GPU (GR3D) | %VIC (Image) | %NVDEC (Video) | FPS | Det. Rate | Accuracy | E2E latency | Ghi chú |")
    print("| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |")
    
    dist_val = f"{avg_dist:.2f}m" if avg_dist > 0 else "N/A"
    vic_str = f"{avg_vic:.1f}%" if avg_vic > 0 else "0.0%"
    
    print(f"| A1 | 1280x720 | 20cm | {dist_val} | {avg_cpu:.1f}% | {avg_gpu:.1f}% | {vic_str} | {nvdec_str} | {fps:.2f} | {det_rate:.1f}% | {accuracy_str} | {avg_e2e:.1f}ms | Giải mã và xoay ảnh bằng phần cứng (NVDEC/VIC) |")
    print("\n")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
