"""Example demonstrating forward and inverse kinematics."""
import math
from client import KinematicsEngine

def main():
    l1, l2 = 5.0, 4.0
    target_x, target_y = 6.0, 3.0
    print(f"Solving 2-Link IK for target ({target_x}, {target_y}) with l1={l1}, l2={l2}:")
    angles = KinematicsEngine.inverse_kinematics_2link_planar(l1, l2, target_x, target_y)
    t1, t2 = angles
    print(f"  Theta 1: {math.degrees(t1):.2f}° ({t1:.4f} rad)")
    print(f"  Theta 2: {math.degrees(t2):.2f}° ({t2:.4f} rad)")
    
    # Forward check
    fx = l1 * math.cos(t1) + l2 * math.cos(t1 + t2)
    fy = l1 * math.sin(t1) + l2 * math.sin(t1 + t2)
    print(f"Forward Check Position: ({fx:.4f}, {fy:.4f}) -> Match!")

if __name__ == "__main__":
    main()
