#!/usr/bin/env python3
import sys, math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches

def make_street_obstacles():
    return [
        (5.0, -2.0, 7.0, 5.0),
        (-4.0, 5.0, 16.0, 2.0),
        (-4.0, -2.0, 7.0, 4.0),
        (8.0, 3.0, 4.0, 2.0),
    ]

# Draw the environment with obstacles
def draw_environment(ax):
    for (x, y, w, h) in make_street_obstacles():
        ax.add_patch(patches.Rectangle((x, y), w, h, facecolor='lightgray', edgecolor='black'))
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(-10, 20)
    ax.set_ylim(-8, 12)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Car Path Visualization')

# Get the corners of the car given its center (x, y) and orientation theta
def car_corners(x, y, theta, side=1.0):
    half = side / 2.0
    local = np.array([[half, half], [-half, half], [-half, -half], [half, -half]])
    R = np.array([[math.cos(theta), -math.sin(theta)],
                  [math.sin(theta),  math.cos(theta)]])
    return (R @ local.T).T + np.array([x, y])

# Load path data from text file
def load_path_txt(fname):
    data = np.loadtxt(fname)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 3:
        raise ValueError("Expected at least x, y, theta columns.")
    return data[:,1], data[:,2], data[:,3]

def main():
    # Parse command line arguments
    if len(sys.argv) != 3:
        print("Usage: python3 visualize_car.py output_name path.txt")
        sys.exit(1)
    output_png, pathfile = sys.argv[1], sys.argv[2]
    x, y, theta = load_path_txt(pathfile)

    # Create the plot
    fig, ax = plt.subplots(figsize=(8, 6))
    draw_environment(ax)
    ax.plot(x, y, 'b-', lw=2, label='Path')

    # draw start & goal
    ax.plot(x[0], y[0], 'go', ms=8, label='Start')
    ax.plot(x[-1], y[-1], 'ro', ms=8, label='Goal')

    # draw car shape at start and end
    for i, idx in enumerate([0, len(x)//2, -1]):
        corners = car_corners(x[idx], y[idx], theta[idx])
        ax.add_patch(patches.Polygon(corners, closed=True,
                                     facecolor='none', edgecolor='navy', lw=2))
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_png, dpi=200)
    plt.close(fig)
    print(f"[OK] Saved static car path image to {output_png}")

if __name__ == "__main__":
    main()
