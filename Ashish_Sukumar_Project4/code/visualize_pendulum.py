#!/usr/bin/env python3
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt

# this function is used put any angle into the range (-pi, pi]
def wrap_angle(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi

# find big jumps in theta so lines don’t cross the ±pi boundary    jumps = np.where(np.abs(np.diff(theta_wrapped)) > jump_thresh)[0] + 1
    idx = np.arange(len(theta_wrapped))
    return np.split(idx, jumps)

# set plot limits a bit wider so data isn’t at the edge
def smart_limits(x, y, xpad_frac=0.04, ypad_frac=0.10, min_ypad=0.35):
    xmin, xmax = float(np.min(x)), float(np.max(x))
    ymin, ymax = float(np.min(y)), float(np.max(y))
    xr = max(1e-9, xmax - xmin)
    yr = max(1e-9, ymax - ymin)
    xpad = xpad_frac * xr
    ypad = max(min_ypad, ypad_frac * yr)
    return (xmin - xpad, xmax + xpad), (ymin - ypad, ymax + ypad)

def main():
    # Parse arguments from command line
    ap = argparse.ArgumentParser()
    ap.add_argument("input_txt", help="path to pendulum.txt (t theta omega [tau])")
    ap.add_argument("output_png", help="output image path")
    ap.add_argument("--unwrapped", action="store_true",
                    help="force unwrapped θ on the x-axis (continuous over multiple revolutions)")
    args = ap.parse_args()

    # Load data from text file
    data = np.loadtxt(args.input_txt)
    if data.ndim == 1:
        data = data[None, :]
    if data.shape[1] < 3:
        raise ValueError("Input must have at least 3 columns: t theta omega (tau optional).")

    t     = data[:, 0]
    theta = data[:, 1]
    omega = data[:, 2]

    # Prepare wrapped and unwrapped theta versions
    theta_wrapped   = wrap_angle(theta)
    theta_unwrapped = np.unwrap(theta)

    use_unwrapped = args.unwrapped
    if not use_unwrapped:
        # check if unwrapped is better for visualization
        span = theta_unwrapped.max() - theta_unwrapped.min()
        many_revs = span > 1.75 * np.pi
        jumps = np.where(np.abs(np.diff(theta_wrapped)) > (np.pi/2))[0]
        use_unwrapped = bool(many_revs and len(jumps) > 2)

    # Select which theta to use for x-axis
    X = theta_unwrapped if use_unwrapped else theta_wrapped

    # Create the plot
    fig = plt.figure(figsize=(9.4, 7.2))
    ax  = plt.gca()

    sc = ax.scatter(X, omega, c=t, s=18, cmap="viridis", edgecolors="none", zorder=3)

    # Draw a thin line to visually connect points
    if use_unwrapped:
        ax.plot(X, omega, lw=1.0, color="k", alpha=0.28, zorder=2)
    else:
        for seg in split_on_wrap(theta_wrapped, omega):
            if len(seg) >= 2:
                ax.plot(X[seg], omega[seg], lw=1.0, color="k", alpha=0.28, zorder=2)

    # Start / End markers
    ax.scatter(X[0],  omega[0],  s=90, c="limegreen", edgecolors="k", linewidths=0.7, zorder=4, label="Start")
    ax.scatter(X[-1], omega[-1], s=90, c="crimson",   edgecolors="k", linewidths=0.7, zorder=4, label="End")

    # Labels and title
    ax.set_xlabel(r"$\theta$ (rad)")
    ax.set_ylabel(r"$\dot{\theta}$ (rad/s)")
    ax.set_title("Pendulum phase portrait (colored by time)")

    if not use_unwrapped:
        xticks = [-np.pi, -np.pi/2, 0, np.pi/2, np.pi]
        xtlbls = [r"$-\pi$", r"$-\frac{\pi}{2}$", r"$0$", r"$\frac{\pi}{2}$", r"$\pi$"]
        ax.set_xticks(xticks)
        ax.set_xticklabels(xtlbls)

    # Grid and legend
    ax.grid(True, alpha=0.28)
    ax.legend(loc="upper right", frameon=True)

    # Make sure the entire path (markers + line) stays inside the frame
    (xmin, xmax), (ymin, ymax) = smart_limits(X, omega)
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)

    # Colorbar
    cbar = plt.colorbar(sc, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("time (s)")

    plt.tight_layout()
    plt.savefig(args.output_png, dpi=220, bbox_inches="tight")
    plt.close()
    print(f"[OK] Saved pendulum plot to {args.output_png} (unwrapped={use_unwrapped})")

if __name__ == "__main__":
    main()
