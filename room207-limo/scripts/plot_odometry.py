#!/usr/bin/env python3

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import sqlite3

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message


def quaternion_to_yaw(q):
    """Convert a quaternion orientation to a yaw angle."""

    return np.arctan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y**2 + q.z**2),
    )


def main():
    """Read odometry data from a rosbag and generate results plots."""

    # ============================================================
    # Command-line arguments
    # ============================================================

    parser = argparse.ArgumentParser(
        description="Plot LIMO odometry data from a rosbag2."
    )

    parser.add_argument(
        "bag",
        help="Path to the rosbag2 directory"
    )

    args = parser.parse_args()

    # ============================================================
    # Paths
    # ============================================================

    bag_dir = Path(args.bag).expanduser()

    workspace_dir = Path(
        get_package_prefix("room207_limo")
    ).parent.parent

    results_dir = workspace_dir / "data"
    results_dir.mkdir(
        parents=True,
        exist_ok=True
    )

    # Circuit map provided by the qcar_large_track package.
    map_file = (
        Path(get_package_share_directory("qcar_large_track"))
        / "models"
        / "qcar_large_track"
        / "materials"
        / "textures"
        / "SDCS_MapLayout.png"
    )

    # ============================================================
    # Check files
    # ============================================================

    if not bag_dir.exists():

        raise FileNotFoundError(
            f"Rosbag not found: {bag_dir}"
        )

    if not map_file.exists():

        raise FileNotFoundError(
            f"Map file not found: {map_file}"
        )

    # ============================================================
    # Open rosbag2 SQLite database
    # ============================================================

    db_files = sorted(bag_dir.glob("*.db3"))

    if not db_files:
        raise FileNotFoundError(
            f"No SQLite3 database found in rosbag: {bag_dir}"
        )

    connection = sqlite3.connect(str(db_files[0]))

    try:
        # ========================================================
        # Check /odom
        # ========================================================

        topic = connection.execute(
            """
            SELECT id, type
            FROM topics
            WHERE name = ?
            """,
            ("/odom",),
        ).fetchone()

        if topic is None:
            raise RuntimeError(
                "Topic /odom not found in rosbag."
            )

        topic_id, topic_type = topic

        if topic_type != "nav_msgs/msg/Odometry":
            raise RuntimeError(
                "Topic /odom does not contain "
                "nav_msgs/msg/Odometry messages."
            )

        # ========================================================
        # Load odometry data
        # ========================================================

        rows = connection.execute(
            """
            SELECT timestamp, data
            FROM messages
            WHERE topic_id = ?
            ORDER BY timestamp
            """,
            (topic_id,),
        )

        time = []
        x = []
        y = []
        yaw = []
        vx = []
        vy = []
        wz = []

        for timestamp, data in rows:
            msg = deserialize_message(
                data,
                Odometry,
            )

            time.append(timestamp * 1e-9)

            x.append(msg.pose.pose.position.x)
            y.append(msg.pose.pose.position.y)

            yaw.append(
                quaternion_to_yaw(
                    msg.pose.pose.orientation
                )
            )

            vx.append(msg.twist.twist.linear.x)
            vy.append(msg.twist.twist.linear.y)
            wz.append(msg.twist.twist.angular.z)

    finally:
        connection.close()


    # ============================================================
    # Convert to numpy arrays
    # ============================================================

    time = np.asarray(time)
    x = np.asarray(x)
    y = np.asarray(y)
    yaw = np.asarray(yaw)
    vx = np.asarray(vx)
    vy = np.asarray(vy)
    wz = np.asarray(wz)

    if len(time) == 0:

        raise RuntimeError(
            "No /odom messages found in rosbag."
        )

    # Convert timestamps to elapsed time.
    time = time - time[0]

    # ============================================================
    # Load circuit map
    # ============================================================

    img = plt.imread(str(map_file))

    # The image coordinate system is vertically inverted
    # compared with the simulation coordinate system.
    img = np.flipud(img)

    # ============================================================
    # Simulation name
    # ============================================================

    simulation_name = bag_dir.name

    # ============================================================
    # Plot trajectory
    # ============================================================

    plt.figure()

    plt.imshow(
        img,
        extent=[-2.44, 2.44, -3.10, 3.10],
        origin="lower",
        alpha=0.5,
    )

    plt.plot(
        x,
        y,
        label="Odometry",
    )

    plt.xlabel(r"$x$ [m]")
    plt.ylabel(r"$y$ [m]")
    plt.title("LIMO trajectory")

    ax = plt.gca()

    ax.set_aspect(
        "equal",
        adjustable="box",
    )

    ax.set_xlim(-2.44, 2.44)
    ax.set_ylim(-3.10, 3.10)

    plt.legend()

    plt.savefig(
        results_dir / f"trajectory_{simulation_name}.pdf",
        bbox_inches="tight",
    )

    # ============================================================
    # Plot velocities
    # ============================================================

    plt.figure()

    plt.plot(
        time,
        vx,
        label=r"$v_x$",
    )

    plt.plot(
        time,
        vy,
        label=r"$v_y$",
    )

    plt.plot(
        time,
        wz,
        label=r"$\omega_z$",
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Velocity")
    plt.title("LIMO velocities")

    plt.grid()
    plt.legend()

    plt.savefig(
        results_dir / f"velocities_{simulation_name}.pdf",
        bbox_inches="tight",
    )

    # ============================================================
    # Plot yaw
    # ============================================================

    plt.figure()

    plt.plot(
        time,
        yaw,
    )

    plt.xlabel("Time [s]")
    plt.ylabel(r"Yaw [rad]")
    plt.title("LIMO orientation")

    plt.grid()

    plt.savefig(
        results_dir / f"orientation_{simulation_name}.pdf",
        bbox_inches="tight",
    )

    # ============================================================
    # Display figures
    # ============================================================

    plt.show()


if __name__ == "__main__":
    main()