#!/usr/bin/env python3
"""Plot a patrol trajectory over the static map and check obstacle clearance.

Usage (inside the container):
  ros2 run tb3_nav2_demo plot_trajectory.py <trajectory.csv> <output.png>

The unmapped obstacles are read from the demo world (after xacro expansion)
and from the `spawn` entries of waypoints.yaml, so the plot always matches
what Gazebo actually simulated. For every obstacle the minimum distance from
the robot centre to the obstacle surface is printed; the TurtleBot3 waffle's
footprint radius is ~0.22 m, so anything above that means no contact.
"""
import csv
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import matplotlib
matplotlib.use('Agg')
import matplotlib.patches as mpatches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import transforms  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
import yaml  # noqa: E402

ROBOT_RADIUS = 0.22
SKIP_MODELS = {'ground_plane', 'turtlebot3_world'}


def load_obstacles(pkg):
    sdf = subprocess.run(
        ['xacro', 'headless:=true', os.path.join(pkg, 'worlds', 'tb3_obstacles.sdf.xacro')],
        capture_output=True, text=True, check=True).stdout
    obstacles = []
    for model in ET.fromstring(sdf).iter('model'):
        name = model.get('name')
        if name in SKIP_MODELS or model.find('pose') is None:
            continue
        x, y, _, _, _, yaw = map(float, model.find('pose').text.split())
        geom = model.find('link/collision/geometry')
        if geom.find('box') is not None:
            sx, sy, _ = map(float, geom.find('box/size').text.split())
            obstacles.append(dict(name=name, kind='box', x=x, y=y, yaw=yaw, sx=sx, sy=sy))
        elif geom.find('cylinder') is not None:
            r = float(geom.find('cylinder/radius').text)
            obstacles.append(dict(name=name, kind='cyl', x=x, y=y, r=r))
    return obstacles


def surface_distance(ob, px, py):
    dx, dy = px - ob['x'], py - ob['y']
    if ob['kind'] == 'cyl':
        return np.hypot(dx, dy) - ob['r']
    c, s = math.cos(-ob['yaw']), math.sin(-ob['yaw'])
    lx, ly = c * dx - s * dy, s * dx + c * dy
    qx = np.maximum(np.abs(lx) - ob['sx'] / 2, 0)
    qy = np.maximum(np.abs(ly) - ob['sy'] / 2, 0)
    return np.hypot(qx, qy)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    csv_path, out_path = sys.argv[1:]
    pkg = get_package_share_directory('tb3_nav2_demo')
    bringup = get_package_share_directory('nav2_bringup')

    with open(os.path.join(bringup, 'maps', 'tb3_sandbox.yaml')) as f:
        mcfg = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(bringup, 'maps', mcfg['image'])))
    res, (ox, oy, _) = mcfg['resolution'], mcfg['origin']
    extent = [ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res]

    with open(os.path.join(pkg, 'config', 'waypoints.yaml')) as f:
        wcfg = yaml.safe_load(f)

    obstacles = load_obstacles(pkg)
    spawns = []
    for wp in wcfg['waypoints']:
        if 'spawn' in wp:
            sp = wp['spawn']
            ob = dict(name=sp['name'] + ' (spawned)', kind='box', x=sp['x'],
                      y=sp['y'], yaw=0.0, sx=sp['size'], sy=sp['size'])
            obstacles.append(ob)
            spawns.append((wp['name'], sp, ob))

    rows = list(csv.DictReader(open(csv_path)))
    ts = np.array([float(r['t']) for r in rows])
    xs = np.array([float(r['x']) for r in rows])
    ys = np.array([float(r['y']) for r in rows])
    legs = [r['leg'] for r in rows]

    fig, ax = plt.subplots(figsize=(8, 8), dpi=110)
    ax.imshow(img, cmap='gray', extent=extent, origin='upper', vmin=0, vmax=255)

    for ob in obstacles:
        if ob['kind'] == 'cyl':
            patch = mpatches.Circle((ob['x'], ob['y']), ob['r'])
        else:
            patch = mpatches.Rectangle((ob['x'] - ob['sx'] / 2, ob['y'] - ob['sy'] / 2),
                                       ob['sx'], ob['sy'])
            patch.set_transform(transforms.Affine2D().rotate_around(
                ob['x'], ob['y'], ob['yaw']) + ax.transData)
        patch.set(facecolor='#d6336c', edgecolor='black', alpha=0.85, zorder=3)
        ax.add_patch(patch)
        ax.annotate(ob['name'], (ob['x'], ob['y']), xytext=(6, 6), textcoords='offset points',
                    fontsize=8, color='#a61e4d', zorder=5)

    leg_names = list(dict.fromkeys(legs))
    cmap = plt.get_cmap('tab10')
    for i, leg in enumerate(leg_names):
        m = np.array([lg == leg for lg in legs])
        ax.plot(xs[m], ys[m], '-', lw=2.2, color=cmap(i), label=leg, zorder=4)

    ip = wcfg['initial_pose']
    ax.plot(ip['x'], ip['y'], 'o', ms=10, mfc='white', mec='black', zorder=6)
    ax.annotate('start', (ip['x'], ip['y']), xytext=(-12, -16), textcoords='offset points',
                fontsize=9)
    for wp in wcfg['waypoints']:
        ax.plot(wp['x'], wp['y'], '*', ms=14, mfc='gold', mec='black', zorder=6)

    ax.set_xlim(-3.2, 3.2)
    ax.set_ylim(-3.0, 3.0)
    ax.set_xlabel('x [m] (map frame)')
    ax.set_ylabel('y [m]')
    ax.set_title('Nav2 patrol: driven path vs. obstacles not in the static map')
    ax.legend(loc='lower left', fontsize=8, title='leg')
    fig.tight_layout()
    fig.savefig(out_path)

    print(f'{"obstacle":<26}{"min centre-to-surface [m]":>27}{"contact?":>10}')
    worst = math.inf
    for ob in obstacles:
        d = float(surface_distance(ob, xs, ys).min())
        worst = min(worst, d)
        print(f'{ob["name"]:<26}{d:>27.3f}{"YES" if d < ROBOT_RADIUS else "no":>10}')
    print(f'robot footprint radius: {ROBOT_RADIUS} m -> min clearance {worst - ROBOT_RADIUS:+.3f} m')

    # Reaction to each spawned obstacle: how long the robot sat (nearly) still
    # between the spawn trigger and its closest approach, i.e. the time Nav2
    # needed before it was moving along a new path.
    for leg, sp, ob in spawns:
        m = np.array([lg == leg for lg in legs])
        t, x, y = ts[m], xs[m], ys[m]
        near = np.hypot(x - sp['x'], y - sp['y']) < float(sp.get('trigger_distance', 1.0))
        if not near.any():
            print(f'{ob["name"]}: never triggered')
            continue
        i0 = int(np.argmax(near))
        i1 = i0 + int(np.argmin(surface_distance(ob, x[i0:], y[i0:])))
        stalled = 0.0
        j = i0
        while j < i1:
            k = j
            while k < i1 and t[k] - t[j] < 0.5:
                k += 1
            if k > j and np.hypot(x[k] - x[j], y[k] - y[j]) / max(t[k] - t[j], 1e-3) < 0.05:
                stalled += t[k] - t[j]
            j = k if k > j else j + 1
        print(f'{ob["name"]}: trigger at t={t[i0]:.1f}s, closest approach at t={t[i1]:.1f}s, '
              f'stalled {stalled:.1f} s, leg time {t[-1]:.1f} s')
    print(f'saved {out_path}')
    return 0 if worst >= ROBOT_RADIUS else 1


if __name__ == '__main__':
    sys.exit(main())
