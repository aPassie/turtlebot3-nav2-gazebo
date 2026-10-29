#!/usr/bin/env python3
"""Build docs/assets/data.js for the project page (docs/index.html).

Everything on the page is derived from files in this repo, so the numbers on
the page are the same numbers as in results/ and media/:
  * occupancy grid  <- nav2_bringup/maps/tb3_sandbox.pgm (read from the Docker image)
  * obstacles       <- tb3_nav2_demo/worlds/tb3_obstacles.sdf.xacro + waypoints.yaml
  * trajectories    <- media/trajectory.csv (demo run), results/run2_stock/trajectory.csv
  * run metrics     <- results/*/launch.log (patrol summary) + trajectory.csv

Usage: python3 tools/build_page_data.py [path/to/tb3_sandbox.pgm]
"""
import csv
import io
import json
import math
import re
import subprocess
import sys
from pathlib import Path

import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
PKG = ROOT / 'tb3_nav2_demo'
ROBOT_RADIUS = 0.22          # Nav2's configured robot_radius
CORNER_RADIUS = 0.24         # conservative: Waffle base corner is ~0.238 m from base_link
VIEW = dict(xmin=-3.2, xmax=3.2, ymin=-3.0, ymax=3.0)   # metres, map frame
MAP_RES, MAP_ORIGIN = 0.05, (-10.0, -10.0)
VIDEO_PATROL_START = 7.5   # s into media/demo.mp4 where the first goal is sent

# Runs 0 and 1 predate trajectory logging in their final form (see README);
# their numbers come from the README table and they have no stall metric.
EARLY_RUNS = [
    dict(id='0', label='Run 0', profile='stock', note='first obstacle layout, timer spawn',
         drive=78.3, recoveries=1, clearance=0.102, stall=None, goals='5/5'),
    dict(id='1', label='Run 1', profile='stock', note='obstacles moved onto paths, timer spawn',
         drive=51.1, recoveries=0, clearance=0.215, stall=None, goals='5/5'),
]
CSV_RUNS = [  # (id, label, profile, note, directory or file)
    ('2', 'Run 2', 'stock', 'proximity-triggered surprise box', 'results/run2_stock'),
    ('4', 'Run 4', 'stock', 'repeat of run 2', 'results/run4_stock'),
    ('3', 'Run 3', 'tuned_v1', 'real TB3 speed limits + 10 Hz local costmap', 'results/run3_tuned'),
    ('5', 'Run 5', 'tuned_v1', 'repeat of run 3', 'results/run5_tuned'),
    ('6a', 'Run 6a', 'tuned', '+ 5 Hz global costmap', 'results/run6_gc5hz_1'),
    ('6b', 'Run 6b', 'tuned', 'repeat of run 6a', 'results/run6_gc5hz_2'),
    ('demo', 'Demo', 'tuned', 'the recorded video run (GUI + RViz)', 'media'),
    ('8', 'Run 8', 'tuned', 'no GPU: software rendering (llvmpipe)', 'results/run8_no_gpu_software_render'),
    ('7a', 'Run 7a', 'speed_only', 'ONLY the speed limits changed (isolation test)', 'results/run7_speed_only_1'),
    ('7b', 'Run 7b', 'speed_only', 'repeat of run 7a', 'results/run7_speed_only_2'),
]
DEMO_SUMMARY = dict(drive=67.2, recoveries=0, goals='5/5')  # from the record_demo.sh output


def load_map(pgm_path):
    if pgm_path:
        data = Path(pgm_path).read_bytes()
    else:
        data = subprocess.run(
            ['docker', 'run', '--rm', 'tb3-nav2:latest', 'cat',
             '/opt/ros/jazzy/share/nav2_bringup/maps/tb3_sandbox.pgm'],
            capture_output=True, check=True).stdout
    img = Image.open(io.BytesIO(data))
    w, h = img.size
    px = img.load()
    c0 = round((VIEW['xmin'] - MAP_ORIGIN[0]) / MAP_RES)
    c1 = round((VIEW['xmax'] - MAP_ORIGIN[0]) / MAP_RES)
    r0 = h - round((VIEW['ymax'] - MAP_ORIGIN[1]) / MAP_RES)
    r1 = h - round((VIEW['ymin'] - MAP_ORIGIN[1]) / MAP_RES)
    rows = []
    for r in range(r0, r1):            # top row = ymax
        row = []
        for c in range(c0, c1):
            v = px[c, r]
            row.append('1' if v < 100 else ('0' if v > 230 else '2'))   # occ / free / unknown
        rows.append(''.join(row))
    return dict(res=MAP_RES, cols=c1 - c0, rows=len(rows), xmin=VIEW['xmin'],
                ymax=VIEW['ymax'], cells=rows)


def load_obstacles():
    text = (PKG / 'worlds' / 'tb3_obstacles.sdf.xacro').read_text()
    obs = []
    for kind, attrs in re.findall(r'<xacro:(box_obstacle|cylinder_obstacle)\s([^>]*)/>', text):
        a = dict(re.findall(r'(\w+)="([^"]*)"', attrs))
        o = dict(name=a['name'], x=float(a['x']), y=float(a['y']), spawned=False,
                 color=[float(a['r']), float(a['g']), float(a['b'])])
        if kind == 'box_obstacle':
            o.update(kind='box', yaw=float(a['yaw']), sx=float(a['sx']), sy=float(a['sy']))
        else:
            o.update(kind='cyl', r=float(a['radius']))
        obs.append(o)
    wcfg = yaml.safe_load((PKG / 'config' / 'waypoints.yaml').read_text())
    for wp in wcfg['waypoints']:
        if 'spawn' in wp:
            sp = wp['spawn']
            obs.append(dict(name=sp['name'], kind='box', x=sp['x'], y=sp['y'], yaw=0.0,
                            sx=sp['size'], sy=sp['size'], spawned=True, leg=wp['name'],
                            trigger=sp.get('trigger_distance', 1.0), color=[0.8, 0.1, 0.6]))
    return obs, wcfg


def surface_distance(o, x, y):
    dx, dy = x - o['x'], y - o['y']
    if o['kind'] == 'cyl':
        return math.hypot(dx, dy) - o['r']
    c, s = math.cos(-o['yaw']), math.sin(-o['yaw'])
    lx, ly = c * dx - s * dy, s * dx + c * dy
    return math.hypot(max(abs(lx) - o['sx'] / 2, 0), max(abs(ly) - o['sy'] / 2, 0))


def read_csv(path):
    return [(float(r['t']), r['leg'], float(r['x']), float(r['y']))
            for r in csv.DictReader(open(path))]


def stall_time(rows, o):
    """Same definition as plot_trajectory.py: time below 0.05 m/s between
    the spawn trigger and the closest approach, in ~0.5 s windows."""
    pts = [r for r in rows if r[1] == o['leg']]
    trig = next((i for i, p in enumerate(pts)
                 if math.hypot(p[2] - o['x'], p[3] - o['y']) < o['trigger']), None)
    if trig is None:
        return None
    i1 = min(range(trig, len(pts)), key=lambda i: surface_distance(o, pts[i][2], pts[i][3]))
    stalled, j = 0.0, trig
    while j < i1:
        k = j
        while k < i1 and pts[k][0] - pts[j][0] < 0.5:
            k += 1
        if k > j:
            dt = max(pts[k][0] - pts[j][0], 1e-3)
            if math.hypot(pts[k][2] - pts[j][2], pts[k][3] - pts[j][3]) / dt < 0.05:
                stalled += pts[k][0] - pts[j][0]
            j = k
        else:
            j += 1
    return round(stalled, 1)


def summary_from_log(log):
    m = re.search(r'(\d+)/(\d+) goals reached, ([\d.]+) s driving, (\d+) recoveries', log)
    if m:
        return dict(goals=f'{m[1]}/{m[2]}', drive=float(m[3]), recoveries=int(m[4]))
    return None


def build_runs(obstacles):
    runs = list(EARLY_RUNS)
    for rid, label, profile, note, d in CSV_RUNS:
        d = ROOT / d
        rows = read_csv(d / 'trajectory.csv')
        log = (d / 'launch.log').read_text() if (d / 'launch.log').exists() else ''
        summ = summary_from_log(log) or dict(DEMO_SUMMARY)
        closest = min(min(surface_distance(o, x, y) for _, _, x, y in rows) for o in obstacles)
        clearance = closest - ROBOT_RADIUS
        south = [r for r in rows if r[1] == 'south']
        cm_events = log.count('Robot to approach') if log else None
        surprise = next(o for o in obstacles if o['spawned'])
        runs.append(dict(id=rid, label=label, profile=profile, note=note,
                         clearance=round(clearance, 3),
                         clearance_corner=round(closest - CORNER_RADIUS, 3),
                         stall=stall_time(rows, surprise), south_leg=round(south[-1][0], 1),
                         collision_monitor=cm_events, **summ))
    return runs


def trajectory(path, legs_order):
    rows = read_csv(path)
    out, offset, last_t, cur_leg, leg_start = [], 0.0, -1.0, None, {}
    for t, leg, x, y in rows:
        if leg != cur_leg:
            if cur_leg is not None:
                offset = out[-1][0] if out else 0.0
            cur_leg = leg
            leg_start[leg] = offset
            last_t = -1.0
        g = offset + t
        if out and g - out[-1][0] < 0.1 and t - last_t < 0.1:
            continue
        last_t = t
        out.append([round(g, 2), legs_order.index(leg), round(x, 3), round(y, 3)])
    return dict(points=out, legStart=leg_start)


def main():
    grid = load_map(sys.argv[1] if len(sys.argv) > 1 else None)
    obstacles, wcfg = load_obstacles()
    legs = [wp['name'] for wp in wcfg['waypoints']]
    data = dict(
        map=grid,
        obstacles=obstacles,
        start=wcfg['initial_pose'],
        waypoints=[dict(name=w['name'], x=w['x'], y=w['y']) for w in wcfg['waypoints']],
        legs=legs,
        robotRadius=ROBOT_RADIUS,
        videoPatrolStart=VIDEO_PATROL_START,
        trajectories=dict(
            tuned=trajectory(ROOT / 'media' / 'trajectory.csv', legs),
            stock=trajectory(ROOT / 'results' / 'run2_stock' / 'trajectory.csv', legs),
        ),
        runs=build_runs(obstacles),
    )
    out = ROOT / 'docs' / 'assets' / 'data.js'
    out.write_text('// Generated by tools/build_page_data.py - do not edit by hand.\n'
                   'window.PROJECT_DATA = ' + json.dumps(data, separators=(',', ':')) + ';\n')
    print(f'wrote {out} ({out.stat().st_size / 1024:.0f} KB)')
    for r in data['runs']:
        print(f"  {r['label']:<7} {r['profile']:<10} drive={r['drive']:>5} rec={r['recoveries']} "
              f"clear={r['clearance']:+.3f} corner={r.get('clearance_corner')} stall={r['stall']} "
              f"south={r.get('south_leg')} cm={r.get('collision_monitor')}")


if __name__ == '__main__':
    main()
