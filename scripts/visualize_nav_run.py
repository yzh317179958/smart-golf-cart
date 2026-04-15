#!/usr/bin/env python3
"""导航测试可视化 — 规划路线 vs 实际轨迹

只画两条线：
  蓝线 = 规划路线（Dijkstra 路点序列）
  红线 = 实际 GPS 轨迹

用法:
    python3 ~/golf_ws/scripts/visualize_nav_run.py
    scp ~/golf_ws/data/nav_run_latest.html laptop:~/Desktop/
"""

import json
import os
import sys
import glob
import heapq

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
DATA_DIR = os.path.join(PROJECT_DIR, 'data')


def find_latest_test_log():
    files = sorted(glob.glob(os.path.join(DATA_DIR, 'test_log_*.json')))
    return files[-1] if files else None


def load_json(path):
    with open(path, 'r') as f:
        return json.load(f)


def dijkstra(waypoints, edges, start_id, goal_id):
    """复制 follower 的 Dijkstra 逻辑，还原规划路径"""
    adj = {wid: [] for wid in waypoints}
    for e in edges:
        a, b, d = e['from'], e['to'], e.get('distance', 0)
        if a in adj:
            adj[a].append((b, d))
        if b in adj:
            adj[b].append((a, d))
    heap = [(0.0, start_id)]
    dist = {start_id: 0.0}
    prev = {}
    while heap:
        cost, cur = heapq.heappop(heap)
        if cur == goal_id:
            path = []
            node = goal_id
            while node is not None:
                path.append(node)
                node = prev.get(node)
            return list(reversed(path))
        if cost > dist.get(cur, float('inf')):
            continue
        for nb, d in adj.get(cur, []):
            nc = cost + d
            if nc < dist.get(nb, float('inf')):
                dist[nb] = nc
                prev[nb] = cur
                heapq.heappush(heap, (nc, nb))
    return None


def find_nearest(waypoints, lat, lon):
    import math
    best, bd = None, float('inf')
    for wid, w in waypoints.items():
        d = (w['lat'] - lat)**2 + (w['lon'] - lon)**2
        if d < bd:
            best, bd = wid, d
    return best


def generate_html(plan_coords, traj_coords, target_name):
    all_lats = [c[0] for c in plan_coords + traj_coords]
    all_lons = [c[1] for c in plan_coords + traj_coords]
    clat = sum(all_lats) / len(all_lats)
    clon = sum(all_lons) / len(all_lons)

    plan_js = json.dumps([[c[0], c[1]] for c in plan_coords])
    traj_js = json.dumps([[c[0], c[1]] for c in traj_coords])

    return f"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>Nav Run → {target_name}</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
body{{margin:0}} #map{{height:100vh}}
#legend{{position:absolute;bottom:20px;right:10px;z-index:999;background:rgba(255,255,255,.9);padding:10px 14px;border-radius:8px;font:14px monospace;line-height:1.8}}
</style></head><body>
<div id="map"></div>
<div id="legend">
  <span style="color:#2196F3">━━</span> 规划路线 ({len(plan_coords)} 点)<br>
  <span style="color:#F44336">━━</span> 实际轨迹 ({len(traj_coords)} 点)<br>
  <span style="color:#4CAF50">●</span> 起点 &nbsp;
  <span style="color:#FF9800">●</span> 终点
</div>
<script>
var map = L.map('map').setView([{clat},{clon}], 19);
L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}',
  {{maxZoom:22, maxNativeZoom:19}}).addTo(map);

var plan = {plan_js};
var traj = {traj_js};

// 规划路线（蓝）
if(plan.length>1) L.polyline(plan, {{color:'#2196F3',weight:4,opacity:0.8}}).addTo(map);
// 实际轨迹（红）
if(traj.length>1) L.polyline(traj, {{color:'#F44336',weight:3,opacity:0.8}}).addTo(map);

// 规划路点标号
plan.forEach(function(p,i){{
  L.circleMarker(p,{{radius:3,color:'#2196F3',fillOpacity:1}})
   .bindPopup('wp '+(i+1)+'/'+plan.length).addTo(map);
}});

// 起点（绿）终点（橙）
if(traj.length>0){{
  L.circleMarker(traj[0],{{radius:8,color:'#4CAF50',fillColor:'#4CAF50',fillOpacity:1}}).bindPopup('START').addTo(map);
  L.circleMarker(traj[traj.length-1],{{radius:8,color:'#FF9800',fillColor:'#FF9800',fillOpacity:1}}).bindPopup('END').addTo(map);
}}

// 自动缩放
var all = plan.concat(traj);
if(all.length>1) map.fitBounds(L.polyline(all).getBounds().pad(0.15));
</script></body></html>"""


def main():
    test_log_path = sys.argv[1] if len(sys.argv) > 1 else find_latest_test_log()
    if not test_log_path or not os.path.exists(test_log_path):
        print(f'[ERROR] 找不到 test_log: {test_log_path}')
        sys.exit(1)

    graph_path = os.path.join(DATA_DIR, 'path_graph.json')
    wp_data = load_json(graph_path)
    test_log = load_json(test_log_path)

    wps = wp_data.get('waypoints', {})
    edges = wp_data.get('edges', [])
    traj = test_log.get('gps_trajectory', [])
    events = test_log.get('events', [])

    # 找导航目标
    target_name = 'unknown'
    for e in events:
        if e.get('type') == 'nav_trigger':
            target_name = e.get('data', '')
            break

    # 还原规划路径：起点=轨迹第一个GPS点最近路点，终点=target
    plan_coords = []
    if traj and target_name in wps:
        start_id = find_nearest(wps, traj[0]['lat'], traj[0]['lon'])
        path_ids = dijkstra(wps, edges, start_id, target_name)
        if path_ids:
            plan_coords = [(wps[wid]['lat'], wps[wid]['lon']) for wid in path_ids]
            print(f'[OK] 规划路径: {start_id} → {target_name}, {len(path_ids)} 路点')

    traj_coords = [(p['lat'], p['lon']) for p in traj]
    print(f'[OK] 实际轨迹: {len(traj_coords)} GPS 点')

    html = generate_html(plan_coords, traj_coords, target_name)
    out = os.path.join(DATA_DIR, 'nav_run_latest.html')
    with open(out, 'w') as f:
        f.write(html)
    print(f'[OK] {out} ({len(html)} bytes)')


if __name__ == '__main__':
    main()
