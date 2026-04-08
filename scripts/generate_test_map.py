#!/usr/bin/env python3
"""P2-17 离线测试地图生成器

读取 path_graph.json + test_log_*.json，生成增强版卫星图 HTML。
包含: 路点图 + GPS 轨迹(彩色) + 事件标记 + 统计面板。

用法:
    scp -P 2225 wheeltec@localhost:~/golf_ws/data/test_log_*.json ~/golf_ws/data/
    scp -P 2225 wheeltec@localhost:~/golf_ws/data/path_graph.json ~/golf_ws/data/
    python3 ~/golf_ws/scripts/generate_test_map.py
    scp ~/golf_ws/data/test_latest.html laptop:~/Desktop/
"""

import glob
import json
import math
import os
import sys
from datetime import datetime

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
DATA_DIR = os.path.join(PROJECT_DIR, 'data')


def load_data():
    """加载路点图 + 最新测试日志"""
    # 路点图
    wp_file = os.path.join(DATA_DIR, 'path_graph.json')
    wp_data = {'waypoints': {}, 'edges': []}
    if os.path.exists(wp_file):
        with open(wp_file) as f:
            wp_data = json.load(f)
        print(f'[OK] 路点: {len(wp_data.get("waypoints", {}))} 个')

    # 找最新的 test_log
    logs = sorted(glob.glob(os.path.join(DATA_DIR, 'test_log_*.json')))
    test_data = {'gps_trajectory': [], 'events': []}
    if logs:
        latest = logs[-1]
        with open(latest) as f:
            test_data = json.load(f)
        n_pts = len(test_data.get('gps_trajectory', []))
        n_evt = len(test_data.get('events', []))
        print(f'[OK] 测试日志: {os.path.basename(latest)} ({n_pts} 轨迹点, {n_evt} 事件)')
    else:
        print('[WARN] 未找到测试日志，只生成路点图')

    return wp_data, test_data


def generate_html(wp_data, test_data, gen_time):
    """生成自包含 HTML"""
    wp_json = json.dumps(wp_data, ensure_ascii=False)
    test_json = json.dumps(test_data, ensure_ascii=False)
    traj = test_data.get('gps_trajectory', [])
    events = test_data.get('events', [])

    # 计算中心点
    all_lats, all_lons = [], []
    for w in wp_data.get('waypoints', {}).values():
        all_lats.append(w['lat'])
        all_lons.append(w['lon'])
    for p in traj:
        all_lats.append(p['lat'])
        all_lons.append(p['lon'])
    if all_lats:
        c_lat = sum(all_lats) / len(all_lats)
        c_lon = sum(all_lons) / len(all_lons)
    else:
        c_lat, c_lon = 22.659, 114.225

    # 统计
    duration = test_data.get('duration_sec', 0)
    total_dist = 0.0
    mode_time = {}
    for i in range(1, len(traj)):
        p0, p1 = traj[i-1], traj[i]
        dlat = math.radians(p1['lat'] - p0['lat'])
        dlon = math.radians(p1['lon'] - p0['lon'])
        a = math.sin(dlat/2)**2 + math.cos(math.radians(p0['lat'])) * math.cos(math.radians(p1['lat'])) * math.sin(dlon/2)**2
        total_dist += 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        m = p0.get('mode', 'unknown')
        dt = p1['t'] - p0['t']
        mode_time[m] = mode_time.get(m, 0) + dt

    nav_count = sum(1 for e in events if e['type'] == 'nav_trigger')
    stop_count = sum(1 for e in events if e['type'] == 'emergency_stop')
    arrive_count = sum(1 for e in events if e['type'] == 'nav_complete' and 'arrived' in e.get('data', ''))

    wp_count = len(wp_data.get('waypoints', {}))
    edge_count = len(wp_data.get('edges', []))

    return f'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>测试地图 — {gen_time}</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
* {{ margin: 0; padding: 0; box-sizing: border-box; }}
body {{ font-family: -apple-system, BlinkMacSystemFont, sans-serif; }}
#map {{ width: 100%; height: 100vh; }}
#info {{
  position: absolute; top: 10px; left: 60px; z-index: 1000;
  background: rgba(255,255,255,0.95); padding: 12px 16px;
  border-radius: 8px; box-shadow: 0 2px 8px rgba(0,0,0,0.25);
  font-size: 13px; max-width: 380px;
}}
#info h3 {{ margin-bottom: 8px; font-size: 15px; }}
.stat {{ color: #555; margin: 2px 0; }}
.stat b {{ color: #222; }}
.legend {{ margin-top: 8px; font-size: 11px; }}
.legend span {{ display: inline-block; width: 14px; height: 3px; margin-right: 4px; vertical-align: middle; }}
#controls {{
  position: absolute; bottom: 20px; left: 50%; transform: translateX(-50%);
  z-index: 1000; display: flex; gap: 6px;
}}
#controls button {{
  background: white; border: 1px solid #ccc; border-radius: 6px;
  padding: 6px 14px; cursor: pointer; font-size: 12px;
  box-shadow: 0 1px 4px rgba(0,0,0,0.15);
}}
#controls button:hover {{ background: #f0f0f0; }}
#controls button.active {{ background: #2a7; color: white; border-color: #2a7; }}
.wp-label {{ background: none; border: none; box-shadow: none; font-size: 10px; font-weight: bold; color: #333; }}
</style>
</head>
<body>
<div id="info">
  <h3>测试地图 (离线)</h3>
  <div class="stat">时间: <b>{test_data.get("start_time","—")}</b> | 时长: <b>{duration:.0f}s</b></div>
  <div class="stat">轨迹点: <b>{len(traj)}</b> | 距离: <b>{total_dist:.0f}m</b></div>
  <div class="stat">事件: 导航<b>{nav_count}</b> 急停<b>{stop_count}</b> 到达<b>{arrive_count}</b></div>
  <div class="stat">路点: <b>{wp_count}</b> | 边: <b>{edge_count}</b></div>
  <div class="legend">
    <span style="background:#3498db"></span>跟随
    <span style="background:#2ecc71"></span>导航
    <span style="background:#e74c3c"></span>急停
    <span style="background:#999"></span>其他
  </div>
  <div style="margin-top:4px;font-size:11px;color:#888">生成: {gen_time}</div>
</div>
<div id="controls">
  <button onclick="fitAll()">全部显示</button>
  <button id="btn-traj" class="active" onclick="toggle('traj')">轨迹</button>
  <button id="btn-wp" class="active" onclick="toggle('wp')">路点</button>
  <button id="btn-evt" class="active" onclick="toggle('evt')">事件</button>
  <button onclick="toggleBase()">切换底图</button>
</div>
<div id="map"></div>
<script>
const WP_DATA = {wp_json};
const TEST_DATA = {test_json};

const map = L.map('map').setView([{c_lat}, {c_lon}], 18);
const satLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}', {{maxZoom:22}});
const osmLayer = L.tileLayer('https://{{s}}.basemaps.cartocdn.com/rastertiles/voyager/{{z}}/{{x}}/{{y}}.png', {{maxZoom:22, subdomains:'abcd'}});
satLayer.addTo(map);
let useSat = true;

const trajGroup = L.layerGroup().addTo(map);
const wpGroup = L.layerGroup().addTo(map);
const evtGroup = L.layerGroup().addTo(map);

const MODE_COLOR = {{following:'#3498db', navigation:'#2ecc71', e_stop:'#e74c3c'}};

// 画轨迹（按模式分段着色）
function drawTrajectory() {{
  trajGroup.clearLayers();
  const traj = TEST_DATA.gps_trajectory || [];
  if (traj.length < 2) return;
  let seg = [traj[0]]; let mode = traj[0].mode;
  for (let i = 1; i < traj.length; i++) {{
    if (traj[i].mode !== mode || i === traj.length - 1) {{
      if (i === traj.length - 1) seg.push(traj[i]);
      const color = MODE_COLOR[mode] || '#999';
      const latlngs = seg.map(p => [p.lat, p.lon]);
      const line = L.polyline(latlngs, {{color, weight: 4, opacity: 0.8}});
      trajGroup.addLayer(line);
      // 每段中间放航向箭头
      if (seg.length > 4) {{
        const mid = seg[Math.floor(seg.length/2)];
        const arrow = L.marker([mid.lat, mid.lon], {{
          icon: L.divIcon({{className:'wp-label',
            html:`<span style="color:${{color}};font-size:12px">v${{mid.speed.toFixed(1)}}</span>`,
            iconAnchor:[0,-8]}})
        }});
        trajGroup.addLayer(arrow);
      }}
      seg = [traj[i]]; mode = traj[i].mode;
    }} else {{
      seg.push(traj[i]);
    }}
  }}
  // 起点终点标记
  if (traj.length > 0) {{
    L.circleMarker([traj[0].lat, traj[0].lon], {{radius:8, color:'#fff', fillColor:'#2ecc71', fillOpacity:1, weight:2}})
      .bindPopup('起点').addTo(trajGroup);
    const last = traj[traj.length-1];
    L.circleMarker([last.lat, last.lon], {{radius:8, color:'#fff', fillColor:'#e74c3c', fillOpacity:1, weight:2}})
      .bindPopup('终点').addTo(trajGroup);
  }}
}}

// 画路点
function drawWaypoints() {{
  wpGroup.clearLayers();
  const wps = WP_DATA.waypoints || {{}};
  const edges = WP_DATA.edges || [];
  for (const e of edges) {{
    const a = wps[e.from], b = wps[e.to];
    if (!a || !b) continue;
    L.polyline([[a.lat,a.lon],[b.lat,b.lon]], {{color:'#f39c12', weight:2, opacity:0.5, dashArray:'5,5'}}).addTo(wpGroup);
  }}
  let idx = 0;
  for (const [id, wp] of Object.entries(wps)) {{
    idx++;
    L.circleMarker([wp.lat, wp.lon], {{radius:4, color:'#fff', weight:1, fillColor:'#f39c12', fillOpacity:0.8}})
      .bindPopup(`<b>${{id}}</b><br>${{wp.lat.toFixed(7)}}, ${{wp.lon.toFixed(7)}}<br>经过: ${{wp.traverse_count}}次`)
      .addTo(wpGroup);
  }}
}}

// 画事件
function drawEvents() {{
  evtGroup.clearLayers();
  const events = TEST_DATA.events || [];
  const traj = TEST_DATA.gps_trajectory || [];
  for (const evt of events) {{
    // 找最近的轨迹点确定位置
    let best = null, bestDt = Infinity;
    for (const p of traj) {{
      const dt = Math.abs(p.t - evt.t);
      if (dt < bestDt) {{ bestDt = dt; best = p; }}
    }}
    if (!best) continue;
    const icons = {{nav_trigger:'#2ecc71', nav_complete:'#e67e22', emergency_stop:'#e74c3c', mode_change:'#9b59b6', follow_state:'#3498db'}};
    const labels = {{nav_trigger:'NAV', nav_complete:'END', emergency_stop:'STOP', mode_change:'MODE', follow_state:'FOL'}};
    const color = icons[evt.type] || '#999';
    const label = labels[evt.type] || '?';
    const marker = L.circleMarker([best.lat, best.lon], {{radius:6, color, fillColor:color, fillOpacity:0.9, weight:1}});
    marker.bindPopup(`<b>${{evt.type}}</b><br>t=${{evt.t}}s<br>${{evt.data}}`);
    evtGroup.addLayer(marker);
    const tag = L.marker([best.lat, best.lon], {{
      icon: L.divIcon({{className:'wp-label', html:`<span style="color:${{color}};font-size:9px;background:rgba(255,255,255,0.8);padding:1px 3px;border-radius:2px">${{label}}</span>`, iconAnchor:[0,-10]}})
    }});
    evtGroup.addLayer(tag);
  }}
}}

function fitAll() {{
  const bounds = [];
  (TEST_DATA.gps_trajectory||[]).forEach(p => bounds.push([p.lat,p.lon]));
  Object.values(WP_DATA.waypoints||{{}}).forEach(w => bounds.push([w.lat,w.lon]));
  if (bounds.length) map.fitBounds(L.latLngBounds(bounds), {{padding:[60,60], maxZoom:19}});
}}

const layers = {{traj:true, wp:true, evt:true}};
function toggle(key) {{
  layers[key] = !layers[key];
  const btn = document.getElementById('btn-'+key);
  btn.classList.toggle('active', layers[key]);
  const g = {{traj:trajGroup, wp:wpGroup, evt:evtGroup}}[key];
  layers[key] ? g.addTo(map) : map.removeLayer(g);
}}

function toggleBase() {{
  useSat = !useSat;
  map.removeLayer(satLayer); map.removeLayer(osmLayer);
  (useSat ? satLayer : osmLayer).addTo(map);
}}

// 点击轨迹点弹出详情
map.on('click', function(e) {{
  const traj = TEST_DATA.gps_trajectory || [];
  let best = null, bestD = Infinity;
  for (const p of traj) {{
    const d = Math.abs(p.lat - e.latlng.lat) + Math.abs(p.lon - e.latlng.lng);
    if (d < bestD) {{ bestD = d; best = p; }}
  }}
  if (best && bestD < 0.0001) {{
    L.popup().setLatLng([best.lat, best.lon])
      .setContent(`t=${{best.t}}s<br>v=${{best.speed.toFixed(2)}}m/s w=${{best.angular.toFixed(2)}}<br>heading=${{best.heading.toFixed(1)}}deg<br>mode=${{best.mode}}`)
      .openOn(map);
  }}
}});

L.DomEvent.disableClickPropagation(document.getElementById('controls'));
L.DomEvent.disableClickPropagation(document.getElementById('info'));

drawTrajectory();
drawWaypoints();
drawEvents();
fitAll();
</script>
</body>
</html>'''


def main():
    wp_data, test_data = load_data()

    gen_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    html = generate_html(wp_data, test_data, gen_time)

    os.makedirs(DATA_DIR, exist_ok=True)
    timestamped = os.path.join(DATA_DIR, f'test_{timestamp}.html')
    latest = os.path.join(DATA_DIR, 'test_latest.html')

    with open(timestamped, 'w') as f:
        f.write(html)
    with open(latest, 'w') as f:
        f.write(html)

    print(f'[OK] 生成完成:')
    print(f'  {timestamped}')
    print(f'  {latest}')
    print(f'')
    print(f'传到笔记本: scp {latest} laptop:~/Desktop/')


if __name__ == '__main__':
    main()
