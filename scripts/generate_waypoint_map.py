#!/usr/bin/env python3
"""离线路点地图生成器

读取 path_graph.json，生成自包含 HTML 文件。
笔记本双击打开即可在卫星地图上查看路点。

用法:
    # 1. 从 WheelTec 拷贝路点数据到 VPS
    scp -P 2225 wheeltec@localhost:~/golf_ws/data/path_graph.json ~/golf_ws/data/

    # 2. 生成离线地图
    python3 ~/golf_ws/scripts/generate_waypoint_map.py

    # 3. 传到笔记本
    scp ~/golf_ws/data/waypoints_latest.html laptop:~/Desktop/

    # 可选: 指定输入文件
    python3 ~/golf_ws/scripts/generate_waypoint_map.py /path/to/path_graph.json
"""

import json
import os
import sys
from datetime import datetime

# === 路径 ===
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
DATA_DIR = os.path.join(PROJECT_DIR, 'data')
DEFAULT_INPUT = os.path.join(DATA_DIR, 'path_graph.json')


def load_waypoint_data(input_path):
    """加载并验证路点 JSON"""
    with open(input_path, 'r') as f:
        data = json.load(f)

    wps = data.get('waypoints', {})
    edges = data.get('edges', [])
    print(f'[OK] 加载 {len(wps)} 个路点, {len(edges)} 条边')

    if len(wps) == 0:
        print('[WARN] 路点为空! 生成的地图将没有路点显示')

    return data


def generate_html(data, gen_time):
    """生成自包含 HTML"""
    data_json = json.dumps(data, ensure_ascii=False)
    wp_count = len(data.get('waypoints', {}))
    edge_count = len(data.get('edges', []))
    total_dist = sum(e.get('distance', 0) for e in data.get('edges', []))

    # 计算中心点（路点均值），没有路点则默认东莞
    wps = data.get('waypoints', {})
    if wps:
        lats = [w['lat'] for w in wps.values()]
        lons = [w['lon'] for w in wps.values()]
        center_lat = sum(lats) / len(lats)
        center_lon = sum(lons) / len(lons)
    else:
        center_lat, center_lon = 22.659, 114.225

    return f'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>GPS 路点地图 — {gen_time}</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
* {{ margin: 0; padding: 0; box-sizing: border-box; }}
body {{ font-family: -apple-system, BlinkMacSystemFont, sans-serif; }}
#map {{ width: 100%; height: 100vh; }}
#info {{
  position: absolute; top: 10px; left: 60px; z-index: 1000;
  background: rgba(255,255,255,0.95); padding: 10px 16px;
  border-radius: 8px; box-shadow: 0 2px 8px rgba(0,0,0,0.2);
  font-size: 14px; max-width: 360px;
}}
#info h3 {{ margin-bottom: 6px; font-size: 16px; }}
#info .stat {{ color: #555; margin: 2px 0; }}
#info .stat b {{ color: #333; }}
#info .gen-time {{ margin-top: 6px; font-size: 11px; color: #888; }}
#controls {{
  position: absolute; bottom: 20px; left: 50%; transform: translateX(-50%);
  z-index: 1000; display: flex; gap: 8px;
}}
#controls button {{
  background: white; border: 1px solid #ccc; border-radius: 6px;
  padding: 8px 16px; cursor: pointer; font-size: 13px;
  box-shadow: 0 1px 4px rgba(0,0,0,0.15);
}}
#controls button:hover {{ background: #f0f0f0; }}
.wp-label {{
  background: none; border: none; box-shadow: none;
  font-size: 11px; font-weight: bold; color: #333;
}}
</style>
</head>
<body>

<div id="info">
  <h3>GPS 路点地图 (离线)</h3>
  <div class="stat">路点: <b>{wp_count}</b> | 边: <b>{edge_count}</b></div>
  <div class="stat">总距离: <b>{total_dist:.1f}</b> m</div>
  <div class="stat">标记点: <b>{sum(1 for w in wps.values() if w.get('label'))}</b></div>
  <div class="gen-time">生成时间: {gen_time}</div>
</div>

<div id="controls">
  <button onclick="fitBounds()">全部显示</button>
  <button onclick="toggleSatellite()">当前: Esri → 切OSM</button>
</div>

<div id="map"></div>

<script>
// === 内嵌路点数据 ===
const WAYPOINT_DATA = {data_json};

// 地图初始化
const map = L.map('map').setView([{center_lat}, {center_lon}], 18);

// 图层
const osmLayer = L.tileLayer('https://{{s}}.basemaps.cartocdn.com/rastertiles/voyager/{{z}}/{{x}}/{{y}}.png', {{
  maxZoom: 22, maxNativeZoom: 20, subdomains: 'abcd',
  attribution: 'CartoDB Voyager (WGS-84)'
}});
const satLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}', {{
  maxZoom: 22, attribution: 'Esri Satellite (WGS-84)'
}});
const gaodeLayer = L.tileLayer('https://webst0{{s}}.is.autonavi.com/appmaptile?style=6&x={{x}}&y={{y}}&z={{z}}', {{
  maxZoom: 20, subdomains: '1234', attribution: '高德卫星 (GCJ-02)'
}});
satLayer.addTo(map);
let currentBase = 'esri';

// 路点图层
const wpLayerGroup = L.layerGroup().addTo(map);
const edgeLayerGroup = L.layerGroup().addTo(map);
const labelLayerGroup = L.layerGroup().addTo(map);

// 颜色
const COLOR_WP = '#e74c3c';
const COLOR_LABELED = '#f39c12';
const COLOR_EDGE = '#3498db';
const COLOR_LATEST = '#2ecc71';

function toggleSatellite() {{
  map.removeLayer(satLayer);
  map.removeLayer(osmLayer);
  map.removeLayer(gaodeLayer);
  if (currentBase === 'esri') {{
    osmLayer.addTo(map);
    currentBase = 'osm';
    document.querySelector('#controls button:nth-child(2)').textContent = '当前: OSM → 切高德';
  }} else if (currentBase === 'osm') {{
    gaodeLayer.addTo(map);
    currentBase = 'gaode';
    document.querySelector('#controls button:nth-child(2)').textContent = '当前: 高德 → 切Esri';
  }} else {{
    satLayer.addTo(map);
    currentBase = 'esri';
    document.querySelector('#controls button:nth-child(2)').textContent = '当前: Esri → 切OSM';
  }}
}}

function fitBounds() {{
  const wps = Object.values(WAYPOINT_DATA.waypoints || {{}});
  if (wps.length === 0) return;
  const bounds = L.latLngBounds(wps.map(w => [w.lat, w.lon]));
  map.fitBounds(bounds, {{ padding: [60, 60], maxZoom: 19 }});
}}

function renderData(data) {{
  const wps = data.waypoints || {{}};
  const edges = data.edges || [];

  // 找最新路点
  let latestId = null;
  let latestTime = '';
  for (const [id, wp] of Object.entries(wps)) {{
    if (wp.last_traversed > latestTime) {{
      latestTime = wp.last_traversed;
      latestId = id;
    }}
  }}

  // 画边
  for (const edge of edges) {{
    const from = wps[edge.from];
    const to = wps[edge.to];
    if (!from || !to) continue;
    const line = L.polyline(
      [[from.lat, from.lon], [to.lat, to.lon]],
      {{ color: COLOR_EDGE, weight: 3, opacity: 0.7 }}
    );
    line.bindTooltip(`${{edge.distance?.toFixed(1) || '?'}}m`, {{
      permanent: false, direction: 'center'
    }});
    edgeLayerGroup.addLayer(line);
  }}

  // 画路点
  let idx = 0;
  for (const [id, wp] of Object.entries(wps)) {{
    idx++;
    const isLabeled = !!wp.label;
    const isLatest = id === latestId;
    const color = isLatest ? COLOR_LATEST : (isLabeled ? COLOR_LABELED : COLOR_WP);
    const radius = isLatest ? 8 : (isLabeled ? 7 : 5);

    const marker = L.circleMarker([wp.lat, wp.lon], {{
      radius: radius,
      color: '#fff',
      weight: 2,
      fillColor: color,
      fillOpacity: 0.9,
    }});

    const popupText = [
      `<b>${{id}}</b>`,
      wp.label ? `标记: ${{wp.label}}` : null,
      `坐标: ${{wp.lat.toFixed(7)}}, ${{wp.lon.toFixed(7)}}`,
      `经过: ${{wp.traverse_count}} 次`,
      `最近: ${{wp.last_traversed}}`,
    ].filter(Boolean).join('<br>');
    marker.bindPopup(popupText);
    wpLayerGroup.addLayer(marker);

    // 标记点名称
    if (isLabeled) {{
      const label = L.marker([wp.lat, wp.lon], {{
        icon: L.divIcon({{
          className: 'wp-label',
          html: wp.label,
          iconAnchor: [-8, 4],
        }})
      }});
      labelLayerGroup.addLayer(label);
    }}

    // 序号标签
    const numLabel = L.marker([wp.lat, wp.lon], {{
      icon: L.divIcon({{
        className: 'wp-label',
        html: `<span style="color:${{color}};font-size:10px">${{idx}}</span>`,
        iconAnchor: [4, -8],
      }})
    }});
    labelLayerGroup.addLayer(numLabel);
  }}
}}

// 防止 Leaflet 拦截控件点击
L.DomEvent.disableClickPropagation(document.getElementById('controls'));
L.DomEvent.disableClickPropagation(document.getElementById('info'));

// 渲染并自动缩放
renderData(WAYPOINT_DATA);
if (Object.keys(WAYPOINT_DATA.waypoints || {{}}).length > 0) {{
  fitBounds();
}}
</script>
</body>
</html>'''


def main():
    # 输入文件
    input_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_INPUT

    if not os.path.exists(input_path):
        print(f'[ERROR] 文件不存在: {input_path}')
        print(f'')
        print(f'先从 WheelTec 拷贝路点数据:')
        print(f'  scp -P 2225 wheeltec@localhost:~/golf_ws/data/path_graph.json ~/golf_ws/data/')
        sys.exit(1)

    # 加载数据
    data = load_waypoint_data(input_path)

    # 生成 HTML
    gen_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    html = generate_html(data, gen_time)

    # 输出文件
    os.makedirs(DATA_DIR, exist_ok=True)
    timestamped = os.path.join(DATA_DIR, f'waypoints_{timestamp}.html')
    latest = os.path.join(DATA_DIR, 'waypoints_latest.html')

    with open(timestamped, 'w') as f:
        f.write(html)
    with open(latest, 'w') as f:
        f.write(html)

    print(f'[OK] 生成完成:')
    print(f'  带时间戳: {timestamped}')
    print(f'  最新版本: {latest}')
    print(f'')
    print(f'传到笔记本:')
    print(f'  scp {latest} laptop:~/Desktop/')


if __name__ == '__main__':
    main()
