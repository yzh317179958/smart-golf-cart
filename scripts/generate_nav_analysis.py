#!/usr/bin/env python3
"""导航路径分析可视化

读取 path_graph.json，生成包含转弯角度分析、危险区域标注的增强地图。
用于诊断拐弯撞墙等路径偏差问题。

用法:
    python3 ~/golf_ws/scripts/generate_nav_analysis.py
    scp ~/golf_ws/data/nav_analysis_latest.html laptop:~/Desktop/
"""

import json
import math
import os
from datetime import datetime

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
DATA_DIR = os.path.join(PROJECT_DIR, 'data')

EARTH_R = 6371000.0


def haversine(lat1, lon1, lat2, lon2):
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return EARTH_R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def compass_bearing(lat1, lon1, lat2, lon2):
    dlat = lat2 - lat1
    dlon = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2.0))
    bearing = math.degrees(math.atan2(dlon, dlat))
    return bearing % 360.0


def analyze_path(wp_data):
    """分析路径几何特性"""
    wps = wp_data.get('waypoints', {})
    edges = wp_data.get('edges', [])

    # 构建有序路点链
    chain = []
    if edges:
        chain = [edges[0]['from']]
        for e in edges:
            chain.append(e['to'])

    # 计算每个路点的属性
    analysis = []
    for i, wp_id in enumerate(chain):
        wp = wps[wp_id]
        entry = {
            'id': wp_id,
            'idx': i,
            'lat': wp['lat'],
            'lon': wp['lon'],
            'traverse_count': wp.get('traverse_count', 1),
        }

        # 计算到前一个路点的距离和方位角
        if i > 0:
            prev = wps[chain[i - 1]]
            entry['dist_from_prev'] = haversine(prev['lat'], prev['lon'], wp['lat'], wp['lon'])
            entry['bearing_from_prev'] = compass_bearing(prev['lat'], prev['lon'], wp['lat'], wp['lon'])
        else:
            entry['dist_from_prev'] = 0
            entry['bearing_from_prev'] = None

        # 计算到下一个路点的方位角
        if i < len(chain) - 1:
            nxt = wps[chain[i + 1]]
            entry['bearing_to_next'] = compass_bearing(wp['lat'], wp['lon'], nxt['lat'], nxt['lon'])
        else:
            entry['bearing_to_next'] = None

        # 计算转弯角度
        if entry['bearing_from_prev'] is not None and entry['bearing_to_next'] is not None:
            diff = entry['bearing_to_next'] - entry['bearing_from_prev']
            while diff > 180:
                diff -= 360
            while diff < -180:
                diff += 360
            entry['turn_angle'] = diff  # 正=右转，负=左转
            entry['turn_abs'] = abs(diff)
        else:
            entry['turn_angle'] = 0
            entry['turn_abs'] = 0

        analysis.append(entry)

    # 识别危险区域（急弯）
    danger_zones = []
    for entry in analysis:
        if entry['turn_abs'] >= 20:
            severity = 'warning' if entry['turn_abs'] < 45 else ('danger' if entry['turn_abs'] < 90 else 'critical')
            danger_zones.append({
                'wp_id': entry['id'],
                'idx': entry['idx'],
                'lat': entry['lat'],
                'lon': entry['lon'],
                'turn_angle': entry['turn_angle'],
                'turn_abs': entry['turn_abs'],
                'severity': severity,
                'direction': 'right' if entry['turn_angle'] > 0 else 'left',
            })

    # 计算累积转弯（连续小弯累加效果）
    cumulative_turns = []
    window = 5  # 5个路点的窗口
    for i in range(len(analysis)):
        start = max(0, i - window + 1)
        cum = sum(a['turn_angle'] for a in analysis[start:i + 1])
        cum_abs = sum(a['turn_abs'] for a in analysis[start:i + 1])
        cumulative_turns.append({
            'idx': i,
            'wp_id': analysis[i]['id'],
            'cum_turn': cum,
            'cum_abs': cum_abs,
        })

    # 计算Ackermann约束下的偏差估算
    # 最小转弯半径 1.65m，路点间距 ~3m
    ackermann_issues = []
    min_r = 1.65
    for entry in analysis:
        if entry['turn_abs'] > 5 and entry['dist_from_prev'] > 0:
            # 从弧度和弦长估算所需转弯半径
            turn_rad = math.radians(entry['turn_abs'])
            if turn_rad > 0.01:
                chord = entry['dist_from_prev']
                required_r = chord / (2 * math.sin(turn_rad / 2)) if turn_rad < math.pi else chord / 2
                if required_r < min_r:
                    # 计算偏差量：实际路径 vs Ackermann约束路径
                    deviation = min_r * (1 - math.cos(turn_rad / 2)) - required_r * (1 - math.cos(turn_rad / 2))
                    ackermann_issues.append({
                        'wp_id': entry['id'],
                        'idx': entry['idx'],
                        'lat': entry['lat'],
                        'lon': entry['lon'],
                        'turn_angle': entry['turn_angle'],
                        'required_r': required_r,
                        'deviation_m': abs(deviation),
                    })

    # 总路径统计
    total_dist = sum(e['dist_from_prev'] for e in analysis)
    max_turn = max((e['turn_abs'] for e in analysis), default=0)
    avg_spacing = total_dist / max(1, len(analysis) - 1)

    stats = {
        'total_waypoints': len(chain),
        'total_distance': total_dist,
        'avg_spacing': avg_spacing,
        'max_turn_angle': max_turn,
        'sharp_turns_30': sum(1 for e in analysis if e['turn_abs'] >= 30),
        'sharp_turns_60': sum(1 for e in analysis if e['turn_abs'] >= 60),
        'sharp_turns_90': sum(1 for e in analysis if e['turn_abs'] >= 90),
        'danger_zones': len(danger_zones),
        'ackermann_issues': len(ackermann_issues),
    }

    return analysis, danger_zones, cumulative_turns, ackermann_issues, stats


def generate_html(wp_data, analysis, danger_zones, cum_turns, ack_issues, stats, gen_time):
    """生成增强版分析HTML"""
    wp_json = json.dumps(wp_data, ensure_ascii=False)
    analysis_json = json.dumps(analysis, ensure_ascii=False)
    danger_json = json.dumps(danger_zones, ensure_ascii=False)
    cum_json = json.dumps(cum_turns, ensure_ascii=False)
    ack_json = json.dumps(ack_issues, ensure_ascii=False)

    # 中心点
    lats = [a['lat'] for a in analysis]
    lons = [a['lon'] for a in analysis]
    c_lat = sum(lats) / len(lats) if lats else 22.659
    c_lon = sum(lons) / len(lons) if lons else 114.225

    # 找出最危险的转弯区域用于文字报告
    top_turns = sorted(danger_zones, key=lambda x: x['turn_abs'], reverse=True)[:5]
    turn_report = ''
    for i, t in enumerate(top_turns):
        dir_cn = '右转' if t['direction'] == 'right' else '左转'
        sev_cn = {'warning': '注意', 'danger': '危险', 'critical': '极危险'}[t['severity']]
        turn_report += f'<div class="turn-item {t["severity"]}">{i+1}. {t["wp_id"]} — {dir_cn} {t["turn_abs"]:.0f}° [{sev_cn}]</div>'

    return f'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>导航路径分析 — {gen_time}</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
* {{ margin: 0; padding: 0; box-sizing: border-box; }}
body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; }}
#map {{ width: 100%; height: 100vh; }}

#panel {{
  position: absolute; top: 10px; right: 10px; z-index: 1000;
  background: rgba(255,255,255,0.97); padding: 16px;
  border-radius: 10px; box-shadow: 0 4px 16px rgba(0,0,0,0.3);
  font-size: 13px; max-width: 420px; max-height: 90vh; overflow-y: auto;
}}
#panel h2 {{ font-size: 16px; margin-bottom: 10px; color: #1a1a1a; }}
#panel h3 {{ font-size: 14px; margin: 12px 0 6px; color: #333; border-bottom: 1px solid #eee; padding-bottom: 4px; }}

.stat-grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 4px 12px; }}
.stat-grid .label {{ color: #666; }}
.stat-grid .value {{ font-weight: 600; color: #222; text-align: right; }}

.turn-item {{ padding: 4px 8px; margin: 2px 0; border-radius: 4px; font-size: 12px; }}
.turn-item.warning {{ background: #fff3cd; border-left: 3px solid #ffc107; }}
.turn-item.danger {{ background: #f8d7da; border-left: 3px solid #dc3545; }}
.turn-item.critical {{ background: #f5c6cb; border-left: 3px solid #c82333; font-weight: bold; }}

.issue-item {{ padding: 6px 8px; margin: 3px 0; border-radius: 4px; background: #e2e3f1; border-left: 3px solid #6c63ff; font-size: 12px; }}

.legend {{ margin-top: 10px; }}
.legend-item {{ display: flex; align-items: center; gap: 6px; margin: 3px 0; font-size: 12px; }}
.legend-dot {{ width: 12px; height: 12px; border-radius: 50%; }}
.legend-line {{ width: 20px; height: 3px; }}

#controls {{
  position: absolute; bottom: 20px; left: 50%; transform: translateX(-50%);
  z-index: 1000; display: flex; gap: 6px;
}}
#controls button {{
  background: white; border: 1px solid #ccc; border-radius: 6px;
  padding: 6px 14px; cursor: pointer; font-size: 12px;
  box-shadow: 0 1px 4px rgba(0,0,0,0.15); transition: all 0.2s;
}}
#controls button:hover {{ background: #f0f0f0; }}
#controls button.active {{ background: #2a7; color: white; border-color: #2a7; }}

.wp-label {{ background: none; border: none; box-shadow: none; }}
.arrow-icon {{ background: none; border: none; box-shadow: none; }}
</style>
</head>
<body>

<div id="panel">
  <h2>导航路径分析报告</h2>
  <div style="font-size:11px;color:#888;margin-bottom:8px;">生成时间: {gen_time}</div>

  <h3>路径统计</h3>
  <div class="stat-grid">
    <span class="label">总路点数</span><span class="value">{stats['total_waypoints']}</span>
    <span class="label">总距离</span><span class="value">{stats['total_distance']:.0f} m</span>
    <span class="label">平均间距</span><span class="value">{stats['avg_spacing']:.1f} m</span>
    <span class="label">最大转角</span><span class="value">{stats['max_turn_angle']:.0f}°</span>
    <span class="label">急弯(&ge;30°)</span><span class="value">{stats['sharp_turns_30']} 处</span>
    <span class="label">大弯(&ge;60°)</span><span class="value">{stats['sharp_turns_60']} 处</span>
    <span class="label">直角弯(&ge;90°)</span><span class="value">{stats['sharp_turns_90']} 处</span>
  </div>

  <h3>危险转弯 TOP5</h3>
  {turn_report if turn_report else '<div style="color:#888">无危险转弯</div>'}

  <h3>Ackermann 约束分析</h3>
  <div style="font-size:12px;color:#555;margin-bottom:6px;">
    最小转弯半径: 1.65m | 路点间距: ~{stats['avg_spacing']:.1f}m
  </div>
  <div id="ack-report"></div>

  <h3>诊断结论</h3>
  <div id="diagnosis" style="font-size:12px; line-height:1.6; color:#333;"></div>

  <div class="legend">
    <h3>图例</h3>
    <div class="legend-item"><div class="legend-dot" style="background:#3498db"></div>正常路点</div>
    <div class="legend-item"><div class="legend-dot" style="background:#ffc107"></div>注意弯道 (20-45°)</div>
    <div class="legend-item"><div class="legend-dot" style="background:#dc3545"></div>危险弯道 (45-90°)</div>
    <div class="legend-item"><div class="legend-dot" style="background:#c82333"></div>极危险弯道 (&ge;90°)</div>
    <div class="legend-item"><div class="legend-line" style="background:#2ecc71"></div>直行段</div>
    <div class="legend-item"><div class="legend-line" style="background:#e74c3c"></div>急弯段</div>
    <div class="legend-item"><div class="legend-line" style="background:rgba(108,99,255,0.3);height:12px;border:1px dashed #6c63ff"></div>Ackermann 最小转弯圆弧</div>
  </div>
</div>

<div id="controls">
  <button onclick="fitAll()">全部显示</button>
  <button id="btn-path" class="active" onclick="toggle('path')">路径</button>
  <button id="btn-turns" class="active" onclick="toggle('turns')">转弯分析</button>
  <button id="btn-ack" class="active" onclick="toggle('ack')">Ackermann</button>
  <button id="btn-arrows" class="active" onclick="toggle('arrows')">方向箭头</button>
  <button onclick="toggleBase()">切换底图</button>
</div>

<div id="map"></div>
<script>
const WP_DATA = {wp_json};
const ANALYSIS = {analysis_json};
const DANGER = {danger_json};
const CUM_TURNS = {cum_json};
const ACK_ISSUES = {ack_json};
const MIN_TURN_R = 1.65;

const map = L.map('map').setView([{c_lat}, {c_lon}], 18);
const satLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}', {{maxZoom:22}});
const osmLayer = L.tileLayer('https://{{s}}.basemaps.cartocdn.com/rastertiles/voyager/{{z}}/{{x}}/{{y}}.png', {{maxZoom:22, subdomains:'abcd'}});
satLayer.addTo(map);
let useSat = true;

const pathGroup = L.layerGroup().addTo(map);
const turnsGroup = L.layerGroup().addTo(map);
const ackGroup = L.layerGroup().addTo(map);
const arrowsGroup = L.layerGroup().addTo(map);

// 颜色函数
function turnColor(absAngle) {{
  if (absAngle >= 90) return '#c82333';
  if (absAngle >= 45) return '#dc3545';
  if (absAngle >= 20) return '#ffc107';
  return '#3498db';
}}

function segColor(absAngle) {{
  if (absAngle >= 30) return '#e74c3c';
  if (absAngle >= 15) return '#e67e22';
  return '#2ecc71';
}}

// 画路径
function drawPath() {{
  pathGroup.clearLayers();
  for (let i = 0; i < ANALYSIS.length - 1; i++) {{
    const a = ANALYSIS[i], b = ANALYSIS[i+1];
    const color = segColor(b.turn_abs);
    L.polyline([[a.lat,a.lon],[b.lat,b.lon]], {{
      color, weight: 4, opacity: 0.85
    }}).bindPopup(`${{a.id}} → ${{b.id}}<br>距离: ${{b.dist_from_prev.toFixed(1)}}m<br>方位: ${{b.bearing_from_prev ? b.bearing_from_prev.toFixed(0) : '-'}}°`)
      .addTo(pathGroup);
  }}

  // 路点标记
  ANALYSIS.forEach((a, i) => {{
    const color = turnColor(a.turn_abs);
    const radius = a.turn_abs >= 30 ? 7 : 4;
    L.circleMarker([a.lat, a.lon], {{
      radius, color: '#fff', weight: 1.5, fillColor: color, fillOpacity: 0.9
    }}).bindPopup(
      `<b>${{a.id}}</b> (#${{a.idx}})<br>`+
      `坐标: ${{a.lat.toFixed(7)}}, ${{a.lon.toFixed(7)}}<br>`+
      `转弯角: ${{a.turn_angle.toFixed(1)}}°<br>`+
      `距前点: ${{a.dist_from_prev.toFixed(1)}}m<br>`+
      `经过次数: ${{a.traverse_count}}`
    ).addTo(pathGroup);

    // 每5个点标注路点ID
    if (i % 5 === 0 || a.turn_abs >= 30) {{
      L.marker([a.lat, a.lon], {{
        icon: L.divIcon({{
          className: 'wp-label',
          html: `<span style="font-size:${{a.turn_abs>=30?'11':'9'}}px;font-weight:${{a.turn_abs>=30?'bold':'normal'}};color:${{a.turn_abs>=30?'#e74c3c':'#fff'}};text-shadow:1px 1px 2px #000,-1px -1px 2px #000">${{a.id.replace('wp_','#')}}</span>`,
          iconAnchor: [0, -10]
        }})
      }}).addTo(pathGroup);
    }}
  }});

  // 起点终点
  if (ANALYSIS.length > 0) {{
    const s = ANALYSIS[0], e = ANALYSIS[ANALYSIS.length-1];
    L.circleMarker([s.lat, s.lon], {{radius:10, color:'#fff', fillColor:'#2ecc71', fillOpacity:1, weight:3}})
      .bindPopup('起点: '+s.id).addTo(pathGroup);
    L.circleMarker([e.lat, e.lon], {{radius:10, color:'#fff', fillColor:'#e74c3c', fillOpacity:1, weight:3}})
      .bindPopup('终点: '+e.id).addTo(pathGroup);
  }}
}}

// 画转弯分析
function drawTurns() {{
  turnsGroup.clearLayers();
  DANGER.forEach(d => {{
    const colors = {{warning:'#ffc107', danger:'#dc3545', critical:'#c82333'}};
    const color = colors[d.severity] || '#ffc107';
    const dir = d.turn_angle > 0 ? '右' : '左';

    // 大圆标注
    L.circle([d.lat, d.lon], {{
      radius: Math.min(8, 2 + d.turn_abs / 15),
      color, fillColor: color, fillOpacity: 0.3, weight: 2, dashArray: '4,4'
    }}).bindPopup(
      `<b>${{d.wp_id}} — ${{dir}}转 ${{d.turn_abs.toFixed(0)}}°</b><br>`+
      `严重程度: ${{d.severity}}`
    ).addTo(turnsGroup);

    // 转弯角度标注
    L.marker([d.lat, d.lon], {{
      icon: L.divIcon({{
        className: 'wp-label',
        html: `<div style="background:${{color}};color:white;padding:2px 6px;border-radius:10px;font-size:11px;font-weight:bold;white-space:nowrap;box-shadow:0 1px 3px rgba(0,0,0,0.3)">${{dir}}${{d.turn_abs.toFixed(0)}}°</div>`,
        iconAnchor: [-15, 5]
      }})
    }}).addTo(turnsGroup);
  }});
}}

// 画方向箭头
function drawArrows() {{
  arrowsGroup.clearLayers();
  for (let i = 0; i < ANALYSIS.length; i++) {{
    const a = ANALYSIS[i];
    if (!a.bearing_to_next && a.bearing_to_next !== 0) continue;
    if (i % 3 !== 0 && a.turn_abs < 20) continue; // 只在每3个点或急弯画箭头

    const bearing = a.bearing_to_next;
    // CSS 箭头
    L.marker([a.lat, a.lon], {{
      icon: L.divIcon({{
        className: 'arrow-icon',
        html: `<div style="transform:rotate(${{bearing}}deg);font-size:16px;color:${{turnColor(a.turn_abs)}};text-shadow:0 0 3px #000">&#x2191;</div>`,
        iconSize: [20, 20],
        iconAnchor: [10, 10]
      }})
    }}).addTo(arrowsGroup);
  }}
}}

// 画Ackermann约束圆弧
function drawAckermann() {{
  ackGroup.clearLayers();
  ACK_ISSUES.forEach(iss => {{
    // 画最小转弯圆
    L.circle([iss.lat, iss.lon], {{
      radius: MIN_TURN_R,
      color: '#6c63ff', fillColor: 'rgba(108,99,255,0.15)',
      fillOpacity: 0.3, weight: 1, dashArray: '4,4'
    }}).bindPopup(
      `<b>Ackermann 约束</b><br>`+
      `${{iss.wp_id}}: 需要半径 ${{iss.required_r.toFixed(2)}}m<br>`+
      `最小可转半径: ${{MIN_TURN_R}}m<br>`+
      `转角: ${{iss.turn_angle.toFixed(0)}}°<br>`+
      `估算偏差: ${{iss.deviation_m.toFixed(2)}}m`
    ).addTo(ackGroup);
  }});

  // 更新报告面板
  const ackDiv = document.getElementById('ack-report');
  if (ACK_ISSUES.length === 0) {{
    ackDiv.innerHTML = '<div style="color:#28a745">所有弯道半径满足 Ackermann 约束 (>1.65m)</div>';
  }} else {{
    let html = '';
    ACK_ISSUES.forEach(iss => {{
      html += `<div class="issue-item">${{iss.wp_id}}: 需要R=${{iss.required_r.toFixed(2)}}m < 1.65m, 转角${{iss.turn_angle.toFixed(0)}}°, 偏差~${{iss.deviation_m.toFixed(2)}}m</div>`;
    }});
    ackDiv.innerHTML = html;
  }}
}}

// 生成诊断结论
function generateDiagnosis() {{
  const diag = document.getElementById('diagnosis');
  let html = '';

  // 分析连续转弯区域
  const sharpRegions = [];
  let regionStart = -1;
  for (let i = 0; i < ANALYSIS.length; i++) {{
    if (ANALYSIS[i].turn_abs >= 20) {{
      if (regionStart < 0) regionStart = i;
    }} else if (regionStart >= 0) {{
      sharpRegions.push({{start: regionStart, end: i-1,
        maxTurn: Math.max(...ANALYSIS.slice(regionStart, i).map(a=>a.turn_abs)),
        totalTurn: ANALYSIS.slice(regionStart, i).reduce((s,a) => s + a.turn_angle, 0)
      }});
      regionStart = -1;
    }}
  }}
  if (regionStart >= 0) {{
    sharpRegions.push({{start: regionStart, end: ANALYSIS.length-1,
      maxTurn: Math.max(...ANALYSIS.slice(regionStart).map(a=>a.turn_abs)),
      totalTurn: ANALYSIS.slice(regionStart).reduce((s,a) => s + a.turn_angle, 0)
    }});
  }}

  html += '<b>发现 ' + sharpRegions.length + ' 个弯道区域:</b><br>';
  sharpRegions.forEach((r, i) => {{
    const startWp = ANALYSIS[r.start].id;
    const endWp = ANALYSIS[r.end].id;
    const dir = r.totalTurn > 0 ? '右' : '左';
    html += `&bull; 弯道${{i+1}}: ${{startWp}}→${{endWp}} (${{Math.abs(r.totalTurn).toFixed(0)}}° ${{dir}}转, 最急${{r.maxTurn.toFixed(0)}}°)<br>`;
  }});

  html += '<br><b>撞墙原因分析:</b><br>';
  html += '1. <span style="color:#dc3545">GPS路点间距~3m，急弯处路径切内角</span> — 录制路点时走弧线，但3m间距记录的是直线折线，MPPI沿折线走会切弯<br>';
  html += '2. <span style="color:#dc3545">样条平滑在急弯处过冲</span> — Catmull-Rom 样条会在控制点急转处向外凸出，推大实际轨迹<br>';
  html += '3. <span style="color:#e67e22">GPS ±1.5m 噪声在弯道叠加</span> — 弯道处多个路点的GPS偏差可能系统性偏向墙壁一侧<br>';
  html += '4. <span style="color:#e67e22">Ackermann 最小转弯半径 1.65m</span> — 部分急弯所需半径小于物理极限<br>';
  html += '5. <span style="color:#ffc107">MPPI 预测时域仅 2.8s (1.4m@0.5m/s)</span> — 看不到即将到来的急弯，来不及提前减速转向<br>';

  html += '<br><b>建议修复:</b><br>';
  html += '1. 录制路点时 <b>弯道处增加密度</b>（减小 min_record_distance 到 1.5m）<br>';
  html += '2. 弯道处 <b>自动减速</b>（turn_abs>30° 时 max_speed 降到 0.3m/s）<br>';
  html += '3. <b>增大样条间距</b> 在急弯附近（0.5m→1.0m），减少过冲<br>';
  html += '4. 考虑 <b>弯道外扩补偿</b>（根据转角自动向弯道外侧偏移路点）<br>';
  html += '5. 增加 MPPI <b>预测时域</b>（time_steps 28→40，看到更远的弯道）<br>';

  diag.innerHTML = html;
}}

function fitAll() {{
  const bounds = ANALYSIS.map(a => [a.lat, a.lon]);
  if (bounds.length) map.fitBounds(L.latLngBounds(bounds), {{padding:[60,60], maxZoom:19}});
}}

const layers = {{path:true, turns:true, ack:true, arrows:true}};
function toggle(key) {{
  layers[key] = !layers[key];
  document.getElementById('btn-'+key).classList.toggle('active', layers[key]);
  const g = {{path:pathGroup, turns:turnsGroup, ack:ackGroup, arrows:arrowsGroup}}[key];
  layers[key] ? g.addTo(map) : map.removeLayer(g);
}}

function toggleBase() {{
  useSat = !useSat;
  map.removeLayer(satLayer); map.removeLayer(osmLayer);
  (useSat ? satLayer : osmLayer).addTo(map);
}}

drawPath();
drawTurns();
drawAckermann();
drawArrows();
generateDiagnosis();
fitAll();
</script>
</body>
</html>'''


def main():
    wp_file = os.path.join(DATA_DIR, 'path_graph.json')
    if not os.path.exists(wp_file):
        print(f'[ERROR] 路点文件不存在: {wp_file}')
        return

    with open(wp_file) as f:
        wp_data = json.load(f)
    print(f'[OK] 加载 {len(wp_data.get("waypoints", {}))} 路点, {len(wp_data.get("edges", []))} 边')

    analysis, danger_zones, cum_turns, ack_issues, stats = analyze_path(wp_data)

    print(f'\n=== 路径分析 ===')
    print(f'总距离: {stats["total_distance"]:.0f}m')
    print(f'路点间距: {stats["avg_spacing"]:.1f}m')
    print(f'最大转角: {stats["max_turn_angle"]:.0f}°')
    print(f'急弯(>=30°): {stats["sharp_turns_30"]} 处')
    print(f'大弯(>=60°): {stats["sharp_turns_60"]} 处')
    print(f'Ackermann约束问题: {stats["ackermann_issues"]} 处')

    print(f'\n=== 危险转弯 ===')
    for d in sorted(danger_zones, key=lambda x: x['turn_abs'], reverse=True)[:10]:
        dir_cn = '右转' if d['direction'] == 'right' else '左转'
        print(f'  {d["wp_id"]} — {dir_cn} {d["turn_abs"]:.0f}° [{d["severity"]}]')

    gen_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    html = generate_html(wp_data, analysis, danger_zones, cum_turns, ack_issues, stats, gen_time)

    os.makedirs(DATA_DIR, exist_ok=True)
    out_file = os.path.join(DATA_DIR, f'nav_analysis_{timestamp}.html')
    latest = os.path.join(DATA_DIR, 'nav_analysis_latest.html')

    with open(out_file, 'w') as f:
        f.write(html)
    with open(latest, 'w') as f:
        f.write(html)

    print(f'\n[OK] 生成完成:')
    print(f'  {out_file}')
    print(f'  {latest}')
    print(f'\n传到笔记本:')
    print(f'  scp {latest} laptop:~/Desktop/')


if __name__ == '__main__':
    main()
