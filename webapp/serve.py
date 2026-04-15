#!/usr/bin/env python3
"""Golf Cart Web APP Server

Serves the web control panel on port 8080.
Routes:
  /              → index.html
  /data/*.json   → ../data/*.json (path_graph.json etc.)
  /api/sync      → pull path_graph.json from WheelTec + rebuild HTML
"""

import http.server
import json
import os
import subprocess
import sys

PORT = 8080
WEBAPP_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(os.path.dirname(WEBAPP_DIR), 'data')
BUILD_SCRIPT = os.path.join(WEBAPP_DIR, 'build.sh')


class GolfHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEBAPP_DIR, **kwargs)

    def do_GET(self):
        # API: sync waypoints from WheelTec
        if self.path.startswith('/api/sync'):
            self._handle_sync()
            return
        # Route /data/* to the data directory
        if self.path.startswith('/data/'):
            rel = self.path[6:].split('?')[0]
            filepath = os.path.join(DATA_DIR, rel)
            if os.path.isfile(filepath):
                self.send_response(200)
                if filepath.endswith('.json'):
                    self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                with open(filepath, 'rb') as f:
                    self.wfile.write(f.read())
                return
            else:
                self.send_error(404, f'File not found: {rel}')
                return
        super().do_GET()

    def _handle_sync(self):
        """Pull path_graph.json from WheelTec via SSH, rebuild HTML"""
        steps = []
        try:
            # Step 1: scp from WheelTec
            r = subprocess.run(
                ['scp', '-P', '2225', '-o', 'ConnectTimeout=5',
                 'wheeltec@localhost:~/golf_ws/data/path_graph.json',
                 os.path.join(DATA_DIR, 'path_graph.json')],
                capture_output=True, text=True, timeout=15)
            if r.returncode != 0:
                steps.append(f'scp failed: {r.stderr.strip()}')
                self._json_response(False, 'WheelTec unreachable', steps)
                return
            steps.append('scp OK')

            # Step 2: rebuild HTML
            r = subprocess.run(
                ['bash', BUILD_SCRIPT],
                capture_output=True, text=True, timeout=10)
            steps.append(r.stdout.strip())

            self._json_response(True, 'Sync complete, refresh page', steps)
        except subprocess.TimeoutExpired:
            steps.append('timeout')
            self._json_response(False, 'SSH timeout - car offline?', steps)
        except Exception as e:
            self._json_response(False, str(e), steps)

    def _json_response(self, ok, message, details=None):
        data = json.dumps({'ok': ok, 'message': message, 'details': details or []})
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(data.encode())

    def log_message(self, format, *args):
        if '200' not in str(args):
            sys.stderr.write(f'[WebAPP] {args[0]}\n')


def main():
    with http.server.ThreadingHTTPServer(('0.0.0.0', PORT), GolfHandler) as httpd:
        print(f'Golf Cart Web APP serving on http://0.0.0.0:{PORT}')
        print(f'  HTML: {WEBAPP_DIR}')
        print(f'  Data: {DATA_DIR}')
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print('\nShutting down.')


if __name__ == '__main__':
    main()
