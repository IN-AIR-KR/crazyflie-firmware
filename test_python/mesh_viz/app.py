#!/usr/bin/env python3
"""
Crazyflie P2P Mesh 네트워크 시각화 서버 (WebSocket)

실행:
    python3 app.py [radio://0/80/2M/E7E7E7E7E7]

접속:
    http://localhost:1942
"""

import sys
import time
import struct
import threading
from collections import deque

from flask import Flask, render_template
from flask_socketio import SocketIO
import cflib.crtp
from cflib.crazyflie import Crazyflie

# ── 상수 ──────────────────────────────────────────────────────────────────────
CRTP_PORT_P2P_PROXY = 0x09
BEACON_FMT  = '<BBBBBBHhhh'
BEACON_SIZE = 14
NODE_TIMEOUT_SEC = 10

# ── 공유 상태 ─────────────────────────────────────────────────────────────────
lock = threading.Lock()
nodes: dict[int, dict] = {}
edges: dict[tuple, dict] = {}
recent_log: deque = deque(maxlen=30)

# ── Flask + SocketIO ──────────────────────────────────────────────────────────
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

# ── 패킷 파싱 & 상태 업데이트 ─────────────────────────────────────────────────
def on_packet(packet):
    if len(packet.data) < BEACON_SIZE:
        return
    vals = struct.unpack(BEACON_FMT, bytes(packet.data)[:BEACON_SIZE])
    beacon = dict(type=vals[0], src_id=vals[1], tx_id=vals[2],
                  seq=vals[3], ttl=vals[4], hop=vals[5], t_ms=vals[6],
                  x_cm=vals[7], y_cm=vals[8], z_cm=vals[9])
    if beacon['type'] != 1:
        return

    now = time.time()
    src, tx = beacon['src_id'], beacon['tx_id']
    x_m = beacon['x_cm'] / 100.0
    y_m = beacon['y_cm'] / 100.0
    z_m = beacon['z_cm'] / 100.0

    with lock:
        if src not in nodes:
            nodes[src] = {'seq': 0, 'hop': 0, 't_ms': 0, 'last_seen': now,
                          'count': 0, 'x_m': x_m, 'y_m': y_m, 'z_m': z_m}
        nodes[src].update(seq=beacon['seq'], hop=beacon['hop'],
                          t_ms=beacon['t_ms'], last_seen=now,
                          x_m=x_m, y_m=y_m, z_m=z_m)
        nodes[src]['count'] += 1

        key = (src, tx)
        if key not in edges:
            edges[key] = {'count': 0, 'last_seen': now}
        edges[key]['count'] += 1
        edges[key]['last_seen'] = now

        log_entry = {
            'time': time.strftime('%H:%M:%S'),
            'src': src, 'tx': tx,
            'seq': beacon['seq'], 'hop': beacon['hop'], 't_ms': beacon['t_ms'],
            'x_m': round(x_m, 2), 'y_m': round(y_m, 2), 'z_m': round(z_m, 2),
        }
        recent_log.appendleft(log_entry)

    # 새 패킷을 WebSocket으로 즉시 push
    socketio.emit('packet', log_entry)

# ── Crazyflie 연결 ────────────────────────────────────────────────────────────
cf = Crazyflie(rw_cache='./cache')
cf.incoming.add_port_callback(CRTP_PORT_P2P_PROXY, on_packet)

def connect(uri: str):
    cflib.crtp.init_drivers()
    cf.connected.add_callback(lambda u: print(f'[CRTP] 연결 성공: {u}'))
    cf.connection_failed.add_callback(lambda u, m: print(f'[CRTP] 연결 실패: {m}'))
    cf.disconnected.add_callback(lambda u: print(f'[CRTP] 연결 해제: {u}'))
    cf.open_link(uri)

# ── HTTP 라우트 ───────────────────────────────────────────────────────────────
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/graph')
def api_graph():
    """초기 연결 시 현재 상태 스냅샷 제공"""
    from flask import jsonify
    now = time.time()
    with lock:
        graph_nodes = [
            {'id': nid, 'count': info['count'], 'seq': info['seq'],
             'hop': info['hop'],
             'x_m': round(info.get('x_m', 0.0), 2),
             'y_m': round(info.get('y_m', 0.0), 2),
             'z_m': round(info.get('z_m', 0.0), 2),
             'active': (now - info['last_seen']) < NODE_TIMEOUT_SEC,
             'age': round(now - info['last_seen'], 1)}
            for nid, info in nodes.items()
        ]
        graph_edges = [
            {'src': src, 'tx': tx, 'count': info['count'],
             'active': (now - info['last_seen']) < NODE_TIMEOUT_SEC}
            for (src, tx), info in edges.items()
        ]
        log = list(recent_log)
    return jsonify(nodes=graph_nodes, edges=graph_edges, log=log)

# ── SocketIO 이벤트 ───────────────────────────────────────────────────────────
@socketio.on('connect')
def on_connect():
    """클라이언트 연결 시 현재 그래프 상태 전송"""
    from flask import request
    now = time.time()
    with lock:
        graph_nodes = [
            {'id': nid, 'count': info['count'], 'seq': info['seq'],
             'hop': info['hop'],
             'x_m': round(info.get('x_m', 0.0), 2),
             'y_m': round(info.get('y_m', 0.0), 2),
             'z_m': round(info.get('z_m', 0.0), 2),
             'active': (now - info['last_seen']) < NODE_TIMEOUT_SEC,
             'age': round(now - info['last_seen'], 1)}
            for nid, info in nodes.items()
        ]
        graph_edges = [
            {'src': src, 'tx': tx, 'count': info['count'],
             'active': (now - info['last_seen']) < NODE_TIMEOUT_SEC}
            for (src, tx), info in edges.items()
        ]
        log = list(recent_log)
    socketio.emit('init', {'nodes': graph_nodes, 'edges': graph_edges, 'log': log},
                  to=request.sid)

if __name__ == '__main__':
    uri = sys.argv[1] if len(sys.argv) > 1 else 'radio://0/80/2M'
    print(f'Crazyflie URI: {uri}')
    connect(uri)
    socketio.run(app, host='0.0.0.0', port=1942, debug=False)
