





# -*- coding: utf-8 -*-

import subprocess
import os
import signal
import time
from pathlib import Path
from threading import Thread, Lock
from flask import Flask, render_template, jsonify, request, Response
from flask_socketio import SocketIO

app = Flask(__name__, template_folder='templates')
app.config['SECRET_KEY'] = 'ros2_web_manager_secret'
socketio = SocketIO(app, async_mode='threading')

ros_env = os.environ.copy()
status_lock = Lock()


@app.route('/api/setup_workspace', methods=['POST'])
def setup_workspace():
    try:
        subprocess.run(['colcon', 'build'], cwd=os.getenv('ROS_WORKSPACE', '~/spot_micro_ws'), check=True)
        return jsonify({'status': 'success'}), 200
    except subprocess.CalledProcessError as e:
        return jsonify({'status': 'error', 'details': str(e)}), 500

@socketio.on('subscribe_logs')
def handle_subscribe_logs(data):
    pkg, node = data['package'], data['node']
    log_path = f"{os.getenv('ROS_WORKSPACE', '~/spot_micro_ws')}/log/latest_build/{pkg}/log/latest_build/{node}.log"
    
    def stream_logs():
        try:
            with open(log_path, 'r', encoding='utf-8') as f:
                f.seek(0, os.SEEK_END)
                while True:
                    line = f.readline()
                    if not line:
                        socketio.sleep(0.5)
                        continue
                    socketio.emit('node_log', {'package': pkg, 'node': node, 'line': line})
        except Exception as e:
            socketio.emit('node_log', {'package': pkg, 'node': node, 'line': f"Error: {str(e)}"})

    socketio.start_background_task(stream_logs)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/build', methods=['POST'])
def build_workspace():
    try:
        ws_path = find_workspace_root()
        result = subprocess.run(
            ['colcon', 'build', '--symlink-install'],
            check=False,
            cwd=str(ws_path),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=ros_env
        )
        return jsonify({'status': 'built', 'output': result.stdout})
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/launch', methods=['POST'])
def launch_node():
    data = request.get_json()
    pkg = data.get('package')
    node = data.get('node')
    if not pkg or not node:
        return jsonify({'error': 'Missing package or node'}), 400

    try:
        ws_root = find_workspace_root()
        env = ros_env.copy()
        ros_setup = f"source /opt/ros/{env.get('ROS_DISTRO', 'jazzy')}/setup.bash && source {ws_root}/install/setup.bash"
        launch_cmd = f"{ros_setup} && ros2 run {pkg} {node}"
        log_path = Path(f"/tmp/{pkg}_{node}.log")
        proc = subprocess.Popen(
            ['bash', '-c', launch_cmd],
            start_new_session=True,
            stdout=open(log_path, 'w', encoding='utf-8'),
            stderr=subprocess.STDOUT,
            env=env
        )
        pid_file = Path.home() / f".web_launch_{pkg}_{node}.pid"
        pid_file.write_text(str(proc.pid))
        return jsonify({'status': 'launched', 'pid': proc.pid})
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/stop', methods=['POST'])
def stop_node():
    data = request.get_json()
    node_name = data.get('node')
    if not node_name:
        return jsonify({'error': 'Missing node name'}), 400

    try:
        # Try to stop using PID file
        pid_files = list(Path.home().glob(f".web_launch_*_{node_name}.pid"))
        for pf in pid_files:
            try:
                pid = int(pf.read_text())
                os.killpg(pid, signal.SIGTERM)
                time.sleep(0.1)
                pf.unlink()
                return jsonify({'status': f"Node {node_name} stopped via PID"})
            except (ValueError, FileNotFoundError, ProcessLookupError):
                pf.unlink()

        # Fallback to ros2 node info
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=ros_env
        )
        if result.returncode == 0:
            for full_node_name in result.stdout.strip().splitlines():
                if not full_node_name.strip():
                    continue
                if full_node_name.endswith(f"/{node_name}") or full_node_name == node_name:
                    info = subprocess.run(
                        ['ros2', 'node', 'info', full_node_name],
                        check=False,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        env=ros_env
                    )
                    if info.returncode == 0:
                        for line in info.stdout.splitlines():
                            if 'PID:' in line:
                                try:
                                    pid = int(line.split(':')[1].strip())
                                    os.kill(pid, signal.SIGTERM)
                                    time.sleep(0.1)
                                    os.kill(pid, 0)
                                    os.kill(pid, signal.SIGKILL)
                                    return jsonify({'status': f"Node {node_name} stopped via signal"})
                                except Exception:
                                    pass
        return jsonify({'error': f"Node {node_name} not found"}), 404
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/stop_all', methods=['POST'])
def stop_all_nodes():
    try:
        pid_files = list(Path.home().glob('.web_launch_*.pid'))
        killed = []
        for pf in pid_files:
            try:
                pid = int(pf.read_text())
                os.killpg(pid, signal.SIGTERM)
                time.sleep(0.1)
                pf.unlink()
                node_name = pf.stem.split('_')[-2]
                killed.append(node_name)
            except (ValueError, FileNotFoundError, ProcessLookupError):
                pf.unlink()

        result = subprocess.run(
            ['ros2', 'node', 'list'],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=ros_env
        )
        if result.returncode == 0:
            for full_node_name in result.stdout.strip().splitlines():
                if not full_node_name.strip():
                    continue
                info = subprocess.run(
                    ['ros2', 'node', 'info', full_node_name],
                    check=False,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    env=ros_env
                )
                if info.returncode == 0:
                    for line in info.stdout.splitlines():
                        if 'PID:' in line:
                            try:
                                pid = int(line.split(':')[1].strip())
                                os.kill(pid, signal.SIGTERM)
                                time.sleep(0.1)
                                os.kill(pid, 0)
                                os.kill(pid, signal.SIGKILL)
                            except Exception:
                                pass
        return jsonify({'status': 'Nodes stopped', 'killed': killed})
    except Exception as e:
        return jsonify({'error': str(e)}), 500
@app.route('/api/logs/<pkg>/<node>')
def stream_logs(pkg, node):
    log_path = Path(f"/tmp/{pkg}_{node}.log")
    def generate():
        with open(log_path, 'r', encoding='utf-8') as f:
            f.seek(0, os.SEEK_END)
            while True:
                line = f.readline()
                if line:
                    yield line
                else:
                    time.sleep(0.3)
    return Response(generate(), mimetype='text/plain')

@app.route('/api/status', methods=['GET'])
def get_status():
    return jsonify(generate_status())

@app.route('/api/create_node', methods=['POST'])
def create_node():
    data = request.get_json()
    pkg = data.get('package')
    node = data.get('name')
    node_type = data.get('type', 'generic')
    if not pkg or not node:
        return jsonify({'error': 'Missing package or node name'}), 400

    try:
        ws_path = find_workspace_root()
        node_path = ws_path / 'src' / pkg / pkg / f'{node}.py'
        node_path.parent.mkdir(parents=True, exist_ok=True)

        if node_type == 'publisher':
            content = f"""#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from std_msgs.msg import String

class {node.capitalize()}Node(LifecycleNode):
    def __init__(self):
        super().__init__('{node}')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.publish_msg)

    def publish_msg(self):
        msg = String()
        msg.data = 'testing publisher'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "' + msg.data + '"')

def main():
    rclpy.init()
    rclpy.spin({node.capitalize()}Node())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
        elif node_type == 'subscriber':
            content = f"""#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from std_msgs.msg import String

class {node.capitalize()}Node(LifecycleNode):
    def __init__(self):
        super().__init__('{node}')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('Received: "' + msg.data + '"')

def main():
    rclpy.init()
    rclpy.spin({node.capitalize()}Node())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
        else:
            content = f"""#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode

class {node.capitalize()}Node(LifecycleNode):
    def __init__(self):
        super().__init__('{node}')
        self.get_logger().info('{node} lifecycle node started')

def main():
    rclpy.init()
    rclpy.spin({node.capitalize()}Node())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""

        node_path.write_text(content)
        node_path.chmod(0o755)

        setup_path = ws_path / 'src' / pkg / 'setup.py'
        if setup_path.exists():
            text = setup_path.read_text()
            lines = text.splitlines()
            new_lines = []
            in_entry = False
            inserted = False
            for line in lines:
                new_lines.append(line)
                if "'console_scripts': [" in line:
                    in_entry = True
                elif in_entry and '],' in line and not inserted:
                    new_lines.insert(-1, f"        '{node} = {pkg}.{node}:main',")
                    inserted = True
            setup_path.write_text('\n'.join(new_lines))

        return jsonify({'status': 'node created'})
    except Exception as e:
        return jsonify({'error': str(e)}), 500




# pylint: disable-next=pointless-string-statement
'''def detect_node_type(node_script_path):
    """Try to infer node type from keywords in source."""
    try:
        text = Path(node_script_path).read_text()
        if 'create_subscription' in text:
            return 'subscriber'
        elif 'create_publisher' in text:
            return 'publisher'
        elif 'broadcast' in text:
            return 'tf broadcaster'
        elif 'discovery' in text:
            return 'discovery'
    except Exception:
        pass
    return 'unknown'
    '''

def check_lifecycle(node_name):
    """Return True if node responds to lifecycle services"""
    try:
        result = subprocess.run(
            ['ros2', 'lifecycle', 'get', node_name],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=ros_env,
            timeout=2
        )
        return 'unknown' not in result.stdout.lower()
    except Exception:
        return False

@app.route('/api/delete_node', methods=['POST'])
def delete_node():
    data = request.get_json()
    pkg = data.get('package')
    node = data.get('node')
    if not pkg or not node:
        return jsonify({'error': 'Missing package or node'}), 400

    try:
        ws_path = find_workspace_root()
        node_file = ws_path / 'src' / pkg / pkg / f'{node}.py'
        setup_path = ws_path / 'src' / pkg / 'setup.py'

        if node_file.exists():
            node_file.unlink()

        if setup_path.exists():
            text = setup_path.read_text()
            if node in text:
                new_text = []
                for line in text.splitlines():
                    if node not in line:
                        new_text.append(line)
                setup_path.write_text('\n'.join(new_text))

        return jsonify({'status': f'{node} deleted'})
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
def get_running_nodes():
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=ros_env
        )
        if result.returncode == 0:
            return [line.strip().split('/')[-1] for line in result.stdout.splitlines() if line.strip()]
    except Exception as e:
        print(f"Error listing nodes: {e}")
    return []

def find_workspace_root():
    current_path = Path(__file__).parent
    while current_path != current_path.root:
        if (current_path / 'src').exists():
            return current_path
        current_path = current_path.parent
    return Path.home() / 'spot_micro_ws'

def list_local_packages():
    packages = set()
    ws_path = find_workspace_root()
    src_path = ws_path / 'src'
    for pkg in src_path.iterdir():
        if (pkg / 'package.xml').exists():
            packages.add(pkg.name)
    return sorted(packages)

def get_package_executables(pkg_name):
    executables = set()
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'executables', pkg_name],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=ros_env,
            timeout=3
        )
        if result.returncode == 0:
            for line in result.stdout.splitlines():
                parts = line.strip().split()
                if len(parts) == 2:
                    executables.add(parts[1])
    except Exception as e:
        print(f"ros2 pkg executables failed for {pkg_name}: {e}")

    try:
        ws = find_workspace_root()
        setup_path = ws / 'src' / pkg_name / 'setup.py'
        if setup_path.exists():
            text = setup_path.read_text()
            if "'console_scripts': [" in text:
                section = text.split("'console_scripts': [", 1)[1].split("]", 1)[0]
                import re
                entries = re.findall(r"'([^=']+)\s*=", section)
                executables.update(e.strip() for e in entries)
    except Exception as e:
        print(f"setup.py parse failed for {pkg_name}: {e}")

    try:
        lib_path = find_workspace_root() / 'install' / 'lib' / pkg_name
        if lib_path.exists():
            for f in lib_path.iterdir():
                if f.is_file() and os.access(f, os.X_OK):
                    executables.add(f.name)
    except Exception as e:
        print(f"lib scan failed for {pkg_name}: {e}")

    return sorted(executables)

def detect_node_type(script_path):
    """Heuristically determine if the node is a publisher, subscriber, etc."""
    if not script_path.exists():
        return "unknown"
    try:
        content = script_path.read_text().lower()
        if "create_subscription" in content:
            return "subscriber"
        elif "create_publisher" in content:
            return "publisher"
        elif "timer_callback" in content:
            return "timer"
        elif "service" in content:
            return "service"
        elif "client" in content:
            return "client"
        elif "discovery" in content or "scan" in content:
            return "discovery"
        else:
            return "generic"
    except Exception:
        return "unknown"


def generate_status():
    running = get_running_nodes()
    known = set()
    packages = []
    for pkg in list_local_packages():
        nodes = []
        for exe in get_package_executables(pkg):
            name = exe.split('/')[-1]
            known.add(name)

            # Guess node path (optional)
            node_script = find_workspace_root() / 'src' / pkg / pkg / f'{name}.py'
            node_type = detect_node_type(node_script)
            is_lifecycle = check_lifecycle(name)

            nodes.append({
                'name': name,
                'status': 'running' if name in running else 'stopped',
                'type': node_type,
                'lifecycle': is_lifecycle
            })

        packages.append({'name': pkg, 'nodes': nodes})
    external = [{'name': n, 'status': 'running', 'type': 'external'} for n in running if n not in known]
    if external:
        packages.append({'name': '__external__', 'nodes': external})
    return {'packages': packages}

def status_loop():
    while True:
        try:
            socketio.emit('status_update', generate_status())
        except Exception as e:
            print(f"Status loop error: {e}")
        time.sleep(0.3)

if __name__ == '__main__':
    ros_env.update(os.environ)
    Thread(target=status_loop, daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)