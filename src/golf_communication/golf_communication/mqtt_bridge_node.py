"""P2-7 MQTT 4G 通信桥接节点

桥接 ROS2 话题 ↔ MQTT，通过 4G 网络连接手机 APP。
话题带 vehicle_id 前缀（默认 cart_01）。

车 → APP（ROS2 订阅 → MQTT 发布）：
  /system_mode      → golf/{vehicle_id}/status/mode
  /gps/fix          → golf/{vehicle_id}/status/gps
  /path_graph/stats → golf/{vehicle_id}/status/path_stats
  /follow_state     → golf/{vehicle_id}/status/follow
  /summon_result    → golf/{vehicle_id}/status/summon_result  (qos=1)
  /nav_complete     → golf/{vehicle_id}/status/nav_complete   (qos=1)

APP → 车（MQTT 订阅 → ROS2 发布）：
  golf/{vehicle_id}/cmd/summon → /summon_request（JSON: {target, lat, lon}）
  golf/{vehicle_id}/cmd/mark   → /mark_waypoint_label
  golf/{vehicle_id}/cmd/mode   → /nav_trigger

线程安全：paho-mqtt 后台线程的回调通过 queue 传递到 ROS2 定时器消费，
避免跨线程直接调用 ROS2 API。
"""

import json
import os
import queue
import threading
import uuid

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None


class MqttBridgeNode(Node):

    def __init__(self):
        super().__init__('mqtt_bridge')

        if mqtt is None:
            self.get_logger().fatal(
                'paho-mqtt not installed. Run: pip3 install paho-mqtt')
            raise RuntimeError('paho-mqtt not installed')

        # 参数
        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_keepalive', 60)
        self.declare_parameter('mqtt_username', '')
        self.declare_parameter('mqtt_password', '')
        self.declare_parameter('vehicle_id', 'cart_01')

        host = self.get_parameter('mqtt_host').value
        port = self.get_parameter('mqtt_port').value
        keepalive = self.get_parameter('mqtt_keepalive').value
        username = self.get_parameter('mqtt_username').value
        # 优先从环境变量读取密码
        password = os.environ.get(
            'MQTT_PASSWORD', self.get_parameter('mqtt_password').value)
        self.vehicle_id = self.get_parameter('vehicle_id').value

        # paho 线程 → ROS2 线程的消息队列（线程安全）
        self._msg_queue = queue.Queue()
        self._connected = threading.Event()

        # MQTT 客户端（client_id 加随机后缀防重连风暴）
        client_id = f'golf_{self.vehicle_id}_{uuid.uuid4().hex[:6]}'
        # 兼容 paho-mqtt 1.x 和 2.x
        if hasattr(mqtt, 'CallbackAPIVersion'):
            self.mqtt_client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
                client_id=client_id,
                protocol=mqtt.MQTTv311)
        else:
            self.mqtt_client = mqtt.Client(
                client_id=client_id,
                protocol=mqtt.MQTTv311)

        if username:
            self.mqtt_client.username_pw_set(username, password)

        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message

        # MQTT 连接（异步循环，不阻塞 ROS2）
        try:
            self.mqtt_client.connect_async(host, port, keepalive)
            self.mqtt_client.loop_start()
            self.get_logger().info(f'MQTT connecting to {host}:{port}...')
        except Exception as e:
            self.get_logger().error(f'MQTT connect failed: {e}')

        # ── ROS2 → MQTT（车 → APP）──

        self.create_subscription(
            String, '/system_mode', self._mode_to_mqtt, 10)
        self.create_subscription(
            String, '/follow_state', self._follow_to_mqtt, 10)
        self.create_subscription(
            String, '/path_graph/stats', self._stats_to_mqtt, 10)

        gps_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            NavSatFix, '/gps/fix', self._gps_to_mqtt, gps_qos)

        # 召唤结果 → APP
        self.create_subscription(
            String, '/summon_result', self._summon_result_to_mqtt, 10)
        # 导航完成 → APP
        self.create_subscription(
            String, '/nav_complete', self._nav_complete_to_mqtt, 10)

        # ── MQTT → ROS2（APP → 车）──

        self.summon_pub = self.create_publisher(String, '/summon_request', 10)
        self.mark_pub = self.create_publisher(String, '/mark_waypoint_label', 10)
        self.nav_trigger_pub = self.create_publisher(String, '/nav_trigger', 10)

        # 定时器：从队列消费 paho 消息 → ROS2 publish（线程安全）
        self.create_timer(0.05, self._drain_queue)
        # 心跳：定时发送车辆在线状态
        self.create_timer(5.0, self._heartbeat)

        self.get_logger().info('MQTT bridge node started')

    # ── MQTT 回调（在 paho 线程中，不直接调用 ROS2 API）──

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._connected.set()
            # 订阅 APP → 车 的命令话题
            prefix = f'golf/{self.vehicle_id}/cmd'
            client.subscribe(f'{prefix}/summon')
            client.subscribe(f'{prefix}/mark')
            client.subscribe(f'{prefix}/mode')
            # 日志通过队列转到 ROS2 线程
            self._msg_queue.put(('_log', 'info', f'MQTT connected, subscribed to {prefix}/*'))
        else:
            self._msg_queue.put(('_log', 'error', f'MQTT connect failed, rc={rc}'))

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self._connected.clear()
        if rc != 0:
            self._msg_queue.put(('_log', 'warning', f'MQTT disconnected unexpectedly, rc={rc}'))

    def _on_mqtt_message(self, client, userdata, msg):
        """放入队列，由 ROS2 定时器消费"""
        payload = msg.payload.decode('utf-8', errors='replace')
        self._msg_queue.put(('msg', msg.topic, payload))

    # ── 队列消费（ROS2 线程，安全调用 publish/logger）──

    def _drain_queue(self):
        while not self._msg_queue.empty():
            try:
                item = self._msg_queue.get_nowait()
            except queue.Empty:
                break

            if item[0] == '_log':
                _, level, text = item
                # ROS2 Humble rcutils_logger bug: severity切换会抛ValueError
                # 统一用 info 级别输出，前缀标注原级别
                try:
                    if level in ('warning', 'error', 'fatal'):
                        self.get_logger().info(f'[{level.upper()}] {text}')
                    else:
                        self.get_logger().info(text)
                except ValueError:
                    pass  # logger severity冲突，静默跳过
            elif item[0] == 'msg':
                _, topic, payload = item
                self._dispatch_mqtt(topic, payload)

    def _dispatch_mqtt(self, topic, payload):
        """解析 MQTT 命令 → ROS2 发布"""
        prefix = f'golf/{self.vehicle_id}/cmd/'
        if not topic.startswith(prefix):
            return

        cmd = topic[len(prefix):]
        ros_msg = String()
        ros_msg.data = payload

        if cmd == 'summon':
            self.summon_pub.publish(ros_msg)
            self.get_logger().info(f'Summon request: {payload}')
        elif cmd == 'mark':
            self.mark_pub.publish(ros_msg)
            self.get_logger().info(f'Mark waypoint: {payload}')
        elif cmd == 'mode':
            self.nav_trigger_pub.publish(ros_msg)
            self.get_logger().info(f'Mode switch: {payload}')

    # ── ROS2 → MQTT 转发 ──

    def _mqtt_publish(self, subtopic, payload, qos=0):
        if not self._connected.is_set():
            return
        topic = f'golf/{self.vehicle_id}/status/{subtopic}'
        self.mqtt_client.publish(topic, payload, qos=qos)

    def _mode_to_mqtt(self, msg: String):
        self._mqtt_publish('mode', msg.data)

    def _follow_to_mqtt(self, msg: String):
        self._mqtt_publish('follow', msg.data)

    def _stats_to_mqtt(self, msg: String):
        self._mqtt_publish('path_stats', msg.data)

    def _summon_result_to_mqtt(self, msg: String):
        self._mqtt_publish('summon_result', msg.data, qos=1)

    def _nav_complete_to_mqtt(self, msg: String):
        self._mqtt_publish('nav_complete', msg.data, qos=1)

    def _gps_to_mqtt(self, msg: NavSatFix):
        gps_data = json.dumps({
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'status': msg.status.status,
        })
        self._mqtt_publish('gps', gps_data)

    def _heartbeat(self):
        if not self._connected.is_set():
            return
        self._mqtt_publish('heartbeat', 'online')

    def destroy_node(self):
        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
