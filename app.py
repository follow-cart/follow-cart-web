from flask import Flask, Response, render_template, request, jsonify
import cv2
import numpy as np
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import roslibpy

# Flask 애플리케이션 설정
app = Flask(__name__, static_url_path='/static')

# YOLOv8 모델 로드
# model = YOLO('./resource/yolov8n.pt')
#
# # ROSBridge 연결 설정
# ros = roslibpy.Ros(host='localhost', port=9090)
# ros.run()
#
# # Publisher 설정
# convoy_topic = roslibpy.Topic(ros, '/convoy_cmd', 'std_msgs/String')
# pedestrian_topic = roslibpy.Topic(ros, '/pedestrian_cmd', 'std_msgs/String')
#
# def move_pedestrian(direction):
#     cmd = roslibpy.Message({
#         'data': ''
#     })
#
#     if direction == 'forward':
#         cmd['data'] = 'Forward'
#     elif direction == 'backward':
#         cmd['data'] = 'Backward'
#     elif direction == 'left':
#         cmd['data'] = 'Left Turn'
#     elif direction == 'right':
#         cmd['data'] = 'Right Turn'
#     elif direction == 'stop':
#         cmd['data'] = 'Stop'
#
#     pedestrian_topic.publish(cmd)
#
# def execute_cmd(cmd):
#     cmd = roslibpy.Message({
#         'data': ''
#     })
#
#     if cmd == 'call':
#         cmd['data'] = 'Call'
#     elif cmd == 'return':
#         cmd['data'] = 'Return'
#
#     pedestrian_topic.publish(cmd)


@app.route('/')
def index():
    return render_template('index.html')


# @app.route('/move', methods=['POST'])
# def move():
#     direction = request.json.get('direction')
#     if direction in ['forward', 'backward', 'left', 'right', 'stop']:
#         move_pedestrian(direction)
#         return jsonify({"status": "success", "message": f"Moving {direction}"}), 200
#     else:
#         return jsonify({"status": "error", "message": "Invalid direction"}), 400
#
#
# @app.route('/command', methods=['POST'])
# def command():
#     cmd = request.json.get('cmd')
#     if cmd in ['call', 'return']:
#         execute_cmd(cmd)
#         return jsonify({"status": "success", "message": f"Executing {cmd}"}), 200
#     else:
#         return jsonify({"status": "error", "message": "Invalid command"}), 400


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)

