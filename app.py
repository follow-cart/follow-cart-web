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
model = YOLO('./resource/yolov8n.pt')

# ROSBridge 연결 설정
ros = roslibpy.Ros(host='localhost', port=9090)
ros.run()

# Publisher 설정
cmd_topic = roslibpy.Topic(ros, '/cmd', 'std_msgs/String')

def move_car(direction):
    cmd = roslibpy.Message({
        'data': ''
    })

    if direction == 'forward':
        cmd['data'] = 'Forward'
    elif direction == 'backward':
        cmd['data'] = 'Backward'
    elif direction == 'left':
        cmd['data'] = 'Left Turn'
    elif direction == 'right':
        cmd['data'] = 'Right Turn'
    elif direction == 'stop':
        cmd['data'] = 'Stop'

    cmd_topic.publish(cmd)

# ROS2 노드 설정
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.frame = None

    def listener_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

# ROS2 초기화
rclpy.init()
image_subscriber = ImageSubscriber()
executor = rclpy.executors.SingleThreadedExecutor()
executor.add_node(image_subscriber)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/object_detection')
def object_detection():
    return render_template('object_detection.html')

@app.route('/move', methods=['POST'])
def move():
    direction = request.json.get('direction')
    if direction in ['forward', 'backward', 'left', 'right', 'stop']:
        move_car(direction)
        return jsonify({"status": "success", "message": f"Moving {direction}"}), 200
    else:
        return jsonify({"status": "error", "message": "Invalid direction"}), 400

def generate_video():
    while True:
        rclpy.spin_once(image_subscriber, timeout_sec=1.0)
        frame = image_subscriber.frame
        if frame is None:
            continue

        # 원하는 해상도로 리사이즈
        frame = cv2.resize(frame, (1024, 768))  # 원하는 해상도로 설정

        # 객체 인식 수행
        results = model(frame)

        # 결과를 프레임에 그리기
        for result in results:
            if result.boxes is not None:
                for box in result.boxes:
                    xyxy = box.xyxy.cpu().numpy()[0]  # 각 텐서 값을 numpy 배열로 변환
                    x1, y1, x2, y2 = map(int, xyxy)
                    conf = box.conf.item()  # 텐서 값을 추출
                    cls = int(box.cls.item())  # 텐서 값을 추출
                    label = f"{model.names[cls]} {conf:.2f}"

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 프레임을 JPEG로 인코딩
        ret, jpeg = cv2.imencode('.jpg', frame)
        if not ret:
            continue

        frame = jpeg.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_video(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)

