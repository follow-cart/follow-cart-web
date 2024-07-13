from flask import Flask, Response, render_template, request, jsonify
import roslibpy

# Flask 애플리케이션 설정
app = Flask(__name__, static_url_path='/static')

# ROSBridge 연결 설정
ros = roslibpy.Ros(host='localhost', port=9090)
ros.run()

# Publisher 설정
convoy_topic = roslibpy.Topic(ros, '/convoy_cmd', 'std_msgs/String')
pedestrian_topic = roslibpy.Topic(ros, '/pedestrian_cmd', 'std_msgs/String')

def move_pedestrian(direction):
    msg = roslibpy.Message({
        'data': ''
    })

    if direction == 'forward':
        msg['data'] = 'Forward'
    elif direction == 'backward':
        msg['data'] = 'Backward'
    elif direction == 'left':
        msg['data'] = 'Left Turn'
    elif direction == 'right':
        msg['data'] = 'Right Turn'
    elif direction == 'stop':
        msg['data'] = 'Stop'

    pedestrian_topic.publish(msg)

def execute_cmd(cmd):
    msg = roslibpy.Message({
        'data': ''
    })

    if cmd == 'call':
        msg['data'] = 'Call'
    elif cmd == 'return':
        msg['data'] = 'Return'

    convoy_topic.publish(msg)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/move', methods=['POST'])
def move():
    direction = request.json.get('direction')
    if direction in ['forward', 'backward', 'left', 'right', 'stop']:
        move_pedestrian(direction)
        return jsonify({"status": "success", "message": f"Moving {direction}"}), 200
    else:
        return jsonify({"status": "error", "message": "Invalid direction"}), 400


@app.route('/command', methods=['POST'])
def command():
    cmd = request.json.get('command')
    if cmd in ['call', 'return']:
        execute_cmd(cmd)
        return jsonify({"status": "success", "message": f"Executing {cmd}"}), 200
    else:
        return jsonify({"status": "error", "message": "Invalid command"}), 400


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)

