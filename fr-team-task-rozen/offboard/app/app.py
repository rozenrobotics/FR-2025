from flask import Flask, render_template, request
import os
import roslaunch
import rosnode
import rospy
from std_msgs.msg import String
import threading
from flask import jsonify
from std_srvs.srv import Trigger, TriggerRequest
from enum import Enum
from functools import partial
import subprocess
import time


app = Flask(__name__)

threading.Thread(target=lambda: rospy.init_node('server_node', disable_signals=True)).start()

status = 'NotRunning'
def mission_status_cb(msg):
    global status

    status = msg.data

status_sub = rospy.Subscriber("/mission_status", String, mission_status_cb)
shutdown_service = rospy.ServiceProxy('/mission_shutdown', Trigger)
disarm_service = rospy.ServiceProxy('/mission_disarm', Trigger)
pause_service = rospy.ServiceProxy('/mission_pause', Trigger)
land_service = rospy.ServiceProxy('/mission_land', Trigger)


last_launch_time = -1
launch_mutex = threading.Lock()
def launch_mission():
    global launch_mutex, last_launch_time

    launch_mutex.acquire()
    if last_launch_time > 0 and (time.time() - last_launch_time) <= 5:
        launch_mutex.release()
        return

    subprocess.Popen(["rosrun", "offboard", "main.py"])
    last_launch_time = time.time()
    launch_mutex.release()

class MissionCommands(Enum):
    Launch = partial(launch_mission)
    Stop   = partial(lambda: shutdown_service(TriggerRequest()))
    Pause  = partial(lambda: pause_service(TriggerRequest()))
    Disarm = partial(lambda: disarm_service(TriggerRequest()))
    Land   = partial(lambda: land_service(TriggerRequest()))


@app.route('/', methods=['GET'])
def root():
    return render_template('index.html')


@app.route('/mission_status', methods=['GET'])
def mission_status():
    global status

    response = {'status': None}
    if '/offboard_node' not in rosnode.get_node_names():
        response['status'] = status = 'NotRunning'
    else:
        response['status'] = status
    
    return jsonify(response)


@app.route('/command', methods=['POST'])
def command():
    name = request.args.get('name')
    for command in MissionCommands:
        if command.name.lower() == name:
            command.value()
            return "", 201

    return "", 404


if __name__ == '__main__':
    app.run(host="0.0.0.0", port=8000, debug=True)
