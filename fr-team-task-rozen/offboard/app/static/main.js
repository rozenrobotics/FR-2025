const canvasScale = 9.66;

var ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

var viewer = undefined;
var buildingsGraphics = [];

function resize() {
    var style = window.getComputedStyle(document.getElementById("aruco-map"), null);
    var size = Math.min(parseInt(style.getPropertyValue("height")), parseInt(style.getPropertyValue("width")));
    viewer.scene.canvas.width = viewer.scene.canvas.height = size;
    viewer.scene.scaleX = viewer.scene.scaleY = size / canvasScale;
    viewer.scene.x = viewer.scene.y = 0;
    viewer.scene.update();
}

function init() {
    viewer = new ROS2D.Viewer({
        divID: 'aruco-map',
        width: 966,
        height: 966,
        background: "#f9f9f9"
    });

    const missionStatusTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/mission_status',
        messageType: 'std_msgs/String'
    });
    
    missionStatusTopic.subscribe((message) => {
        // console.log('Received message on /mission_status:', message.data);
    
        // Обновляем HTML-элемент с данными
        const statusElement = document.getElementById('mission-status');
        if (statusElement) {
            statusElement.textContent = `Mission Status: ${message.data}`;
        }
    });

    const voltageTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/mavros/battery',
        messageType: 'sensor_msgs/BatteryState'
    });
    
    voltageTopic.subscribe((message) => {    
        const voltageElement = document.getElementById('voltage');
        if (voltageElement) {
            voltageElement.textContent = `Voltage: ${message.voltage.toFixed(2)}V`;
        }
    
        const percentageElement = document.getElementById('percentage');
        if (percentageElement) {
            percentageElement.textContent = `${(message.percentage * 100).toFixed(2)}%`;
        }
    });

    // // Подписка на топик с координатами
    // const positionTopic = new ROSLIB.Topic({
    //     ros: ros,
    //     name: '/drone_position',
    //     messageType: 'geometry_msgs/Point'
    // });

    // positionTopic.subscribe((message) => {
    //     const positionElement = document.getElementById('position');
    //     if (positionElement) {
    //         positionElement.textContent = `Position: X=${message.x}, Y=${message.y}, Z=${message.z}`;
    //     }
    // });
    

    var buildingsTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/buildings',
        messageType: 'visualization_msgs/MarkerArray'
    });

    buildingsTopic.subscribe(function (msg) {
        removeAllBuildingLabels();
        for (var graphic of buildingsGraphics) {
            viewer.scene.removeChild(graphic);
        }
        buildingsObjects = [];

        Object.entries(msg.markers).forEach(([index, marker]) => {
            var building = new Marker({
                strokeColor: createjs.Graphics.getRGB(marker.color.r * 255, marker.color.g * 255, marker.color.b * 255)
            });
            building.scaleX = marker.scale.x;
            building.scaleY = marker.scale.y;
            building.x = marker.pose.position.x + marker.scale.x / 2;
            building.y = 9.165 - (marker.pose.position.y);

            var number = new createjs.Text((marker.id + 1).toString(), "15px Arial", "#000000");
            number.x = marker.pose.position.x + marker.scale.x / 2 - 0.08;
            number.y = 9.165 - (marker.pose.position.y) - 0.08;
            number.scaleX = 0.05 * marker.scale.x;
            number.scaleY = 0.05 * marker.scale.y;

            buildingsGraphics.push(building, number);
            viewer.scene.addChild(building);
            viewer.scene.addChild(number);

            addBuildingLabel(marker.id.toString());
            renderBuilding(marker, marker.id.toString());
        })
    });

    var arucoMarkersTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/aruco_map/map',
        messageType: 'aruco_pose/MarkerArray'
    });

    arucoMarkersTopic.subscribe(function (msg) {
        Object.entries(msg.markers).forEach(([index, marker]) => {
            var aruco = new createjs.Bitmap(generateArucoMarker(marker.id));

            aruco.scaleX = 0.067 * marker.length;
            aruco.scaleY = 0.067 * marker.length;
            aruco.x = 1 + marker.pose.position.x + marker.length / 2 - 0.05;
            aruco.y = 7 - (marker.pose.position.y) - 0.05;

            viewer.scene.addChild(aruco);

            var xAxis = new CoordinateAxis({
                width: 0.1,
                height: 1.5,
                strokeColor: createjs.Graphics.getRGB(200, 10, 0)
            });
            var yAxis = new CoordinateAxis({
                width: 0.1,
                height: 1.5,
                strokeColor: createjs.Graphics.getRGB(10, 180, 0)
            });

            xAxis.x = yAxis.x = 1.15 - 0.01;
            xAxis.y = yAxis.y = 7.15 + 0.01;
            xAxis.rotation = -90;
            yAxis.rotation = 180;
            viewer.scene.addChild(xAxis);
            viewer.scene.addChild(yAxis);

            arucoMarkersTopic.unsubscribe();
        })
    });

    window.addEventListener('resize', resize, false);
    resize();
}


const MissionStatus = {
    NOTRUNNING: "NotRunning",
    RUNNING: "Running",
    DISARMING: "Disarming",
    PAUSED: "Paused",
    HOMING: "Homing",
    LANDING: "Landing"
}

setInterval(() => {
    const url = "/mission_status";
    fetch(url).then(function (response) {
        return response.json();
    }).then(function (mission) {
        document.getElementById('launch-btn').disabled = mission.status !== MissionStatus.NOTRUNNING;
        document.getElementById('shutdown-btn').disabled = mission.status !== MissionStatus.PAUSED && mission.status !== MissionStatus.RUNNING;
        document.getElementById('disarm-btn').disabled = mission.status === MissionStatus.NOTRUNNING || mission.status === MissionStatus.DISARMING;
        document.getElementById('pause-resume-btn').disabled = mission.status !== MissionStatus.RUNNING && mission.status !== MissionStatus.PAUSED;
        document.getElementById('land-btn').disabled = mission.status !== MissionStatus.PAUSED;


        document.getElementById('pause-resume-btn').innerHTML = mission.status === MissionStatus.PAUSED ? "Продолжить" : "Стоп";
        // document.getElementById('land-btn').innerHTML = mission.status === MissionStatus.PAUSED;

    });
},
    100
);

function sendCommand(cmd) {
    var req = new XMLHttpRequest();
    req.open('POST', "/command?name=" + cmd)
    req.send();
}