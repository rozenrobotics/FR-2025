#!/usr/bin/python3

# -*- coding: utf-8 -*-

import rospy
from clover import srv
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Empty, String
from mavros_msgs.srv import CommandBool, CommandLong

import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point32, Vector3Stamped, Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray

from typing import Tuple, Optional, List
import numpy as np

from copy import copy, deepcopy
from math import hypot
from abc import ABC, abstractmethod
from enum import Enum
import threading
import time


class Offboard(object):
    def __init__(self, flight_height) -> None:
        self.__flight_height = flight_height

        self.__get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.__navigate      = rospy.ServiceProxy('navigate', srv.Navigate)
        self.__land          = rospy.ServiceProxy('land', Trigger)
        self.__arming        = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

    def navigate_wait(self,
                      x=0, y=0, z=None,
                      yaw=float('nan'),
                      speed=2,
                      frame_id='body',
                      auto_arm=False,
                      tolerance=0.2) -> None:
    
        assert tolerance >= 0.05

        if z is None: z = self.__flight_height
        print(f'Flight to point x={round(x, 2)}; y={round(y, 2)}; z={round(z, 2)}...', end='')

        self.__navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
        while not rospy.is_shutdown():
            t = self.__get_telemetry(frame_id='navigate_target')
            if hypot(t.x, t.y, t.z) < tolerance:
                break
            
            rospy.sleep(0.2)
        print('Done')

    def navigate(self,
                 x=0, y=0, z=None,
                 yaw=float('nan'),
                 speed=2,
                 frame_id='body',
                 auto_arm=False):
        
        if z is None: z = self.__flight_height
        print(f'Flight to point x={round(x, 2)}; y={round(y, 2)}; z={round(z, 2)}. [non block]')
        self.__navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    def target_achieved(self, tolerance=0.2):
        assert tolerance >= 0.05

        t = self.__get_telemetry(frame_id='navigate_target')
        return hypot(t.x, t.y, t.z) <= tolerance

    def takeoff(self) -> None:
        print('Takeoff...', end='')
        self.navigate_wait(frame_id='body', speed=3.0, auto_arm=True)
        print('Done')

    def nonblock_takeoff(self) -> None:
        print('Takeoff. [non block]')
        self.navigate(frame_id='body', speed=3.0, auto_arm=True)

    def land(self) -> None:
        print('Landing...', end='')
        self.__land()
        print('Done')

    def disarm(self) -> None:
        print('Disarming...', end='')
        self.__arming(command=400, param2=21196)
        print('Done')

    def get_position(self, frame_id: str) -> None:
        return self.__get_telemetry(frame_id=frame_id)


class Planner(ABC):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def path(self):
        raise NotImplementedError()


class ZigzagPlanner(Planner):
    def __init__(self, area_x: float = 0, area_y: float = 0, bypass_number: int = 0) -> None:
        self.__width = area_x
        self.__height = area_y
        self.__n = bypass_number

    def path(self):
        step = self.__width / self.__n
        for i, x in enumerate(np.arange(0, self.__width + step, step)):
            yield (x, i%2 * self.__height)

            yield (x, (i+1)%2 * self.__height)


def camera_cfg_cvt(msg: CameraInfo) -> Tuple[np.ndarray, np.ndarray]:
    return (np.reshape(np.array(msg.K, dtype="float64"), (3, 3)), np.array(msg.D, dtype="float64"))


def unpack_vec(v):
    return np.array([v.vector.x, v.vector.y, v.vector.z])


n_plane = np.array([0, 0, 1])
p_plane = np.array([0, 0, 1])
def intersect_ray_plane(ray_v, ray_o) -> Optional[np.ndarray]:
    a = n_plane.dot(ray_v)
    if a == 0:
        return None
    
    d = (p_plane - ray_o).dot(n_plane) / a

    return ray_o + d * ray_v


def centroids_of_contours(contours) -> List[Tuple[float, float]]:
    centroids = []
    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] == 0:
            continue

        centroids.append([M["m10"] / (M["m00"]),
                          M["m01"] / (M["m00"])])
    return centroids


class PointsHandler(ABC):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def insert_point(self, x: float, y: float) -> None:
        raise NotImplementedError()

    @abstractmethod
    def retrieve_points(self) -> List[Tuple[float, float]]:
        raise NotImplementedError()


class ProbabilityMap(PointsHandler):
    # resolution [pix/m]
    def __init__(self, width: float = 1, height: float = 1, resolution: float = 5, debug: bool = False) -> None:
        self.__map = np.zeros(list(map(int, (width*resolution, height*resolution))), dtype=np.uint8)
        self.__resolution = resolution

    def insert_point(self, x: float, y: float, probability: float = 0.5) -> None:
        assert probability >= 0 and probability <= 1

        x = int(x * self.__resolution)
        y = int(y * self.__resolution)

        height, width = self.__map.shape
        if x < 0 or x >= width or y < 0 or y >= height:
            return

        self.__map[y, x] = int(probability * 255)

    def retrieve_points(self, threshold: np.uint8 = 127) -> List[Tuple[float, float]]:
        proc = np.where(self.__map >= threshold, 255, 0).astype(np.uint8)
        morphKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        proc = cv2.morphologyEx(proc, cv2.MORPH_DILATE, morphKernel, None, None, 4, cv2.BORDER_REFLECT101)
        contours, _ = cv2.findContours(proc, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return list(map(lambda x: [x[0]/self.__resolution, x[1]/self.__resolution], centroids_of_contours(contours)))


class VideoHandler(object):
    color_thresholds = {
        'red': (((0, 70, 70), (10, 255, 255)),
                ((168, 70, 70), (180, 255, 255))),
        
        'blue': (((100, 70, 70), (127, 255, 255)),),
        
        'green': (((44, 70, 70), (70, 255, 255)),),

        'yellow': (((19, 90, 100), (45, 255, 255)),),
    }

    color_codes = {
        'red': (1.0, 0.1, 0),
        'blue': (0.0, 0.0, 0.8),
        'green': (0.2, 1.0, 0.0),
        'yellow': (1.0, 0.9, 0.0)
    }

    building_types = {
        'red': 'Административное здание',
        'blue': "Здание обогащения угля",
        'green': "Лаборатория",
        'yellow': "Вход в шахту"
    }

    def __init__(self, objects_handler_instance: PointsHandler,
                       needed_objects_count: int = 5,
                       image_topic: str = "/main_camera/image_raw",
                       camera_info_topic: str = "/main_camera/camera_info",
                       camera_frame: str = "main_camera_optical",
                       debug: bool = False) -> None:
        self.objects = []

        self.__bridge = CvBridge()
        self.__tf_buffer = tf2_ros.Buffer()
        self.__listener = tf2_ros.TransformListener(self.__tf_buffer)

        self.__cm, self.__dc = camera_cfg_cvt(rospy.wait_for_message(camera_info_topic, CameraInfo))
        self.__camera_frame = camera_frame

        self.__image_topic = image_topic
        self.__image_subscriber = None

        self.__all_objects_finded = True
        self.__needed_objects_count = needed_objects_count
        self.__objects_handlers = dict()
        for color in self.color_thresholds:
            self.__objects_handlers[color] = deepcopy(objects_handler_instance)

        self.__mask_pub    = rospy.Publisher("/a/buildings_mask", Image, queue_size=1)  if debug else None
        self.__markers_pub = rospy.Publisher("/buildings", MarkerArray, queue_size=10) if debug else None
        if debug:
            rospy.Timer(rospy.Duration(0.25), self.__update_markers) # 10 Hz

    def enable(self) -> None:
        if self.__image_subscriber is None:
            self.__image_subscriber = rospy.Subscriber(self.__image_topic, Image, self.__image_cb, queue_size=1)
            print('Video handler enabled')

    def disable(self) -> None:
        if self.__image_subscriber is not None:
            self.__image_subscriber.unregister()
            self.__image_subscriber = None
            print('Video handler disabled')

    @property
    def all_objects_finded(self) -> bool:
        return self.__all_objects_finded

    def __image_cb(self, msg) -> None:
        try:
            frame = self.__bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return

        height, width, _ = frame.shape
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        masks = {}
        global_mask = np.zeros((height, width), dtype=np.uint8)
        for color, thresholds in self.color_thresholds.items():
            masks[color] = np.zeros((height, width), dtype=np.uint8)
            for (lower, upper) in thresholds: masks[color] |= cv2.inRange(hsv, lower, upper)
            global_mask |= masks[color]

        if self.__mask_pub: self.__mask_pub.publish(self.__bridge.cv2_to_imgmsg(cv2.bitwise_and(frame, frame, mask=global_mask), "bgr8"))

        for color, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            centroids = np.array(centroids_of_contours(filter(lambda x: cv2.contourArea(x) >= 1100, contours))).astype(np.float64)
            contours = list(filter(lambda x: hypot(x[0] - (width//2), x[1] - (height//2)) < 100, centroids))
            if len(centroids) == 0:
                continue
            
            centroids_undist = cv2.undistortPoints(centroids.reshape(-1, 1, 2), self.__cm, self.__dc, None, None).reshape(-1, 2).T
            ray_v = np.ones((3, centroids_undist.shape[1]))
            ray_v[:2, :] = centroids_undist
            ray_v /= np.linalg.norm(ray_v, axis=0)

            try:
                transform = self.__tf_buffer.lookup_transform("aruco_map", self.__camera_frame, rospy.Time())
            except tf2_ros.ConnectivityException:
                print("LookupException")
                continue
            except tf2_ros.LookupException:
                print("LookupException")
                continue

            t_wb = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
            ray_v = np.array([unpack_vec(tf2_geometry_msgs.do_transform_vector3(Vector3Stamped(vector=Vector3(v[0], v[1], v[2])), transform)) for v in ray_v.T])
            ray_o = t_wb

            centroids = [intersect_ray_plane(v, ray_o) for v in ray_v]
            [self.__objects_handlers[color].insert_point(*p[:2]) for p in centroids if p is not None]

    def __update_markers(self, evt) -> None:
        markers = []
        for color, handler in self.__objects_handlers.items():
            for (x, y) in handler.retrieve_points():
                marker = Marker()
                marker.header.frame_id = "aruco_map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "color_markers"
                marker.id = len(markers)
                marker.type =  Marker.CUBE
                marker.action = Marker.ADD

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.5
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 1.0

                r, g, b = self.color_codes[color]
                marker.color.a = 0.8
                marker.color.r = r
                marker.color.g = g
                marker.color.b = b

                marker.text = self.building_types[color]

                markers.append(marker)

        self.__markers_pub.publish(MarkerArray(markers=markers))
        self.__all_objects_finded = len(markers) >= self.__needed_objects_count


class IState(ABC):
    def __init__(self, machine):
        self.__machine = machine
        self.__next_state = None
        self.__initialised = False
        self.__post_function = None

    def next(self, next_state):
        self.__next_state = next_state
        return self

    def after_execution(self, func, *args, **kwargs):
        self.__post_function = func
        self.__post_args = args
        self.__post_kwargs = kwargs
        return self

    @abstractmethod
    def initialise(self):
        self.__initialised = True
        return self

    @abstractmethod
    def transition(self):
        if self.__post_function:
            self.__post_function(*self.__post_args, **self.__post_kwargs)
        
        if self.__next_state is None:
            return None
        
        if self.__next_state.initialised:
            return self.__next_state
        return self.__next_state.initialise()

    @property
    def machine(self):
        return self.__machine

    @property
    def initialised(self) -> bool:
        return self.__initialised

class Takeoff(IState):
    def __init__(self, machine):
        super().__init__(machine)
            
    def initialise(self):
        self.machine.offboard.nonblock_takeoff()
        return super().initialise()

    def transition(self):
        if self.machine.offboard.target_achieved():
            return super().transition()
        return self

class FlightToPoint(IState):
    def __init__(self, machine, x: float, y: float, z: float = None, speed: float = 0.8):
        super().__init__(machine)
        self.__x = x
        self.__y = y
        self.__z = z
            
    def initialise(self):
        self.machine.offboard.navigate(x=self.__x, y=self.__y, z=self.__z, frame_id='aruco_map', speed=1.0)
        return super().initialise()

    def transition(self):
        if self.machine.offboard.target_achieved():
            return super().transition()
        return self

class WaitFor(IState):
    def __init__(self, time: float = 0.0):
        super().__init__(None)
        self.__duration = rospy.Duration(time)
        self.__finish_time = None

    def initialise(self):
        print(f'Wait for {self.__duration.to_sec()} seconds. [non block]')
        self.__finish_time = rospy.Time.now() + self.__duration
        return super().initialise()

    def transition(self):
        if (self.__finish_time - rospy.Time.now()).to_sec() <= 0:
            return super().transition()
        return self

class MapBypass(IState):
    def __init__(self, machine):
        super().__init__(machine)

    def initialise(self):
        self.__iter = iter(ZigzagPlanner(*self.machine.flyzone_dimensions, bypass_number=9).path())
        return super().initialise()

    def transition(self):
        try:
            x, y = next(self.__iter)
        except StopIteration:
            return super().transition()

        return FlightToPoint(self.machine, x, y).next(WaitFor(1.0).next(self)).initialise()

class Land(IState):
    def __init__(self, machine):
        super().__init__(machine)

    def initialise(self):
        return super().initialise()

    def transition(self):
        self.machine.offboard.land()
        return super().transition()

class Disarm(IState):
    def __init__(self, machine):
        super().__init__(machine)

    def initialise(self):
        return super().initialise()

    def transition(self):
        self.machine.offboard.disarm()
        return super().transition()

class HoldCurrentPosition(IState):
    def __init__(self, machine):
        super().__init__(machine)

    def initialise(self):
        pos = self.machine.offboard.get_position('aruco_map')
        self.machine.offboard.navigate(x=pos.x, y=pos.y, z=pos.z, frame_id='aruco_map', speed=0.8)

    def transition(self):
        raise NotImplementedError()


class MissionStateMachine(object):
    class MissionStatus(Enum):
        NOTRUNNING = "NotRunning"
        RUNNING = "Running"
        DISARMING = "Disarming"
        PAUSED = "Paused"
        HOMING = "Homing"

    def __init__(self,
                 flight_height: float,
                 flyzone_dimensions: Tuple[float, float],
                 debug_mode: bool):
        self.flight_height = flight_height
        self.flyzone_dimensions = flyzone_dimensions
        self.offboard = Offboard(flight_height)
        self.video_handler = VideoHandler(ProbabilityMap(*flyzone_dimensions, 20, debug=debug_mode),
                                          debug=debug_mode)
        
        self.__shutdown_service = rospy.Service('/mission_shutdown', Trigger, self.__shutdown_response)
        self.__disarm_service = rospy.Service('/mission_disarm', Trigger, self.__disarm_response)
        self.__pause_service = rospy.Service('/mission_pause', Trigger, self.__pause_response)
        self.__status_publisher = rospy.Publisher('/mission_status', String, queue_size=1)

        self.__timer_returning = -1
        self.__mutex = threading.Lock()
        self.__returning_to_home = False
        self.__disarming = False
        self.__pause = False
        self.__status = self.MissionStatus.RUNNING.value
        self.__current_state = Takeoff(self).after_execution(lambda vh: vh.enable(), self.video_handler).next(
            MapBypass(self).after_execution(lambda vh: vh.disable(), self.video_handler).next(
            FlightToPoint(self, 0, 0).next(
            FlightToPoint(self, 0, 0, self.flight_height/2).next(
            Land(self)
        )))).initialise()

    def __shutdown_response(self, request):
        if self.__disarming:
            return TriggerResponse(
                success=False,
                message="Reason: disarming"
            )

        if self.__returning_to_home:
            return TriggerResponse(
                success=False,
                message="Reason: mission already shutdown"
            )

        self.__mutex.acquire()
        self.__current_state = FlightToPoint(self, 0, 0).next(
                               FlightToPoint(self, 0, 0, self.flight_height/2).next(
                               Land(self))).initialise()
        self.__returning_to_home = True
        self.__pause = False
        self.__status = self.MissionStatus.HOMING.value
        self.__mutex.release()

        return TriggerResponse(
            success=True,
            message="Shutdown mission..."
        )
    
    def __disarm_response(self, request):
        self.__mutex.acquire()
        self.__current_state = Disarm(self).initialise()
        self.__disarming = True
        self.__pause = False
        self.__status = self.MissionStatus.DISARMING.value
        self.__mutex.release()

        return TriggerResponse(
            success=True,
            message="Disarming clover..."
        )

    def __pause_response(self, request):
        if self.__disarming:
            return TriggerResponse(
                success=False,
                message="Reason: disarming"
            )

        if self.__returning_to_home:
            return TriggerResponse(
                success=False,
                message="Reason: returning to home position"
            )

        self.__mutex.acquire()
        if self.__pause:
            self.__current_state.initialise()
            self.__pause = False
            self.__status = self.MissionStatus.RUNNING.value
        else:
            HoldCurrentPosition(self).initialise()
            self.__pause = True
            self.__status = self.MissionStatus.PAUSED.value
        self.__mutex.release()
        return TriggerResponse(
            success=True,
            message="Pause mission..."
        )

    def process(self) -> None:
        self.__mutex.acquire()

        self.__status_publisher.publish(String(data=self.__status))

        if self.__current_state is None:
            self.__status = self.MissionStatus.NOTRUNNING.value
            self.__mutex.release()
            return True

        if self.__pause:
            self.__mutex.release()
            return False

        if self.video_handler.all_objects_finded and self.__timer_returning < 0:
            self.__timer_returning = time.time()

        if self.video_handler.all_objects_finded and self.__timer_returning > 0 and (time.time() - self.__timer_returning) >= 20 and not self.__returning_to_home:
            self.__returning_to_home = True
            self.__status = self.MissionStatus.HOMING.value
            self.__current_state = FlightToPoint(self, 0, 0).next(
                                   FlightToPoint(self, 0, 0, self.flight_height/2).next(
                                   Land(self))).initialise()

        self.__current_state = self.__current_state.transition()
        self.__mutex.release()
        return False




def main() -> None:
    rospy.init_node('offboard_node')

    FLYZONE_DIMENSIONS = [9, 9]
    FLIGHT_HEIGHT      = 2
    DEBUG_MODE         = True

    state_machine = MissionStateMachine(FLIGHT_HEIGHT,
                                        FLYZONE_DIMENSIONS,
                                        DEBUG_MODE)

    while not rospy.is_shutdown() and not state_machine.process():
        pass


if __name__ == '__main__':
    main()
