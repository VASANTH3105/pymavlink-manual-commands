from pymavlink import mavutil
import logging
from threading import Thread
from time import sleep
import time


class AUV:
    def __init__(self):
        logging.basicConfig(filename="movement.log",
                            format='%(asctime)s %(message)s',
                            filemode='w')
        self._logger = logging.getLogger()
        self._logger.setLevel(logging.DEBUG)
        self._master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        self._logger.info("Waiting for heartbeat...")
        self._master.wait_heartbeat()
        self._logger.info("Heartbeat from %s", self._master.target_system)
        self._master.arducopter_arm()
        self._logger.info("Waiting for the thrusters to arm...")
        self._master.motors_armed_wait()
        self._logger.info("Thrusters armed")
        self._forward_thread = Thread(target=self._movements, args=(5, 1600))
        self._backward_thread = Thread(target=self._movements, args=(5, 1400))
        self._left_thread = Thread(target=self._movements, args=(6, 1400))
        self._right_thread = Thread(target=self._movements, args=(6, 1600))

        self._thread_list = [
            self._forward_thread, self._backward_thread, self._left_thread, self._right_thread]
        self._stop = False

        self._heartbeat_requirement = mavutil.periodic_event(
            1)  # 1 Hz heartbeat
        self._pilot_input_requirement = mavutil.periodic_event(1/3)  # 0.33 Hz
        # self._buttons = 1 + 1 << 3 + 1 << 7
        self._logger.info("Initialization complete.")

    def _beat_heart_if_necessary(self):
        if self._heartbeat_requirement.trigger():
            self._logger.info("Sending heartbeat...")
            self._master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )

    def set_gain_max(self):
        self._logger.info("Setting gain to max...")
        self._master.param_set_send(
            self._master.target_system,
            self._master.target_component,
            "JS_GAIN_DEFAULT",
            1.0,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )

    def set_gain_mid(self):
        self._logger.info("Setting gain to mid...")
        self._master.param_set_send(
            self._master.target_system,
            self._master.target_component,
            "JS_GAIN_DEFAULT",
            0.5,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )

    def set_gain_min(self):
        self._logger.info("Setting gain to min...")
        self._master.param_set_send(
            self._master.target_system,
            self._master.target_component,
            "JS_GAIN_DEFAULT",
            0.2,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )

    # def _manual_control(self, x=0, y=0, z=500, yaw=0):
    #     self._master.mav.manual_control_send(
    #         self._master.target_system,
    #         x,
    #         y,
    #         z,  # 500 means neutral throttle
    #         yaw,
    #         self._buttons
    #     )

    def _set_rc_channel_pwm(self, channel_id, pwm):
        if channel_id < 1 or channel_id > 18:
            self._logger.error("Channel does not exist.")
            return

        rc_channel_values = [1500 for _ in range(8)]
        rc_channel_values[channel_id - 1] = pwm
        self._master.mav.rc_channels_override_send(
            self._master.target_system,
            self._master.target_component,
            *rc_channel_values)

    def stop_motion(self):
        self._logger.info("Stopping motion...")
        self._stop = True
        for thread in self._thread_list:
            if thread.is_alive():
                thread.join()
        # for thread in self._thread_list:
        #     thread.join()
        for channel in range(1, 7):
            self._set_rc_channel_pwm(channel, 1500)

    def _movements(self, channel_id, pwm):
        while True:
            if self._stop:
                self._stop = False
                break
            self._beat_heart_if_necessary()
            if self._pilot_input_requirement.trigger():
                self._set_rc_channel_pwm(channel_id, pwm)
                sleep(0.1)

    def forward(self):
        self._logger.info("Moving forward...")
        self.stop_motion()
        self._thread_list[0].start()

    def backward(self):
        self._logger.info("Moving backward...")
        self.stop_motion()
        self._thread_list[1].start()

    def left(self):
        self._logger.info("Moving left...")
        self.stop_motion()
        self._thread_list[2].start()

    def right(self):
        self._logger.info("Moving right...")
        self.stop_motion()
        self._thread_list[3].start()
'''
    def turn_cam_left(self):
        self._logger.info("Turning camera left...")
        # self._master.param_set_send(
        #     self._master.target_system,
        #     self._master.target_component,
        #     "CAM_PAN",
        #     -1,
        #     mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        # )
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            1 + 8,
            1100,
            0, 0, 0, 0, 0,
        )

    def turn_cam_right(self):
        self._logger.info("Turning camera left...")
        # self._master.param_set_send(
        #     self._master.target_system,
        #     self._master.target_component,
        #     "CAM_PAN",
        #     -1,
        #     mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        # )
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            1 + 8,
            1900,
            0, 0, 0, 0, 0,
        )

    def set_cam_center(self):
        self._logger.info("Setting camera to center...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            1 + 8,
            1400,
            0, 0, 0, 0, 0,
        )
'''
    def set_mode(self, mode):
        self._logger.info("Setting mode...")
        if mode not in self._master.mode_mapping():
            self._logger.error("Mode does not exist.")
            sys.exit(1)
        mode_id = self._master.mode_mapping()[mode]
        self._master.set_mode(mode_id)
        self._logger.info("Mode set to %s", mode)
'''
    def set_depth(self, depth):
        boot_time = time.time()
        self._logger.info("Setting target depth to %s", depth)
        self._master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - boot_time)),  # ms since boot
            self._master.target_system, self._master.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask=(
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            ), lat_int=0, lon_int=0, alt=depth,
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        )
        self._logger.info("Target depth set to %s", depth)
 '''

#main.py
'''
import cv2
# Always import cv2 at the top
import time
import commons as cm
import zed
from track_and_move import TrackObject
import shared_variables as sv
import auv


threshold = 0.2
top_k = 5
first_time = 1
auv = AUV()
tr = TrackObject(auv=auv, tolerance=0.02)
zed = Zed(640, 640)
servo_cam = cv2.VideoCapture(0)
bottom_cam = cv2.VideoCapture(1)

prev_command = None

detect_fn, labels = cm.load_model(sv.PATH_TO_MODEL, sv.PATH_TO_LABELS)


def main():
    arr_dur = [0, 0, 0]
    zed.start_capture_thread()

    global first_time, prev_command
    while True:

        if sv.USE_SERVO_CAM:
            frame = servo_cam.read()[1]
        elif sv.USE_ZED:
            frame, depth = zed.get_new_image()
        elif sv.USE_BOTTOM_CAM:
            frame = bottom_cam.read()[1]

        detections = cm.predictions(detect_fn, frame, (640, 640))

        objs, obj_lst = cm.get_output(
            detections=detections, score_threshold=threshold, top_k=top_k, labels=labels)

        if sv.MOVE_DOWN_FLAG:
            auv.set_depth(0.5)
            sv.MOVE_DOWN_FLAG = 0
        elif not sv.TASK1:
            print("Task 1")
            if not sv.FIND_GATE_FLAG:
                if 'gate' in obj_lst:
                    sv.START_TRACKING_FLAG = 1
                    sv.FIND_GATE_FLAG = 1
                    sv.USE_SERVO_CAM = 0
                    sv.USE_ZED = 1

                if 'gate' not in obj_lst and not sv.TURN_LEFT_FLAG:
                    auv.turn_cam_left()
                    sv.TURN_LEFT_FLAG = 1
                    continue
                elif 'gate' not in obj_lst and not sv.TURN_RIGHT_FLAG:
                    sv.TURN_LEFT_FLAG = 0
                    auv.turn_cam_right()
                    sv.TURN_RIGHT_FLAG = 1
                    continue

            elif sv.START_TRACKING_FLAG and sv.FIND_GATE_FLAG and not sv.STOP_TRACKING:
                if (sv.TURN_LEFT_FLAG or sv.TURN_RIGHT_FLAG) and not sv.CHANGE_CAMERA_DIRECTION_FLAG:
                    sv.CHANGE_CAMERA_DIRECTION_FLAG = 1
                    sv.TURN_LEFT_FLAG = 0
                    auv.set_cam_center()

                if sv.TURN_RIGHT_FLAG and not sv.CHANGE_CAMERA_DIRECTION_FLAG:
                    sv.CHANGE_CAMERA_DIRECTION_FLAG = 1
                    sv.TURN_RIGHT_FLAG = 0
                    auv.set_cam_center()
                tr.track_object(objs, labels, 'gate', obj_lst)

            elif sv.STOP_TRACKING:
                if 'red_flare' in obj_lst:
                    dist = cm.get_distance(objs, labels, 'red_flare')
                    print(dist)
                    if dist > .75:
                        cmd = "forward"
                        if prev_command != cmd:
                            auv.forward()
                            prev_command = cmd
                    elif dist <= .75:
                        cmd = "stop"
                        if prev_command != cmd:
                            auv.left()
                            time.sleep(1)
                            auv.forward()
                            time.sleep(2)
                            auv.right()
                            time.sleep(1)
                            auv.stop_motion()
                            auv.set_depth(0.7)
                            prev_command = cmd
                elif 'gate' in obj_lst:
                    cmd = "forward"
                    if prev_command != cmd:
                        auv.forward()
                        prev_command = cmd
                elif 'gate' not in obj_lst:
                    auv.stop_motion()
                    prev_command = None
                    sv.TASK1 = 1
                    sv.TURN_RIGHT_FLAG = 0
                    sv.TURN_LEFT_FLAG = 0
                    sv.START_TRACKING_FLAG = 0
                    sv.CHANGE_CAMERA_DIRECTION_FLAG = 0
                    sv.STOP_TRACKING = 0
                    sv.USE_ZED = 0
                    sv.USE_SERVO_CAM = 1

        elif not sv.TASK2_PART1:
            print("Task 2 I")
            if first_time:
                auv.forward()
                time.sleep(3)
                auv.stop_motion()
                first_time = 0

            if not sv.FIND_TUB_FLAG:
                if 'tub' in obj_lst:
                    sv.START_TRACKING_FLAG = 1
                    sv.FIND_TUB_FLAG = 1
                    sv.USE_SERVO_CAM = 0
                    sv.USE_ZED = 1
                elif 'tub' not in obj_lst and not sv.TURN_LEFT_FLAG:
                    auv.turn_cam_left()
                    sv.TURN_LEFT_FLAG = 1
                    continue

            elif sv.START_TRACKING_FLAG and sv.FIND_TUB_FLAG and not sv.STOP_TRACKING:
                if sv.TURN_LEFT_FLAG and not sv.CHANGE_CAMERA_DIRECTION_FLAG:
                    sv.CHANGE_CAMERA_DIRECTION_FLAG = 1
                    sv.TURN_LEFT_FLAG = 0
                    auv.set_cam_center()
                tr.track_object(objs, labels, 'tub', obj_lst)

            elif sv.STOP_TRACKING:
                if 'tub' in obj_lst:
                    dist = cm.get_distance(objs, labels, 'tub')
                    print(dist)
                    if dist > .07:
                        cmd = "forward"
                        if prev_command != cmd:
                            auv.forward()
                            prev_command = cmd
                    elif dist <= .07:
                        auv.stop_motion()
                        sv.TASK2_PART1 = 1
                        sv.TURN_RIGHT_FLAG = 0
                        sv.TURN_LEFT_FLAG = 0
                        sv.START_TRACKING_FLAG = 0
                        sv.CHANGE_CAMERA_DIRECTION_FLAG = 0
                        sv.STOP_TRACKING = 0
                        first_time = 1
                        sv.USE_ZED = 1
                        sv.USE_SERVO_CAM = 0

        elif not sv.TASK2_PART2:
            print("Task 2 II")
            tr.track_object(objs, labels, 'blue_tub', obj_lst)

            if sv.STOP_TRACKING:
                sv.TASK2_PART2 = 1
                sv.STOP_TRACKING = 0
                sv.USE_ZED = 0
                sv.USE_SERVO_CAM = 0
                sv.USE_BOTTOM_CAM = 11
                prev_command = None

        elif not sv.TASK2_PART3:
            print("Task 2 III")
            if first_time:
                sv.STOP_TRACKING = 0
                auv.stop_motion()
                first_time = 0
                obj_lst = []

            print(obj_lst)

            if 'blue_tub' in obj_lst:
                sv.START_TRACKING_FLAG = 1

            if 'blue_tub' not in obj_lst:
                cmd = "forward"
                if prev_command != cmd:
                    auv.forward()
                continue

            if sv.START_TRACKING_FLAG and not sv.STOP_TRACKING:
                if 'blue_tub' not in obj_lst:
                    auv.backward()
                    time.sleep(1.5)
                    auv.stop_motion()
                tr.track_object(objs,
                                labels, 'blue_tub', obj_lst, "task2_part3")
            if sv.STOP_TRACKING:
                sv.TASK2_PART3 = 1
                sv.STOP_TRACKING = 0
                auv.stop_motion()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # frame = tr.draw_overlays(frame, objs, labels, arr_dur)
        cv2.imshow('Object Tracking', cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    cv2.destroyAllWindows()


main()
'''
