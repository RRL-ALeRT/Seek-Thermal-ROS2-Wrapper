import rclpy
from rclpy.node import Node

from seekcamera import (
    SeekCameraIOType,
    SeekCameraColorPalette,
    SeekCameraManager,
    SeekCameraManagerEvent,
    SeekCameraFrameFormat,
    SeekCamera,
    SeekFrame,
)

from sensor_msgs.msg import Image
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange
from cv_bridge import CvBridge
import cv2
from time import sleep


class ThermalImagePublisher(Node):
    def __init__(self):
        super().__init__('thermalImagePublisher')

        self.description_Color = "colorPalette has the following options: \n\
                            WHITE_HOT \n\
                            BLACK_HOT \n\
                            SPECTRA \n\
                            PRISM \n\
                            TYRIAN -- Default \n\
                            IRON \n\
                            AMBER \n\
                            HI \n\
                            GREEN \n"

        self.description_Rotation = "Min:0\nMax:360\nStep:90\nDefault:90"

        self.get_logger().info(self.description_Color)
        self.get_logger().info(self.description_Rotation)

        self.declare_parameter('colorPalette', 'TYRIAN', descriptor=ParameterDescriptor(
            description=self.description_Color))
        self.colorPalette_input = str(self.get_parameter(
            'colorPalette').get_parameter_value().string_value).upper()

        self.declare_parameter('rotationValue', 90, descriptor=ParameterDescriptor(description=self.description_Rotation, type=ParameterType.PARAMETER_INTEGER, integer_range=[IntegerRange(from_value=0, to_value=360, step=90)],))
        self.rotationValue = self.get_parameter(
            'rotationValue').get_parameter_value().integer_value

        self.declare_parameter('greyscale', False, descriptor=ParameterDescriptor(description=self.description_Rotation, type=ParameterType.PARAMETER_INTEGER, integer_range=[IntegerRange(from_value=0, to_value=360, step=90)],))
        self.greyscale = self.get_parameter(
            'greyscale').get_parameter_value().bool_value
        
        self.publisher_ = self.create_publisher(Image, 'thermalImage', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.busy = False
        self.frame = SeekFrame()
        self.camera = SeekCamera()
        self.first_frame = True
        self.frame = None

        self.manager = SeekCameraManager(SeekCameraIOType.USB)
        self.br = CvBridge()

        self.manager.register_event_callback(self.on_event)

    def __del__(self):
        self.manager.destroy()

    def timer_callback(self):
        if self.frame == None:
            return

        img_array = self.frame.data
        height, width = img_array.shape[:2]
        center_x, center_y = width // 2, height // 2
        rotation_matrix = cv2.getRotationMatrix2D((center_x, center_y), self.rotationValue, 1.0)
        rotated_img = cv2.warpAffine(img_array, rotation_matrix, (width, height))

        if self.greyscale:
            self.publisher_.publish(self.br.cv2_to_imgmsg(rotated_img, "mono8"))
        else:
            self.publisher_.publish(self.br.cv2_to_imgmsg(rotated_img, "rgba8"))

    def on_frame(self, _camera, camera_frame, _):
        if self.greyscale:
            self.frame = camera_frame.grayscale
        else:
            self.frame = camera_frame.color_argb8888

    def on_event(self, camera, event_type, event_status, _):
        self.get_logger().info(("{}: {}".format(str(event_type), camera.chipid)))

        if event_type == SeekCameraManagerEvent.CONNECT:
            if self.busy:
                return

            self.busy = True
            self.camera = camera

            self.first_frame = True

            camera.color_palette = SeekCameraColorPalette[self.colorPalette_input]

            camera.register_frame_available_callback(self.on_frame, self)
            if self.greyscale:
                camera.capture_session_start(SeekCameraFrameFormat.GRAYSCALE)
            else:
                camera.capture_session_start(SeekCameraFrameFormat.COLOR_ARGB8888)

        elif event_type == SeekCameraManagerEvent.DISCONNECT:
            if self.camera == camera:
                camera.capture_session_stop()
                self.camera = None
                self.frame = None
                self.busy = False

        elif event_type == SeekCameraManagerEvent.ERROR:
            self.get_logger().error("{}: {}".format(str(event_status), camera.chipid))
            rclpy.shutdown()

        elif event_type == SeekCameraManagerEvent.READY_TO_PAIR:
            return


def main(args=None):
    rclpy.init(args=args)
    thermal_node = ThermalImagePublisher()
    rclpy.spin(thermal_node)
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
