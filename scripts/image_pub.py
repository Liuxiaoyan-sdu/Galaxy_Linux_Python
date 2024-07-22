import cv2
import gxipy as gx
import time
import numpy as np
import rospy
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge


def acq_color(device, image_publisher):
    """
    Acquisition function for color device
    :param device: device object [Device]
    :param image_publisher: ROS publisher for image [rospy.Publisher]
    """
    bridge = CvBridge()
    while not rospy.is_shutdown():
        time.sleep(0.1)

        # Send software trigger command
        device.TriggerSoftware.send_command()

        # Get raw image
        raw_image = device.data_stream[0].get_image()
        if raw_image is None:
            print("Getting image failed.")
            continue

        # Get RGB image from raw image
        rgb_image = raw_image.convert("RGB")
        if rgb_image is None:
            continue

        # Create numpy array with data from raw image
        numpy_image = rgb_image.get_numpy_array()
        if numpy_image is None:
            continue

        # Publish image as ROS message
        ros_image = bridge.cv2_to_imgmsg(cv2.cvtColor(numpy_image, cv2.COLOR_RGB2BGR), "bgr8")
        ros_image.header.stamp = rospy.Time.now()
        image_publisher.publish(ros_image)


def acq_mono(device, image_publisher):
    """
    Acquisition function for mono device
    :param device: device object [Device]
    :param image_publisher: ROS publisher for image [rospy.Publisher]
    """
    bridge = CvBridge()
    while not rospy.is_shutdown():
        time.sleep(0.1)

        # Send software trigger command
        device.TriggerSoftware.send_command()

        # Get raw image
        raw_image = device.data_stream[0].get_image()
        if raw_image is None:
            print("Getting image failed.")
            continue

        # Create numpy array with data from raw image
        numpy_image = raw_image.get_numpy_array()
        if numpy_image is None:
            continue

        # Publish image as ROS message
        ros_image = bridge.cv2_to_imgmsg(numpy_image, "mono8")
        ros_image.header.stamp = rospy.Time.now()
        image_publisher.publish(ros_image)



def main():
    # Initialize ROS node
    rospy.init_node('camera_publisher', anonymous=True)

    # Create a ROS publisher for the image topic
    image_publisher = rospy.Publisher('camera/image', ROSImage, queue_size=10)

    # Print the demo information
    print("")
    print("-------------------------------------------------------------")
    print("Sample to show how to acquire mono or color image by soft trigger "
          "and publish acquired image using ROS.")
    print("-------------------------------------------------------------")
    print("")
    print("Initializing......")
    print("")

    # Create a device manager
    device_manager = gx.DeviceManager()
    dev_num, dev_info_list = device_manager.update_device_list()
    if dev_num == 0:
        print("Number of enumerated devices is 0")
        return

    # Open the first device
    cam = device_manager.open_device_by_index(1)

    # Set exposure
    cam.ExposureTime.set(10000)

    # Set gain
    cam.Gain.set(10.0)

    if dev_info_list[0].get("device_class") == gx.GxDeviceClassList.USB2:
        # Set trigger mode
        cam.TriggerMode.set(gx.GxSwitchEntry.ON)
    else:
        # Set trigger mode and trigger source
        cam.TriggerMode.set(gx.GxSwitchEntry.ON)
        cam.TriggerSource.set(gx.GxTriggerSourceEntry.SOFTWARE)

    # Start data acquisition
    cam.stream_on()

    # Determine if the camera is color or mono
    if cam.PixelColorFilter.is_implemented() is True:
        acq_color(cam, image_publisher)
    else:
        acq_mono(cam, image_publisher)

    # Stop acquisition
    cam.stream_off()

    # Close device
    cam.close_device()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
