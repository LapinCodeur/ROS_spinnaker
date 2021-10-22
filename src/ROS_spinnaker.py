#!/usr/bin/python3

import sys
import cv2
import rospy
import PySpin as ps

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CamLight:
    def __init__(self):
        self.br = CvBridge()
        self.image_pub = rospy.Publisher("flir", Image, queue_size=1)

        self.system = ps.System.GetInstance()
        self.cam_list = self.system.GetCameras()
        # Close if no camera detected
        if self.cam_list.GetSize() == 0:
            self.cam_list.Clear()
            self.system.ReleaseInstance()
            raise OSError('No camera has been detected')

        self.cam = self.cam_list[0]
        self.n_map_tldevice = self.cam.GetTLDeviceNodeMap()
        # Initialize camera
        self.cam.Init()
        # Retrieve GenICam nodemap
        self.n_map = self.cam.GetNodeMap()

    def launch(self):
        self.set_buffer()
        rospy.loginfo("The camera has been launched")
        try:
            self.set_acquisition()
            print('Acquisition mode set to continuous.')
            rospy.loginfo(f'Framerate is set to {self.get_framerate()}')

            self.cam.BeginAcquisition()

            # Retrieve and publish images
            while not rospy.is_shutdown():
                try:
                    img = self.cam.GetNextImage(1000)

                    #  Ensure image completion
                    if img.IsIncomplete():
                        rospy.logwarn(f"Image incomplete (status {img.GetImageStatus()})")
                    else:
                        # Convert image to BGR8
                        img_conv= img.Convert(ps.PixelFormat_BGR8)
                        # Getting the image data as a numpy array
                        img_data = img_conv.GetNDArray()
                        rospy.loginfo_once("Grabbed Image, height = {}, width = {}".format(*img_data.shape[:2]))
                        # Split channels of the image as image is GRB instead of RGB
                        b = img_data[:, :, 0]
                        g = img_data[:, :, 1]
                        r = img_data[:, :, 2]
                        # Publish image
                        self.image_pub.publish(self.br.cv2_to_imgmsg(cv2.merge([b, r, g]), "bgr8"))

                    img.Release()

                except ps.SpinnakerException as ex:
                    rospy.logerr(f'Error: {ex}')
                    return False

            self.cam.EndAcquisition()
            self.cam.DeInit()

        except ps.SpinnakerException as ex:
            rospy.logerr(f'Error: {ex}')
            return False

        return True

    def get_framerate(self):
        n_framerate = ps.CFloatPtr(self.n_map.GetNode('AcquisitionFrameRate'))

        if not ps.IsAvailable(n_framerate) and not ps.IsReadable(n_framerate):
            print('Unable to retrieve framerate.')
            return False
        return n_framerate.GetValue()

    def set_buffer(self):
        sNodemap = self.cam.GetTLStreamNodeMap()
        n_newestonly_mode = None
        # Change bufferhandling mode to NewestOnly
        n_bufferhandling_mode = ps.CEnumerationPtr(sNodemap.GetNode('StreamBufferHandlingMode'))
        if ps.IsAvailable(n_bufferhandling_mode) and ps.IsWritable(n_bufferhandling_mode):
            # Retrieve entry node from enumeration node
            n_newestonly = n_bufferhandling_mode.GetEntryByName('NewestOnly')
            if ps.IsAvailable(n_newestonly) and ps.IsReadable(n_newestonly):
                # Retrieve value from entry node
                n_newestonly_mode = n_newestonly.GetValue()
                # Set value from entry node as new value of enumeration node
                n_bufferhandling_mode.SetIntValue(n_newestonly_mode)
        if n_newestonly_mode is None:
            raise AttributeError("Unable to set stream buffer handling mode")

    def set_acquisition(self):
        acquisition_mode_continuous = None
        n_acquisition_mode = ps.CEnumerationPtr(self.n_map.GetNode('AcquisitionMode'))
        if ps.IsAvailable(n_acquisition_mode) and ps.IsWritable(n_acquisition_mode):
            # Retrieve entry node from enumeration node
            n_acquisition_mode_continuous = n_acquisition_mode.GetEntryByName('Continuous')
            if ps.IsAvailable(n_acquisition_mode_continuous) and ps.IsReadable(n_acquisition_mode_continuous):
                # Retrieve value from entry node
                acquisition_mode_continuous = n_acquisition_mode_continuous.GetValue()
                # Set value from entry node as new value of enumeration node
                n_acquisition_mode.SetIntValue(acquisition_mode_continuous)
        if acquisition_mode_continuous is None:
            raise AttributeError("Unable to set acquisition mode to continuous (entry retrieval).")

    def quit(self):
        del self.cam
        # Clear camera list before releasing system
        self.cam_list.Clear()
        # Release system instance
        self.system.ReleaseInstance()

def main():
    rospy.init_node('ROS_spinnaker', anonymous=True)

    cl = CamLight()
    result = True
    result &= cl.launch()
    cl.quit()

    return result

if __name__ == '__main__':
    if main():
        sys.exit(0)
    else:
        sys.exit(1)
