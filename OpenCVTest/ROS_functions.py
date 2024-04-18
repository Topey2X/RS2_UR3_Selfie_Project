import rospy

class ROSNODE_img_processor:
  latest_image = None
  latest_output = None

  _rate = None
  _sub_webcam = None
  _pub_contours = None

  _process_flag = False
  _processing_function: function = None

  def image_callback(self, data):
    self.latest_image = data.pose.pose.position  # TODO
    self._process_flag = True

  def init_ros(self, process: function):
    self._processing_function = process
    rospy.init_node("distance_to_hazards")
    self._rate = rospy.Rate(5)
    rospy.Subscriber("/usbcam/image", TYPE_TODO, self.image_callback)
    self.pub_detected = rospy.Publisher("/selfie/lines", TYPE_TODO, queue_size=1)

  def __init__(self, process: function):
    self.init_ros(process)

  def run(self):
    while not rospy.is_shutdown():
      if self._process_flag:
        self._process_flag = False
        self._processing_function()
        self._pub_contours.publish()
      self._rate.sleep()
