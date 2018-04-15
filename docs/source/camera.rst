Cameras
=======

Two main cameras were used in the setup; one RGB camera embedded into the hand of the Baxter robot and one externally placed RGB-D Kinect camera.  To connect to a camera RGB and depth video stream within a python script, you need to subscribe to the camera topics.

To access the video stream for analysis with opencv, a package called cv_bridge is required to convert the image subscription to an image format. Cv_bridge needs to be defined in the init function to be used later.

.. code-block:: python

    #initialise bridge:
    self.bridge = CvBridge()

    # in main code:
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

Now the image is in RGB format that OpenCV can understand.

Hand Camera
^^^^^^^^^^^

The hand camera is part of the baxter ros environment. Therefore to access the topic, you must run the python code within catkin and launch baxter:

.. code-block:: python

    $ cd catkin_ws
    $ bash baxter.sh

Because the hand camera has only one available topic (RGB), a simple rospy subscription can be used.

.. code-block:: python

    self.image_sub = rospy.Subscriber("/cameras/right_hand_camera/image",Image,self.callback)

The subscriber requires a callback function to send the subscribed video stream to.

Kinect Camera
^^^^^^^^^^^^^

To access the kinect camera, first the driver needs to be installed along with openni.

In order to access the RGB and depth camera streams at the same time, a function called approximate time synchroniser is required. This bundles both video streams together even if they have different time stamps and sends them to the determined function together.

.. code-block:: python

    # initialise subscribers:
    image_sub = Subscriber("/camera/rgb/image_rect_color",Image)
    depth_sub = Subscriber("/camera/depth_registered/image_raw", Image)

    # bundle together subscribers
    tss = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub],queue_size=10, slop=0.5)

    # hit up the callback function with both images
    tss.registerCallback(self.callback)

Perception
^^^^^^^^^^
For more information on the facial, mouth and object/food detection done, please take a look at our `reports (section 3.3 and 3.4).
<https://docs.google.com/document/d/1gYLpNBUq5_0UUhcM4bcgzlkC_TXOMYgSC03uhh11frI/edit?usp=sharing>`_