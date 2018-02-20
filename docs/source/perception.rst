Perception
==========

Setting up ASTRA camera
^^^^^^^^^^^^^^^^^^^^^^^

Visit `this link <http://jsk-recognition.readthedocs.io/en/latest/install_astra_camera.html>`_ to find the documentation of ASTRA.

The following codes are run in the terminal to run the ASTRA camera.

>>> $ roscore
>>> $ roslaunch openni2_launch openni2.launch

Extracting data from ASTRA
^^^^^^^^^^^^^^^^^^^^^^^^^^

By subscribing to two nodes of the camera through message filters, the depth grayscale and the colour frames of the camera input can be obtained.

* Image node: ``/camera/rgb/image_rect_color``
* Depth node: ``/camera/depth_registered/image_raw``

The mouth recognition is done through an OpenCV shap_predictor_68_face_landmarks.dat. The wideness of the mouth can also be determined and used.