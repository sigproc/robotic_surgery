# Face detection nodes

A simple set of nodes for performing face detection in video streams using
OpenCV's Haar cascade classifier.

The most useful node is ``face_detect.py`` which will listen for incoming
images and attempt to detect faces in them. It publishes detected faces on a
ROS topic. There is a companion node, ``highlight_faces.py`` which can be used
to draw boxes around faces.

This set of nodes, along with [face_detection_msgs](../face_detection_msgs),
serve as an example template which you can use you write your own Python-based
nodes.

Note that the face detection node is quite sophisticated; it supports
configuration via ROS parameters and also decouples the input video sequences
from the face detection algorithm. Since the face detection algorithm runs
slower than 30Hz, this decoupling is needed to avoid ending up with high
latency or a backlog of frames.

## Usage

There is a demo launch file, ``face_detect_example.launch``, which will attempt
to detect faces in real-time from an attached USB webcam. Launch it directly via:

```console
$ roslaunch face_detection face_detect_example.launch
```

or, if you are using our Docker-based workflow, via ``make gui_launch``:

```console
$ make gui_launch PKG=face_detection LAUNCH=face_detect_example.launch
```

When the bundled launch file is running, you may see the set of face detection
results via:

```console
$ rostopic echo /face_detect/faces
```

## Nodes

### face_detect.py

Detect faces in input images via OpenCV.

#### Subscribed topics

<dl>
<dt>``camera/image_raw`` (sensor_msgs/Image)</dt>
<dd>The image topic. Should be remapped to the name of the real image topic if
necessary.</dd>
</dl>

#### Published topics

<dl>
<dt>*name*/``faces`` (face_detection_msgs/Faces)</dt>
<dd>The detected faces in an image. The sequence number and stamp will match
the ``Image`` message which was used. This fact may be used for
synchronisation.</dd>
</dl>

#### Parameters

<dl>
<dt>``~haar_cascade_dir`` (``string``, default: "/usr/share/opencv/haarcascades/")</dt>
<dd>Location of the directory on disk containing the pre-trained Haar
classifiers shipped with OpenCV.</dd>
</dl>

### highlight_faces.py

Synchronise input camera images and face detection results and draw a highlight
around faces detected in the input image.

#### Subscribed topics

<dl>
<dt>``camera/image_raw`` (sensor_msgs/Image)</dt>
<dd>The image topic. Should be remapped to the name of the real image topic if
necessary.</dd>
<dt>``face_detect/faces`` (face_detection_msgs/Faces)</dt>
<dd>The detected faces in an image. Assumes that timestamps are synchronised
with the input images.</dd>
</dl>

#### Published topics

<dl>
<dt>*name*``/image_raw`` (sensor_msgs/Image)</dt>
<dd>The output image topic. Contains the input image with detected faces
highlighted.</dd>
</dl>

## Launch files

### face_detect_example.launch

An example of a real-time face detection pipeline. The attached USB webcam is
used to capture video which is fed to ``face_detect.py``. The detected faces
are then highlighted via ``highlight_faces.py``. Two ``image_view`` nodes are
used to show the input and output video sequences.

## Related packages

[face_detection_msgs](../face_detection_msgs) - message types used by this library

