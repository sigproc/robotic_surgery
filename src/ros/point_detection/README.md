# Point detection nodes

A simple set of nodes for performing point detection in video streams using
OpenCV's Haar cascade classifier.

The most useful node is ``point_detect.py`` which will listen for incoming
images and attempt to detect points in them. It publishes detected points on a
ROS topic. There is a companion node, ``highlight_points.py`` which can be used
to draw boxes around points.

This set of nodes, along with [point_detection_msgs](../point_detection_msgs),
serve as an example template which you can use you write your own Python-based
nodes.

Note that the point detection node is quite sophisticated; it supports
configuration via ROS parameters and also decouples the input video sequences
from the point detection algorithm. Since the point detection algorithm runs
slower than 30Hz, this decoupling is needed to avoid ending up with high
latency or a backlog of frames.

## Usage

There is a demo launch file, ``point_detect_example.launch``, which will attempt
to detect points in real-time from an attached USB webcam. Launch it directly via:

```console
$ roslaunch point_detection point_detect.launch
```

or, if you are using our Docker-based workflow, via ``make gui_launch``:

```console
$ make gui_launch PKG=point_detection LAUNCH=point_detect.launch
```

When the bundled launch file is running, you may see the set of point detection
results via:

```console
$ rostopic echo /point_detect/points
```

## Nodes

### point_detect.py

Detect points in input images via OpenCV.

#### Subscribed topics

<dl>
<dt>``camera/image_raw`` (sensor_msgs/Image)</dt>
<dd>The image topic. Should be remapped to the name of the real image topic if
necessary.</dd>
</dl>

#### Published topics

<dl>
<dt>*name*/``points`` (point_detection_msgs/Points)</dt>
<dd>The detected points in an image. The sequence number and stamp will match
the ``Image`` message which was used. This fact may be used for
synchronisation.</dd>
</dl>

#### Parameters

<dl>
<dt>``~haar_cascade_dir`` (``string``, default: "/usr/share/opencv/haarcascades/")</dt>
<dd>Location of the directory on disk containing the pre-trained Haar
classifiers shipped with OpenCV.</dd>
</dl>

### highlight_points.py

Synchronise input camera images and point detection results and draw a highlight
around points detected in the input image.

#### Subscribed topics

<dl>
<dt>``camera/image_raw`` (sensor_msgs/Image)</dt>
<dd>The image topic. Should be remapped to the name of the real image topic if
necessary.</dd>
<dt>``point_detect/points`` (point_detection_msgs/Points)</dt>
<dd>The detected points in an image. Assumes that timestamps are synchronised
with the input images.</dd>
</dl>

#### Published topics

<dl>
<dt>*name*``/image_raw`` (sensor_msgs/Image)</dt>
<dd>The output image topic. Contains the input image with detected points
highlighted.</dd>
</dl>

## Launch files

### point_detect_example.launch

An example of a real-time point detection pipeline. The attached USB webcam is
used to capture video which is fed to ``point_detect.py``. The detected points
are then highlighted via ``highlight_points.py``. Two ``image_view`` nodes are
used to show the input and output video sequences.

## Related packages

[point_detection_msgs](../point_detection_msgs) - message types used by this library

