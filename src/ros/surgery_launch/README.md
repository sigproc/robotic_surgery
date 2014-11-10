A simple proportional controller ("PID_test.py") and a simple increment command ("auto_increment.py" or "auto_increment_2joints.py") has been added to the surgery_launch file.

They were built from existing "cobra_demo.launch" files. To run the demos, please change the final line of the "increment_demo.launch" file. Change the 'type' for node 'demo_motion' into the respective executable files.

In order to run from terminal, simply type 'make gui_launch LAUNCH=increment_demo.launch' 
