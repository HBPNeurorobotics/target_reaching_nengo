# target_reaching_nengo

target reaching code with primitives, arm_robot and Nengo

## to start the main target reaching
* gazebo with arm and target should be running
* start the nengo gui:
```
roslaunch target_reaching_nengo Main_TR_CL_nengo_gui.launch
```
* in the gui click on "start" button
* for HBP mapping is necessary:
```
roslaunch target_reaching_nengo hbp_mapping.launch
```


## Nengo and ROS
to get a ros publisher running from the nengo_gui, we added a line in the nengo_gui/main.py (something like ~/.local/lib/python2.7/site-packages/nengo_gui/main.py) file before the start call:

``` python
    import rospy    # add this
    ...
    s = nengo_gui.gui.GUI(filename=args.filename,
                                       backend=args.backend)

    rospy.init_node('nengo_ros')   # this is the added line

    s.start(port=args.port, password=args.password, browser=args.browser)
```

https://github.com/nengo/nengo_gui/issues/827
