# target_reaching_nengo

target reaching code with primitives, arm_robot and Nengo


## code main repository
the full code repo for snn nengo is here --> auf branch pub_sub:
https://ids-git.fzi.de/steffen/snn_nengo_motion/tree/pub_sub/src/nengo_ros/src_lea/motion_modules 



## Nengo and ROS
to get a ros publisher running from the nengo_gui, we added a line in the nengo_gui/main.py file before the start call:

``` python
    import rospy    # add this
    ...
    s = nengo_gui.gui.GUI(filename=args.filename,
                                       backend=args.backend)

    rospy.init_node('nengo_ros')   # this is the added line

    s.start(port=args.port, password=args.password, browser=args.browser)
```

https://github.com/nengo/nengo_gui/issues/827  
