#!/usr/bin/env python

from gazebo_msgs.srv import SpawnEntity, SpawnEntityRequest
import numpy as np
import rospy
from rospy import ServiceProxy, wait_for_service

model_name = "model"
model_sdf_xml = """
<?xml version='1.0'?>
<sdf version='1.5'>
  <model name='{model_name}'>
    <pose>0 0 0 0 0 0</pose>
    <link name='{model_name}'>
      <inertial>
        <mass>0.057</mass>
      </inertial>
      <collision name='collision'>
        <geometry>
          <box>
            <size>.1 .1 .1</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>.1 .1 .1</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/{color}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

class SpawnGazeboURDFModel:
    def __init__(self, model_name, urdf_xml):
        self._model_name = model_name
        self._sdf_spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_entity',
                                               SpawnEntity, persistent=True)
        self._urdf_spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_urdf_entity',
                                               SpawnEntity, persistent=True)
        self._model_msg = SpawnEntityRequest()
        self._model_msg.entity_name = self._model_name
        self._sdf_xml = sdf_xml
        self._model_colors = ['Green']  #, 'Red', 'Blue', 'Orange', 'Yellow', 'Purple', 'Turquoise']
        self._model_msg.entity_xml = sdf_xml.format(model_name=model_name, color=np.random.choice(self._model_colors, size=1)[0])
        self._model_msg.initial_pose.position.x = 0.0
        self._model_msg.initial_pose.position.y = 0.0
        self._model_msg.initial_pose.position.z = 0.1
        self._model_msg.reference_frame = "world"

    def spawn_sdf(self):
        self._model_msg.entity_xml = self._sdf_xml.format(model_name=model_name, color=np.random.choice(self._model_colors, size=1)[0])
        self._sdf_spawn_proxy(self._model_msg)
        return 'success'

def main(argv=None):
    rospy.init_node("SpawnGazeboURDFModel")
    spawn = SpawnGazeboURDFModel("ball", model_sdf_xml)
    rospy.loginfo("SpawnGazeboURDFModel initialized")
    spawn.spawn_sdf()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    main()
