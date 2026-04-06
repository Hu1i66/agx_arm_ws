import os
f = '/home/lxf/agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/config/agx_arm.urdf.xacro'
with open(f, 'r') as file:
    content = file.read()
if '<xacro:arg name="use_gazebo" default="false" />' not in content:
    content = content.replace('<xacro:arg name="effector_type" default="none" />', '<xacro:arg name="effector_type" default="none" />\n  <xacro:arg name="use_gazebo" default="false" />')
    content = content.replace('name="FakeSystem"', 'name="FakeSystem"\n    use_gazebo="$(arg use_gazebo)"')
    # only append if not there
    content = content.replace('</robot>', '''
  <xacro:if value="$(arg use_gazebo)">
    <gazebo>
      <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
        <parameters>$(find agx_arm_moveit)/config/ros2_controllers.yaml</parameters>
      </plugin>
    </gazebo>
  </xacro:if>
</robot>
''')
with open(f, 'w') as file:
    file.write(content)
