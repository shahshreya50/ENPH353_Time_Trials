# Eyeinthesky

`eyeinthesky` is a custom force-controlled drone model for ROS/Gazebo.

To use it in the 2025_competition world instead of `robbie`, the skid-steering robot,
go to `2025_competition/enph353/enph353_utils/launch/robots.launch` and change

```
<group ns="B1">
    <param name="tf_prefix" value="B1_tf" />
    <include file="$(find robbie)/launch/robbie.launch">
        <arg name="init_pose" value="-x 5.5 -y 2.5 -z 0.2 -R 0.0 -P 0.0 -Y -1.57" />
        <arg name="robot_name"  value="B1" />
    </include>
    <!-- Add teleop_keyboard controller -->
    <include file="$(find enph353_utils)/launch/desktop.launch"/>
</group>
```

to

```
<group ns="B1">
    <param name="tf_prefix" value="B1_tf" />
    <include file="$(find eyeinthesky)/launch/eyeinthesky.launch">
        <arg name="init_pose" value="-x 5.5 -y 2.5 -z 0.2 -R 0.0 -P 0.0 -Y -1.57" />
        <arg name="robot_name"  value="B1" />
    </include>
</group>
```

There is also a 'ghost' model, `eyeinthesky_ghost`. This version has collision, gravity and wind disabled so that it can be teleported around as to collect camera view data.