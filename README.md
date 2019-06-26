# Joint State publisher
## Gazebo plugin for ROS
This repository contains a ROS1-package with Gazebo model plugin.

The plugin publishes the states of the joints of the parent model.
Tested on Ubuntu 18.04 LTS + ROS Melodic + Gazebo 9.9.

## Examples
Put the plugin inside `<model>` tag.

### Publish all non-fixed model joints:
```
<plugin name="JointStatePublisher" filename="libJointStatePublisher.so"/>
```
### Publish specified model joints:
One tag:
```
<plugin name="JointStatePublisher" filename="libJointStatePublisher.so">
    <joints>joint_a, joint_B</joints>
    <publish_selected_only>1</publish_selected_only>
</plugin>
```
Multiple tags:
```
<plugin name="JointStatePublisher" filename="libJointStatePublisher.so">
    <publish_selected_only>true</publish_selected_only>
    <joint>roll_joint</joint>
    <joint>yaw_joint</joint>
    <joint>pitch_joint</joint>
</plugin> 
```

Both 1 and true accepted as boolean in `<publish_selected_only>` tag


