#!/usr/bin/env python3

import rospy
import numpy as np
import tf.transformations as tr
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Quaternion, Point

# Default: -x 5.5 -y 2.5 -z 0.2 -R 0.0 -P 0.0 -Y -1.57 (see robots.launch)
DEFAULT_XYZ = Point(5.5, 2.5, 0.2)
DEFAULT_ORIENTATION = Quaternion(0.0, 0.0, -np.sqrt(2)/2, np.sqrt(2)/2)
DEFAULT_WORLD_POSE = Pose(DEFAULT_XYZ, DEFAULT_ORIENTATION)

def move_model_relative(
        move_model_name: str,
        local_pose: Pose,
        ref_model_name: str = None
    ):
    """
    Moves move_model_name to a pose relative to ref_model_name.

    If ref_model_name is not provided, moves relative to the world frame.
    """
    rospy.wait_for_service('/gazebo/get_model_state')
    rospy.wait_for_service('/gazebo/set_model_state')
    
    try:
        # Get reference model's current world position
        if ref_model_name is None:
            # Default: local_pose is relative to world frame
            m_ref = np.eye(4)
        else:
            get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            ref_state = get_state(ref_model_name, 'world')
            
            if not ref_state.success:
                rospy.logerr(f"Failed to find reference model: {ref_model_name}")
                return

            # Convert Reference Pose to 4x4 Matrix
            m_ref = tr.concatenate_matrices(
                tr.translation_matrix([ref_state.pose.position.x, 
                                    ref_state.pose.position.y, 
                                    ref_state.pose.position.z]),
                tr.quaternion_matrix([ref_state.pose.orientation.x, 
                                    ref_state.pose.orientation.y, 
                                    ref_state.pose.orientation.z, 
                                    ref_state.pose.orientation.w])
            )

        # Convert Local Target Pose to 4x4 Matrix
        m_local = tr.concatenate_matrices(
            tr.translation_matrix([local_pose.position.x, 
                                   local_pose.position.y, 
                                   local_pose.position.z]),
            tr.quaternion_matrix([local_pose.orientation.x, 
                                  local_pose.orientation.y, 
                                  local_pose.orientation.z, 
                                  local_pose.orientation.w])
        )

        # Chain: World_Target = World_Ref @ Ref_Target
        m_final = m_ref @ m_local

        # Extract components for the new ModelState
        trans = tr.translation_from_matrix(m_final)
        quat = tr.quaternion_from_matrix(m_final)

        new_state = ModelState()
        new_state.model_name = move_model_name
        new_state.pose.position = Point(*trans)
        new_state.pose.orientation = Quaternion(*quat)
        new_state.reference_frame = "world"

        # Apply the movement in Gazebo
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state(new_state)
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def respawn_model(model_name: str):
    """
    Moves move_model_name to DEFAULT_WORLD_POSE relative to the world frame
    """
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        new_state = ModelState()
        new_state.model_name = model_name
        new_state.pose = DEFAULT_WORLD_POSE
        new_state.reference_frame = "world"

        # Apply the movement in Gazebo
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state(new_state)
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
