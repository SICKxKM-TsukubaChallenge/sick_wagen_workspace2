from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    v4l2_node = Node(
        package = 'v4l2_camera',
        executable = 'v4l2_camera_node',
        parameters = [{'video_device':"/dev/video2"}], #cameraを指定
        output = 'screen'
    )
    
    image_viewer_node = Node(
        package = 'rqt_image_view',
        executable = 'rqt_image_view',
        output = 'screen'
    )
    
    ld.add_action(v4l2_node)
    ld.add_action(image_viewer_node)
    
    return ld