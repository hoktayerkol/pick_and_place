import os
from os import environ, pathsep

from scripts import GazeboRosPaths

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.actions import Node


# custom_models_dir = get_package_share_directory('test_world')
# custom_models_dir = os.path.join(custom_models_dir, 'models')
custom_models_dir = ''

tb3_world_model_dir = get_package_share_directory('turtlebot3_gazebo')
tb3_world_model_dir = os.path.join(tb3_world_model_dir, 'models')


model_names = {'turtlebot3_world':[0,0,0,0,0,0], 
               'qr_table1':[0,-2.5,0,0,0,1.5707],'qr_table2':[0,2.5,0,0,0,-1.5707], 
               'qr_table3':[2.5,-0.7,0,0,0,2.6179],'qr_table4':[-2.5,0,0,0,0,0]}

def spawn_model(name, pos):

    if (name=='turtlebot3_world'):
        pass
        model_path = os.path.join(tb3_world_model_dir, name, 'model.sdf')
    else:
        model_path = os.path.join(custom_models_dir, name, 'model.sdf')

    print(model_path)

    spawn_model_cmd = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', name,
                '-file', model_path,
                #'-robot_namespace', namespace,
                '-x', str(pos[0]), '-y', str(pos[1]), '-z', str(pos[2]),
                '-R', str(pos[3]), '-P', str(pos[4]), '-Y', str(pos[5])])
    
    return spawn_model_cmd


def set_gazebo_paths(models, plugins='', medias=''):
    # models, plugins and medias are lists with full path of models, plugins and medias

    # get registered paths
    model, plugin, media = GazeboRosPaths.get_paths()
    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep + environ['GAZEBO_MODEL_PATH']  
 
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep + environ['GAZEBO_PLUGIN_PATH']

    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep + environ['GAZEBO_RESOURCE_PATH']

    # add new paths
    for mdl in models:
        model += pathsep + mdl

    for plg in plugins:
        plugin += pathsep + plg

    for md in medias:
        media += pathsep + md

    # set environment variables
    if not model=='':
        os.environ['GAZEBO_MODEL_PATH'] = model
    if not plugin=='':
        os.environ['GAZEBO_PLUGIN_PATH'] = plugin
    if not media=='':
        os.environ['GAZEBO_RESOURCE_PATH'] = media

    # end of set_gazebo_paths


def generate_launch_description():
   
    
    models=[custom_models_dir, custom_models_dir]

    set_gazebo_paths(models)
    
    # start gazebo server
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 
                        'launch', 
                        'gzserver.launch.py')
        )
    )

    # start gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 
                        'launch', 
                        'gzclient.launch.py',
                        )
        )
    )


    ld = LaunchDescription()
    
    #ld.add_action(set_custom_models_path_cmd1)
    #ld.add_action(set_custom_models_path_cmd2)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    
    for model, position in model_names.items():
        ld.add_action(spawn_model(model, position))

    return ld