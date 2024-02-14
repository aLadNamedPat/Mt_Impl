from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mt_impl.PointGen import Map 

def launch_setup(context, *args, **kwargs):
    num_nodes = LaunchConfiguration('num_nodes').perform(context)
    num_goals = LaunchConfiguration('num_goals').perform(context)
    nodes = []

    try:
        num_nodes = int(num_nodes)
        num_goals = int(num_goals)
    except ValueError:
        num_nodes = 2
        num_goals = 2
    
    gen = Map(num_nodes, num_goals, 0.5)
    
    #We utilize the hungarian algorithm to generate starting and ending points
    start_end_pos = gen.hA()
    for i in range(int(num_nodes)):
        nodes.append(
            Node(
                package = "mt_impl",
                executable = "Linear_Int",
                name = f'linSim_{i}',
                parameters = 
                [
                    {
                    "starting_position": start_end_pos[i][0],
                    "final_position": start_end_pos[i][1]
                    }
                ]
             )
        )
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_nodes',
            default_value = '2',
            description = "Number of robots operating"
        ), 
        DeclareLaunchArgument(
            'num_goals',
            default_value = '2',
            description = "Number of goals to travel to"
        ),
        OpaqueFunction(function=launch_setup)
    ])


# def generate_launch_description():

#     current_position = [1.0, 2.0, 3.0]
#     final_position = [4.0, 5.0, 6.0]

#     return LaunchDescription([
#         # DeclareLaunchArgument("starting_position",
#         # default_value = str(current_position)),
#         # DeclareLaunchArgument("final_position",
#         # default_value = str(final_position)),

#         Node(
#             package = "mt_impl",
#             executable='Linear_Int',
#             name='linSim',
#             parameters=
#             [
#                 {
#                     "starting_position":current_position, 
#                     "final_position":final_position
#                 }
#             ]
#         )
#     ])