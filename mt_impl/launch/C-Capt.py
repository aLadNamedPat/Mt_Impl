from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mt_impl.DSM import DSM

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
    
    #Distance squared matrix, Robot Starting Location Vector, Goal Positons Vector
    a = DSM(num_nodes, num_goals, 0.5)
    sM, rM, gM = a.distanceSquareMatrix()

    lsM = str(sM.tolist())
    lrM = str(rM.tolist())
    lgM = str(gM.tolist())

    nodes.append(
        Node(
            package = "mt_impl",
            executable = "ControlNode",
            name = "ControlNode",
            parameters = 
            [
                {
                    "sDM" : lsM,
                    "rPM" : lrM,
                    "gPM" : lgM
                }
            ]
        )
    )

    for i in range(int(num_nodes)):
        nodes.append(
            Node(
                package = "mt_impl",
                executable = "MultiLinear_Int",
                name = f'linSim_{i}',
                parameters = 
                [
                    {
                    "robot_id": i
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