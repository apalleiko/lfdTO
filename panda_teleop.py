import numpy as np
from pydrake.all import *
from test import create_ball_plant, create_panda_plant
import time
import matplotlib
from matplotlib import pyplot as plt
import matplotlib as mpl
mpl.rcParams['figure.figsize'] = [20, 20]

dt = 1e-2

meshcat = StartMeshcat()
meshcat.ResetRenderMode()
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder,dt)
plant, scene_graph = create_panda_plant(plant, scene_graph, finalize=True)
visualizer = MeshcatVisualizer.AddToBuilder( 
    builder, scene_graph, meshcat,
    MeshcatVisualizerParams(role=Role.kPerception, prefix="visual"))

plot_graphviz(plant.GetTopologyGraphvizString())

meshcat.DeleteAddedControls()
teleop = builder.AddSystem(
        MeshcatPoseSliders(
            meshcat,
            lower_limit=[0, -0.5, -np.pi, -0.6, -0.8, 0.0],
            upper_limit=[2 * np.pi, np.pi, np.pi, 0.8, 0.3, 1.1],
        )
    )

arm = plant.GetModelInstanceByName("panda")
n = int(plant.num_multibody_states(arm)/2)
params = DifferentialInverseKinematicsParameters(n,n)
ik = builder.AddSystem(
    DifferentialInverseKinematicsIntegrator(plant, plant.GetFrameByName("panda_hand"),dt,params)
    )

controller = builder.AddSystem(
    PidController(10*np.ones(n),0*np.ones(n),3*np.ones(n))
    )

builder.Connect(
    teleop.get_output_port(),
    ik.GetInputPort("X_WE_desired"),
    )

builder.Connect(
    plant.GetOutputPort("state"),
    ik.GetInputPort("robot_state")
    )

builder.Connect(
    plant.GetOutputPort("panda_state"),
    controller.GetInputPort("estimated_state")
    )

# builder.Connect(
#     ik.GetOutputPort("joint_positions"),
#     controller.GetInputPort("desired_state")
#     )

builder.ExportOutput(ik.get_output_port())
diagram = builder.Build()

plot = plot_graphviz(diagram.GetGraphvizString())

radius = 0.1   # of ball
q_ball_start = np.array([0, 0, 0, 1, 0.6, 0.0, radius])
q_home = np.array([0., -0.785, 0., -2.356, 0., 1.57, .785])
# x0 = np.hstack([q_home, q_ball_start, np.zeros(13)])
x0 = np.hstack([q_home, np.zeros(7)])

diagram_context = diagram.CreateDefaultContext()

plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
plant.SetPositionsAndVelocities(plant_context, x0)

ik_context = diagram.GetMutableSubsystemContext(ik, diagram_context)
ik.SetPositions(ik_context,q_home)

X_AE = ik.ForwardKinematics(ik_context)
teleop.SetPose(X_AE)

diagram.ForcedPublish(diagram_context)
state = diagram.get_output_port().Eval(diagram_context)
print(state)

# simulator = Simulator(diagram)
# simulator.set_target_realtime_rate(.25)
# sim_context = simulator.get_mutable_context()
# sim_context.SetStateAndParametersFrom(diagram_context)
# simulator.AdvanceTo(0.1)

# meshcat.AddButton("Stop Simulation", "Escape")
# print("Press Escape to stop the simulation")
# while meshcat.GetButtonClicks("Stop Simulation") < 1:
#     context = simulator.get_context()
#     state = diagram.get_output_port().Eval(context)
#     # print(state)
#     simulator.AdvanceTo(simulator.get_context().get_time() + 1)

# meshcat.DeleteButton("Stop Simulation")
