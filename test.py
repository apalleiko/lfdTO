from pydrake.all import *
import sys

scenario = "forward"   # "lift", "forward", or "side"
save_file = "panda_" + scenario + ".npz"

####################################
# Parameters
####################################

T = 0.5
dt = 1e-2
playback_rate = 0.125

# Some useful joint angle definitions
q_home = np.array([0., -0.785, 0., -2.356, 0., 1.57, .785])
q_push = np.array([0., 0.7, 0., -2.356, 0., 4.4, .785])
q_wrap = np.array([-2.0, -1.8, 2., -2.0, 0.0057, 1.1, -0.083])

# Some useful ball pose definitions
radius = 0.1   # of ball
q_ball_start = np.array([0, 0, 0, 1, 0.6, 0.0, radius])
q_ball_target = np.array([0, 0, 0, 1, 0.6, 0.0, radius])

# Initial state
q_start = q_push
x0 = np.hstack([q_start, q_ball_start, np.zeros(13)])

# Target state
x_nom = np.hstack([q_start, q_ball_target, np.zeros(13)])

# Contact model parameters
dissipation = 5.0              # controls "bounciness" of collisions: lower is bouncier
# controls "squishiness" of collisions: lower is squishier
hydroelastic_modulus = 5e7
resolution_hint = 0.05         # smaller means a finer mesh

mu_static = 0.3
mu_dynamic = 0.2

# Hydroelastic, Point, or HydroelasticWithFallback
contact_model = ContactModel.kHydroelastic
mesh_type = HydroelasticContactRepresentation.kTriangle  # Triangle or Polygon

####################################
# Tools for system setup
####################################

def create_panda_plant(plant, scene_graph, finalize=False, contact=True):
    # Add the panda arm model from urdf
    # (rigid hydroelastic contact included)
    urdf = "./assets/models/panda_fr3/urdf/panda_fr3.urdf"
    arm = Parser(plant).AddModels(urdf)
    X_robot = RigidTransform()
    # base attachment sets the robot up a bit
    X_robot.set_translation([0, 0, 0.015])
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("panda_link0"),
                     X_robot)

    if contact:
        # Add a ground with compliant hydroelastic contact
        ground_props = ProximityProperties()
        AddCompliantHydroelasticProperties(
            resolution_hint, hydroelastic_modulus, ground_props)
        friction = CoulombFriction(0.7*mu_static, 0.7*mu_dynamic)
        AddContactMaterial(dissipation=dissipation,
                        friction=friction, properties=ground_props)
        X_ground = RigidTransform()
        X_ground.set_translation([0, 0, -0.5])
        ground_shape = Box(25, 25, 1)
        plant.RegisterCollisionGeometry(plant.world_body(), X_ground,
                                        ground_shape, "ground_collision", ground_props)

        # Choose contact model
        plant.set_contact_surface_representation(mesh_type)
        plant.set_contact_model(contact_model)
    
    if finalize:
        plant.Finalize()

    return plant, scene_graph


def register_plant_with_scene_graph(builder, scene_graph, plant):
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    builder.Connect(
        plant.get_geometry_poses_output_port(),
        scene_graph.get_source_pose_port(plant.get_source_id()),
    )
    builder.Connect(
        scene_graph.get_query_output_port(),
        plant.get_geometry_query_input_port(),
    )


def create_ball_plant(plant, builder, finalize=True):
    # Add the panda arm model from urdf
    plant = builder.AddNamedSystem("ball", plant)

    # Add a ball with compliant hydroelastic contact
    mass = 0.258
    I = SpatialInertia(mass, np.zeros(3), UnitInertia.HollowSphere(radius))
    ball_instance = plant.AddModelInstance("ball")
    ball = plant.AddRigidBody("ball", ball_instance, I)
    X_ball = RigidTransform()

    ball_props = ProximityProperties()
    friction = CoulombFriction(0.7*mu_static, 0.7*mu_dynamic)
    AddCompliantHydroelasticProperties(
        resolution_hint, hydroelastic_modulus, ball_props)
    AddContactMaterial(dissipation=dissipation,
                       friction=friction, properties=ball_props)
    plant.RegisterCollisionGeometry(ball, X_ball, Sphere(radius),
                                    "ball_collision", ball_props)

    color = np.array([0.8, 1.0, 0.0, 0.5])
    plant.RegisterVisualGeometry(
        ball, X_ball, Sphere(radius), "ball_visual", color)

    # Add some spots to visualize the ball's roation
    spot_color = np.array([0.0, 0.00, 0.0, 0.5])
    spot_radius = 0.05*radius
    spot = Sphere(spot_radius)
    spot_offset = radius - 0.45*spot_radius

    plant.RegisterVisualGeometry(
        ball, RigidTransform(RotationMatrix(), np.array([radius, 0, 0])),
        spot, "sphere_x+", spot_color)
    plant.RegisterVisualGeometry(
        ball, RigidTransform(RotationMatrix(), np.array([-radius, 0, 0])),
        spot, "sphere_x-", spot_color)
    plant.RegisterVisualGeometry(
        ball, RigidTransform(RotationMatrix(), np.array([0, radius, 0])),
        spot, "sphere_y+", spot_color)
    plant.RegisterVisualGeometry(
        ball, RigidTransform(RotationMatrix(), np.array([0, -radius, 0])),
        spot, "sphere_y-", spot_color)
    plant.RegisterVisualGeometry(
        ball, RigidTransform(RotationMatrix(), np.array([0, 0, radius])),
        spot, "sphere_z+", spot_color)
    plant.RegisterVisualGeometry(
        ball, RigidTransform(RotationMatrix(), np.array([0, 0, -radius])),
        spot, "sphere_z-", spot_color)

    # Choose contact model
    plant.set_contact_surface_representation(mesh_type)
    plant.set_contact_model(contact_model)
    
    if finalize:
        plant.Finalize()

    return plant

####################################
# Create system diagram
####################################

# builder = DiagramBuilder()
# plant, scene_graph = AddMultibodyPlantSceneGraph(builder, dt)
# panda, scene_graph = create_panda_plant(plant, scene_graph)

# ball_plant = create_ball_plant(builder, scene_graph)

# meshcat = StartMeshcat()
# visualizer = MeshcatVisualizer.AddToBuilder( 
#     builder, scene_graph, meshcat,
#     MeshcatVisualizerParams(role=Role.kPerception, prefix="visual"))

# diagram = builder.Build()
# plot_graphviz(diagram.GetGraphvizString())

# diagram_context = diagram.CreateDefaultContext()
# panda_context = diagram.GetMutableSubsystemContext(panda, diagram_context)

# state = panda.GetPositionsAndVelocities(panda_context)
# print(state)
# panda.SetPositionsAndVelocities(panda_context, np.hstack([q_home, np.zeros(shape=q_home.shape)]))

# ball_context = diagram.GetMutableSubsystemContext(ball_plant, diagram_context)
# state = ball_plant.GetPositionsAndVelocities(ball_context)
# print(state)
# # sys.exit(-1)
# ball_plant.SetPositionsAndVelocities(ball_context, np.hstack([q_ball_start, [0,0,0,-0.1,0,0]]))
# diagram.ForcedPublish(diagram_context)

# simulator = Simulator(diagram)
# simulator.set_target_realtime_rate(1.0)
# simulator.AdvanceTo(0.2)