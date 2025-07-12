#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ur_client_library/ur/dashboard_client.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// #include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
// #include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/robot_state/robot_state.h>
// #include <trajectory_processing/iterative_parabolic_time_parameterization.h>


void addObjeto(shape_msgs::msg::SolidPrimitive primitive, geometry_msgs::msg::Pose pose)
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::msg::CollisionObject objeto;
  objeto.id = "pick_target";
  objeto.header.frame_id = "world";

  // Asigna geometría y pose
  objeto.primitives.push_back(primitive);
  objeto.primitive_poses.push_back(pose);
  objeto.operation = objeto.ADD;

  // Vector de objetos para añadir a la escena
  std::vector<moveit_msgs::msg::CollisionObject> objetos;
  objetos.push_back(objeto);

  // Añade a la escena
  planning_scene_interface.applyCollisionObjects(objetos);
}


void gripperControl(std::unique_ptr<urcl::DashboardClient>& dashboard, const std::string& comando) {
    return;
    dashboard->commandLoadProgram(comando);
    dashboard->commandPlay();
    dashboard->commandLoadProgram("external_control.urp");
    dashboard->commandPlay();
}

bool mover(moveit::planning_interface::MoveGroupInterface& move_group, const rclcpp::Logger& logger, geometry_msgs::msg::Pose pose, std::string frame, std::string nombreFase) {
    int intentos = 0;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode result;
    bool success;
    while (intentos<10) {
        move_group.setStartStateToCurrentState();
        move_group.setPoseTarget(pose, frame);

        success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_ERROR(logger, "Falló la planificación: %s", nombreFase.c_str());
            intentos++;
        }
        else {

            RCLCPP_INFO(logger, "Planificación '%s' exitosa, ejecutando.", nombreFase.c_str());
            result = move_group.execute(plan);

            if (result != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(logger, "Falló la ejecución del plan: %s", nombreFase.c_str());
                intentos++;

                const auto& joint_names = move_group.getJointNames();
                const auto& traj_points = plan.trajectory_.joint_trajectory.points;
                if (!traj_points.empty()) {
                    const auto& goal_positions = traj_points.back().positions;

                    RCLCPP_INFO(logger, "Posiciones objetivo del plan:");
                    for (size_t i = 0; i < goal_positions.size(); ++i) {
                        RCLCPP_INFO(logger, "  %s: %.6f", joint_names[i].c_str(), goal_positions[i]);
                    }
                }
            }
            else {
                return true;
            }
        }
    }
    RCLCPP_ERROR(logger, "Error grave en %s", nombreFase.c_str());
    return false;
}

bool moverCart(moveit::planning_interface::MoveGroupInterface& move_group, const rclcpp::Logger& logger, std::vector<geometry_msgs::msg::Pose> waypoints, std::string nombreFase) {
    int intentos = 0;

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0; // debería ser 0.3 al probar con hardware real, es el error que puede haber entre 2 puntos de la trayectoria
    const double eef_step = 0.01; // cuanta distancia hay entre cada punto de la trayectoria
    double fraction;
    // trajectory_processing::IterativeParabolicTimeParameterization iptp;
    moveit::core::MoveItErrorCode result;

    while (intentos<10) {
        move_group.setStartStateToCurrentState();
        
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, false);
        RCLCPP_INFO(logger, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

        // Si únicamente se ha podido calcular menos del 90% de la trayectoria, termina
        if (fraction < 0.9) {
            RCLCPP_ERROR(logger, "Falló la planificación: %s", nombreFase.c_str());
            intentos++;
        }
        else {
            // iptp.computeTimeStamps(trajectory);

            RCLCPP_INFO(logger, "Planificación '%s' exitosa, ejecutando.", nombreFase.c_str());
            result = move_group.execute(trajectory);

            if (result != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(logger, "Falló la ejecución del plan: %s", nombreFase.c_str());
                intentos++;
            
                const auto& first_point = trajectory.joint_trajectory.points.front();
                RCLCPP_INFO(logger, "---- Goal Joint Positions (first trajectory point) ----");
                for (size_t i = 0; i < trajectory.joint_trajectory.joint_names.size(); ++i) {
                    const auto& joint_name = trajectory.joint_trajectory.joint_names[i];
                    double position = first_point.positions[i];
                    RCLCPP_INFO(logger, "Joint %s: %.4f rad", joint_name.c_str(), position);
                }
            }
            else {
                return true;
            }
        }
    }
    RCLCPP_ERROR(logger, "Error grave en %s", nombreFase.c_str());
    return false;
}

int main(int argc, char** argv)
{
    //Inicialización con configuración del robot
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
 
    const std::string robot_ip = "212.128.172.172";  // ip del robot

    auto my_dashboard = std::make_unique<urcl::DashboardClient>(robot_ip);
    // if (!my_dashboard->connect())
    // {
    //     return 1;
    // }
    
    // my_dashboard->commandLoadProgram("external_control.urp");
    // my_dashboard->commandPlay();

    // Crear interface para el grupo de planificación
    auto node = rclcpp::Node::make_shared("celda_move_to_pose", node_options);
    
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 1);

    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Espera a que se conecte el subscriber
    
    static const std::string PLANNING_GROUP = "ur_arm";
    static const std::string GRIPPER_FRAME = "ur3e_tool0";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    // Para añadir o quitar objetos
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Para permitir contacto entre objetos
    planning_scene::PlanningScenePtr planning_scene;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");

    // Evitar que se mueva joint_5
    // moveit_msgs::msg::JointConstraint joint5_constraint;
    // joint5_constraint.joint_name = "ur3e_wrist_3_joint";
    // joint5_constraint.position = move_group.getCurrentJointValues()[5];  // valor actual
    // joint5_constraint.tolerance_above = 0.01;
    // joint5_constraint.tolerance_below = 0.01;
    // joint5_constraint.weight = 1.0;

    // moveit_msgs::msg::Constraints constraints;
    // constraints.joint_constraints.push_back(joint5_constraint);
    // move_group.setPathConstraints(constraints);


    // Define el cilindro
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.025;  // altura en metros
    primitive.dimensions[1] = 0.015; // radio en metros

    int i=0;
    while(i<10) {
    // Posicion del cilindro
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.4;
    pose.position.y = -0.3;
    pose.position.z = primitive.dimensions[0]/2;  // z + altura/2 para que se apoye sobre la superficie
    
    // Añado el cilindro
    addObjeto(primitive, pose);

    //Esperamos a que se actualice la escena
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Definir la pose pre-pick
    float distancia = 0.05; // Distancia de precaucion
    float d_prepick = primitive.dimensions[0]/2 + distancia; //Distancia de la posicion prepick, la mitad superior del cilindro y distancia extra
    pose.position.z += d_prepick  + 0.21; //distancia de precaucion + longitud del gripper
    // pose.orientation.x = 1; //Con orientación hacia abajo
    // pose.orientation.y = 0;
    // pose.orientation.z = 0;
    // pose.orientation.w = 0;
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, 0);  // rotación de 180° en X
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    
    if (!mover(move_group, node->get_logger(), pose, GRIPPER_FRAME, "pre_pick")) {
        return 0; //Falló
    }

    // Abrir gripper
    gripperControl(my_dashboard, "abrir.urp");

    // // Consigo la escena actual
    // planning_scene_monitor->requestPlanningSceneState();
    // planning_scene = planning_scene_monitor->getPlanningScene();

    // // Indico que parte pueden estar en colision
    // planning_scene->getAllowedCollisionMatrixNonConst().setEntry("gripper", "target_object", true);

    // // Enviar la escena modificada al entorno de planificación
    // moveit_msgs::msg::PlanningScene planning_scene_msg;
    // planning_scene->getPlanningSceneMsg(planning_scene_msg);
    // planning_scene_msg.is_diff = true;

    // planning_scene_pub->publish(planning_scene_msg);

    // Planificacion cartesian path para acercarse al objeto
    // Vector con los puntos de la trayectoria
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(pose);
    pose.position.z -= d_prepick/2;
    waypoints.push_back(pose);
    pose.position.z -= d_prepick/2;
    waypoints.push_back(pose);

    if (!moverCart(move_group, node->get_logger(), waypoints, "pick")) {
        return 0; //Falló
    }

    RCLCPP_INFO(node->get_logger(), "Unir objeto al movimiento del robot");
    // Indicar qué links están en contacto con el objeto para evitar errores por colision
    std::vector<std::string> touch_links;
    touch_links.push_back("gripper");

    move_group.attachObject("pick_target", GRIPPER_FRAME, touch_links);

    // Agarrar gripper
    gripperControl(my_dashboard, "agarrar.urp");
    
    // Planificacion cartesian path para alejarse hacia arriba
    waypoints.clear();
    waypoints.push_back(pose);
    pose.position.z += d_prepick/2;
    waypoints.push_back(pose);
    pose.position.z += d_prepick/2;
    waypoints.push_back(pose);

    if (!moverCart(move_group, node->get_logger(), waypoints, "post-pick")) {
        return 0; //Falló
    }

    // Definir posición pre-place
    pose.position.x = 0.0;
    pose.position.y = -0.3;
    //Mantenemos la misma posición en altura y orientación hacia abajo

    if (!mover(move_group, node->get_logger(), pose, GRIPPER_FRAME, "pre_place")) {
        return 0; //Falló
    }
    
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Planificacion cartesian path para acercarse al objetivo
    move_group.setStartStateToCurrentState();

    // Vector con los puntos de la trayectoria
    waypoints.clear();
    waypoints.push_back(pose);
    pose.position.z -= d_prepick/2;
    waypoints.push_back(pose);
    pose.position.z -= d_prepick/2;
    waypoints.push_back(pose);

    if (!moverCart(move_group, node->get_logger(), waypoints, "place")) {
        return 0; //Falló
    }

    // Abrir gripper
    gripperControl(my_dashboard, "abrir.urp");

    // Separar objeto del robot
    move_group.detachObject("pick_target");

    RCLCPP_INFO(node->get_logger(), "Eliminar objeto");
    std::vector<std::string> objetos_eliminar;
    objetos_eliminar.push_back("pick_target");
    planning_scene_interface.removeCollisionObjects(objetos_eliminar);

    // Planificacion cartesian path para alejarse del objeto
    move_group.setStartStateToCurrentState();
    // Vector con los puntos de la trayectoria
    waypoints.clear();
    waypoints.push_back(pose);
    pose.position.z += d_prepick/2;
    waypoints.push_back(pose);
    pose.position.z += d_prepick/2;
    waypoints.push_back(pose);

    if (!moverCart(move_group, node->get_logger(), waypoints, "post-place")) {
        return 0; //Falló
    }

    // Cerrar gripper
    gripperControl(my_dashboard, "cerrar.urp");

    // // RCLCPP_INFO(node->get_logger(), "Eliminar objeto");
    // // std::vector<std::string> objetos_eliminar;
    // // objetos_eliminar.push_back("pick_target");
    // // planning_scene_interface.removeCollisionObjects(objetos_eliminar);
    i++;
    }

    // Shutdown ROS
    rclcpp::shutdown();
    // Desconexión del server dashboard
    // my_dashboard->disconnect();
    return 0;
}
