#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ur_client_library/ur/dashboard_client.h>
#include <yaml-cpp/yaml.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// #include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
// #include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/robot_state/robot_state.h>
// #include <trajectory_processing/iterative_parabolic_time_parameterization.h>

//En las funciones pasar directamente node para evitar tantos parámetros?
void addObjeto(const std::string& id, const std::string& frame_id, shape_msgs::msg::SolidPrimitive primitive, geometry_msgs::msg::Pose pose)
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::msg::CollisionObject objeto;
  objeto.id = id;
  objeto.header.frame_id = frame_id;

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

bool gripperControl(std::unique_ptr<urcl::DashboardClient>& dashboard, const std::string& comando) {
    // // return true;
    // int intentos = 0;
    // while (!dashboard->commandLoadProgram(comando)) {
    //     rclcpp::sleep_for(std::chrono::milliseconds(250));
    //     if (intentos >= 10) {
    //         return false;
    //     }
    //     intentos++;
    // }
    
    // intentos = 0;
    // while (!dashboard->commandPlay()) {
    //     rclcpp::sleep_for(std::chrono::milliseconds(250));
    //     if (intentos >= 10) {
    //         return false;
    //     }
    //     intentos++;
    // }
    
    // dashboard->commandPlay();

    // std::string estado;
    // do
    // {
    //     dashboard->commandProgramState(estado);
    //     rclcpp::sleep_for(std::chrono::milliseconds(100));
    //     // Espera a que el comando se complete
    // } while (estado == "PLAYING " + comando || estado == "LOADING " + comando);

    // dashboard->commandLoadProgram("external_control.urp");
    // dashboard->commandPlay();

    // return true;
    // Cambiarlo por un for a ver que pasa
    // Y añadir un logger para ver si se ha cargado el programa
    // Y añadir tiempo de espera entre intentos
    int intentos = 0;
    while (!dashboard->commandLoadProgram(comando)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        if (++intentos >= 10) {
        RCLCPP_ERROR(rclcpp::get_logger("gripper"), "No pude cargar '%s'", comando.c_str());
        return false;
        }
    }

    // Arranca la ejecución del programa
    intentos = 0;
    while (!dashboard->commandPlay()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        if (++intentos >= 10) {
        RCLCPP_ERROR(rclcpp::get_logger("gripper"), "No pude ejecutar '%s'", comando.c_str());
        return false;
        }
    }

    // Espera a que termine PLAYING o LOADING
    std::string estado;
    do {
        dashboard->commandProgramState(estado);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (estado.rfind("PLAYING", 0) == 0 || estado.rfind("LOADING", 0) == 0);

    // Vuelve al programa de external_control
    intentos = 0;
    while (!dashboard->commandLoadProgram("external_control.urp")) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        if (++intentos >= 10) {
        RCLCPP_ERROR(rclcpp::get_logger("gripper"),
                    "No pude recargar 'external_control.urp'");
        return false;
        }
    }
    dashboard->commandPlay();

    RCLCPP_INFO(rclcpp::get_logger("gripper"), "Gripper '%s' completado.", comando.c_str());
    return true;
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

bool pick_and_place(moveit::planning_interface::MoveGroupInterface& move_group, const rclcpp::Logger& logger,rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor, const std::string gripper_frame, float distancia, const std::string robot_ip, geometry_msgs::msg::Pose place_pose)
{
    auto my_dashboard = std::make_unique<urcl::DashboardClient>(robot_ip);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    if (!my_dashboard->connect())
    {
        return 1;
    }
    
    my_dashboard->commandLoadProgram("external_control.urp");
    my_dashboard->commandPlay();

    // Para permitir contacto entre objetos
    planning_scene::PlanningScenePtr planning_scene;

    // float distancia = 0.05; // Distancia de precaucion
    std::vector<std::string> nombres_objetos = planning_scene_interface.getKnownObjectNames();
    auto object_poses = planning_scene_interface.getObjectPoses(nombres_objetos);
    
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, M_PI/4);  // rotación de 180° en X, 0° en Y, 45° en Z

    const float D_PLACE = -0.07; // Distancia entre cada posición de place
    int i = 0;

    for (const auto& pair : object_poses) {
        geometry_msgs::msg::Pose pose = pair.second; 
        RCLCPP_INFO(logger, "Objeto: %s, Pose: [%.2f, %.2f, %.2f]",pair.first.c_str(), pose.position.x, pose.position.y, pose.position.z);
        float d_prepick = pose.position.z + distancia; //Distancia de la posición prepick, la mitad superior del cilindro y distancia extra
        pose.position.z += d_prepick; //distancia de precaucion + longitud del gripper
        RCLCPP_INFO(logger, "Orientation: [%.2f, %.2f, %.2f, %.2f]",q.x(), q.y(), q.z(), q.w());
        
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        if (!mover(move_group, logger, pose, gripper_frame, "pre_pick")) {
            return false; //Falló
        }

        // Abrir gripper
        gripperControl(my_dashboard, "abrir.urp");

        // // Consigo la escena actual
        planning_scene_monitor->requestPlanningSceneState();
        planning_scene = planning_scene_monitor->getPlanningScene();

        // Indico que parte pueden estar en colision
        planning_scene->getAllowedCollisionMatrixNonConst().setEntry("gripper", pair.first.c_str(), true);

        // Enviar la escena modificada al entorno de planificación
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene->getPlanningSceneMsg(planning_scene_msg);
        planning_scene_msg.is_diff = true;

        planning_scene_pub->publish(planning_scene_msg);

        // Planificacion cartesian path para acercarse al objeto
        // Vector con los puntos de la trayectoria
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(pose);
        pose.position.z -= d_prepick/2;
        waypoints.push_back(pose);
        pose.position.z -= d_prepick/2;
        waypoints.push_back(pose);

        if (!moverCart(move_group, logger, waypoints, "pick")) {
            return false; //Falló
        }

        RCLCPP_INFO(logger, "Unir objeto al movimiento del robot");
        // Indicar qué links están en contacto con el objeto para evitar errores por colision
        std::vector<std::string> touch_links;
        touch_links.push_back("gripper");

        move_group.attachObject(pair.first.c_str(), gripper_frame, touch_links);

        // Agarrar gripper
        gripperControl(my_dashboard, "agarrar.urp");
        
        // Planificacion cartesian path para alejarse hacia arriba
        waypoints.clear();
        waypoints.push_back(pose);
        pose.position.z += d_prepick/2;
        waypoints.push_back(pose);
        pose.position.z += d_prepick/2;
        waypoints.push_back(pose);

        if (!moverCart(move_group, logger, waypoints, "post-pick")) {
            return false; //Falló
        }

        // Definir posición pre-place
        pose.position.x = place_pose.position.x + D_PLACE*i;
        pose.position.y = place_pose.position.y;
        pose.position.z = place_pose.position.z + d_prepick; // z + altura
        i++;

        if (!mover(move_group, logger, pose, gripper_frame, "pre_place")) {
            return false; //Falló
        }

        // Planificacion cartesian path para acercarse al objetivo
        move_group.setStartStateToCurrentState();

        // Vector con los puntos de la trayectoria
        waypoints.clear();
        waypoints.push_back(pose);
        pose.position.z -= d_prepick/2;
        waypoints.push_back(pose);
        pose.position.z -= d_prepick/2;
        waypoints.push_back(pose);

        if (!moverCart(move_group, logger, waypoints, "place")) {
            return false; //Falló
        }

        // Abrir gripper
        gripperControl(my_dashboard, "abrir.urp");

        // Separar objeto del robot
        move_group.detachObject(pair.first.c_str());

        // Consigo la escena actual
        planning_scene_monitor->requestPlanningSceneState();
        planning_scene = planning_scene_monitor->getPlanningScene();

        // Indico que parte pueden estar en colision
        planning_scene->getAllowedCollisionMatrixNonConst().setEntry("gripper", pair.first.c_str(), true);

        // Enviar la escena modificada al entorno de planificación
        // moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene->getPlanningSceneMsg(planning_scene_msg);
        planning_scene_msg.is_diff = true;

        planning_scene_pub->publish(planning_scene_msg);

        // Planificacion cartesian path para alejarse del objeto
        move_group.setStartStateToCurrentState();
        // Vector con los puntos de la trayectoria
        waypoints.clear();
        waypoints.push_back(pose);
        pose.position.z += d_prepick/2;
        waypoints.push_back(pose);
        pose.position.z += d_prepick/2;
        waypoints.push_back(pose);

        if (!moverCart(move_group, logger, waypoints, "post-place")) {
            return false; //Falló
        }

        // Cerrar gripper
        gripperControl(my_dashboard, "cerrar.urp");
    }
    return true;
}

int main(int argc, char** argv)
{
    //Inicialización con configuración del robot
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
 
    const std::string robot_ip = "212.128.172.172";  // ip del robot

    // Crear interface para el grupo de planificación
    auto node = rclcpp::Node::make_shared("celda_move_to_pose", node_options);
    
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 1);

    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Espera a que se conecte el subscriber
    
    static const std::string PLANNING_GROUP = "ur_arm";
    static const std::string GRIPPER_FRAME = "gripper_tip";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    move_group.setEndEffectorLink(GRIPPER_FRAME); //Para que computeCartesianPath use el gripper como referencia

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

    std::string vision_share = ament_index_cpp::get_package_share_directory("vision");

    // Construye la ruta al fichero objetos.yaml
    std::string objetos_yaml = vision_share + "/objetos.yaml";

    RCLCPP_INFO(node->get_logger(), "Cargando objetos desde: %s", objetos_yaml.c_str());

    // Carga el YAML
    YAML::Node escena = YAML::LoadFile(objetos_yaml);
    
    tf2_ros::StaticTransformBroadcaster broadcaster(node);
    
    double aruco_roll = escena["aruco"]["orientacion"]["roll"].as<double>();
    double aruco_pitch = escena["aruco"]["orientacion"]["pitch"].as<double>();
    double aruco_yaw = escena["aruco"]["orientacion"]["yaw"].as<double>();
    RCLCPP_INFO(node->get_logger(), "Orientación del aruco: roll=%.2f, pitch=%.2f, yaw=%.2f", aruco_roll, aruco_pitch, aruco_yaw);
    double aruco_x = escena["aruco"]["posicion"]["x"].as<double>();
    double aruco_y = escena["aruco"]["posicion"]["y"].as<double>(); 
    double aruco_z = escena["aruco"]["posicion"]["z"].as<double>();
    RCLCPP_INFO(node->get_logger(), "Posición del aruco: x=%.2f, y=%.2f, z=%.2f", aruco_x, aruco_y, aruco_z);

    // 2) Crea el broadcaster de TF2 **una sola vez**

    // 3) Monta el TransformStamped
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node->now();
    t.header.frame_id = "world";          // o el frame padre que uses
    t.child_frame_id = "aruco";   // el nombre de tu nuevo frame

    // posición
    t.transform.translation.x = aruco_x;
    t.transform.translation.y = aruco_y;
    t.transform.translation.z = aruco_z;

    // convierto RPY → quaternion
    tf2::Quaternion q;
    q.setRPY(aruco_roll, aruco_pitch, aruco_yaw);
    t.transform.rotation = tf2::toMsg(q);

    // 4) Publica el transform **una vez** (es estático)
    broadcaster.sendTransform(t);

    // Define el cilindro
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.025;  // altura en metros
    primitive.dimensions[1] = 0.015; // radio en metros

    // Posición del cilindro
    geometry_msgs::msg::Pose pose;

    int i = 0;
    for (auto objeto : escena["objetos"]) {
        double x = objeto["posicion"]["x"].as<double>();
        double y = objeto["posicion"]["y"].as<double>();
        double z = objeto["posicion"]["z"].as<double>();

        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z + primitive.dimensions[0]/2;  // z + altura/2 para que se apoye sobre la superficie
        addObjeto("pick_target" + i, "aruco", primitive, pose);
        i++;
    }
    
    // pose.position.x = 0.1;
    // pose.position.y = -0.1;
    // pose.position.z = primitive.dimensions[0]/2;  // z + altura/2 para que se apoye sobre la superficie
    // addObjeto("pick_target1", "aruco", primitive, pose);

    // pose.position.x = 0.4;
    // pose.position.y = -0.3;
    // pose.position.z = primitive.dimensions[0]/2;  // z + altura/2 para que se apoye sobre la superficie
    // addObjeto("pick_target1", primitive, pose);

    // pose.position.x = 0.2;
    // pose.position.y = -0.3;
    // addObjeto("pick_target2", primitive, pose);

    // pose.position.x = 0.4;
    // pose.position.y = -0.1;
    // addObjeto("pick_target3", primitive, pose);

    // pose.position.x = 0.2;
    // pose.position.y = -0.1;
    // addObjeto("pick_target4", primitive, pose);

    //Esperamos a que se actualice la escena
    rclcpp::sleep_for(std::chrono::seconds(1));

    geometry_msgs::msg::Pose place_pose;
    place_pose.position.x = 0.0;
    place_pose.position.y = -0.3;
    place_pose.position.z = primitive.dimensions[0]/2;  // z + altura/2 para que se apoye sobre la superficie
    pick_and_place(move_group, node->get_logger(), planning_scene_pub, planning_scene_monitor, GRIPPER_FRAME, 0.05, robot_ip, place_pose);

    // // RCLCPP_INFO(node->get_logger(), "Eliminar objeto");
    // // std::vector<std::string> objetos_eliminar;
    // // objetos_eliminar.push_back(pair.first.c_str());
    // // planning_scene_interface.removeCollisionObjects(objetos_eliminar);

    // Shutdown ROS
    rclcpp::shutdown();
    // Desconexión del server dashboard
    // my_dashboard->disconnect();
    return 0;
}
