#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

void addObjeto(shape_msgs::msg::SolidPrimitive primitive, geometry_msgs::msg::Pose pose)
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::msg::CollisionObject objeto;
  objeto.id = "pick_target";
  objeto.header.frame_id = "world";  // O usa "base_link" según tu escena

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

int main(int argc, char** argv)
{
  //Inicialización con configuración del robot
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  
  // Crear interface para el grupo de planificación
  auto node = rclcpp::Node::make_shared("ur3e_move_to_pose", node_options);
  
  // Crear un planning group con los 2 grupos para que al calcular la trayectoria también tenga en cuenta el eef ???
  static const std::string PLANNING_GROUP = "ur_onrobot_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Crear publicador para el gripper
  auto gripper_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/finger_width_controller/commands", 10);

  // Mensaje para abrir gripper
  std_msgs::msg::Float64MultiArray open_msg;
  open_msg.data = {0.11};

  // Mensaje para cerrar gripper
  std_msgs::msg::Float64MultiArray close_msg;
  close_msg.data = {0.0};

  // Define el cilindro
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = 0.1;  // altura en metros
  primitive.dimensions[1] = 0.02; // radio en metros

  // Posicion del cilindro
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.4;
  pose.position.y = -0.4;
  pose.position.z = 0.5 + primitive.dimensions[0]/2;  // z + altura/2 para que se apoye sobre la superficie
  
  // Añado el cilindro
  addObjeto(primitive, pose);

  //Esperamos a que se actualice la escena
  rclcpp::sleep_for(std::chrono::seconds(1));

  // Definir la pose pre-pick
  float d_prepick = 0.1;
  pose.position.z += d_prepick; //Posición del objeto 0.1 más arriba
  pose.orientation.x = 1; //Con orientación hacia abajo
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 0;
  
  // move_group.setPoseTarget(target_pose, "gripper_tcp");

  //move_group.setNamedTarget("home");
  move_group.setPoseTarget(pose, move_group.getEndEffectorLink().c_str());
  // move_group.setGoalOrientationTolerance(3.14);  // Radianes: permite cualquier orientación
  // move_group.setPlanningTime(10.0); // Aumenta el tiempo de planificación

  // Planificar
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // Si falla la planificación termina
  if (!success) {
    RCLCPP_ERROR(node->get_logger(), "Falló la planificación pre-pick.");
    rclcpp::shutdown();
    return 1;
  }

  // Si no falla la planificación sigue el código
  RCLCPP_INFO(node->get_logger(), "Planificación pre-pick exitosa, ejecutando.");
  move_group.execute(plan);

  // Abrir gripper
  gripper_pub->publish(open_msg);
  rclcpp::sleep_for(std::chrono::milliseconds(1000)); // Esperar a que termine de abrirse

  // Planificacion cartesian path para acercarse al objeto
  // Vector con los puntos de la trayectoria
  std::vector<geometry_msgs::msg::Pose> waypoints;
  pose.position.z -= d_prepick/2;
  waypoints.push_back(pose);
  pose.position.z -= d_prepick/2;
  waypoints.push_back(pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0; // debería ser 0.3 al probar con hardware real, es el error que puede haber entre 2 puntos de la trayectoria
  const double eef_step = 0.005; // cuanta distancia hay entre cada punto de la trayectoria
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(node->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // Si únicamente se ha podido calcular menos del 90% de la trayectoria, termina
  if (fraction < 0.9) {
    RCLCPP_ERROR(node->get_logger(), "Falló la planificación pick.");
    return 2;
  }
  
  RCLCPP_INFO(node->get_logger(), "Planificación pick exitosa, ejecutando.");
  move_group.execute(trajectory);

  RCLCPP_INFO(node->get_logger(), "Unir objeto al movimiento del robot");
  // Indicar qué links están en contacto con el objeto para evitar errores por colision
  std::vector<std::string> touch_links;
  touch_links.push_back("left_finger_tip");
  touch_links.push_back("left_inner_finger");
  touch_links.push_back("left_inner_knuckle");
  touch_links.push_back("left_outer_knuckle");
  
  touch_links.push_back("right_finger_tip");
  touch_links.push_back("right_inner_finger");
  touch_links.push_back("right_inner_knuckle");
  touch_links.push_back("right_outer_knuckle");

  move_group.attachObject("pick_target", move_group.getEndEffectorLink().c_str(), touch_links);

  // Cerrar gripper
  gripper_pub->publish(close_msg);
  rclcpp::sleep_for(std::chrono::milliseconds(1000)); // Esperar a que termine de abrirse
  
  // Planificacion cartesian path para alejarse hacia arriba
  // Vaciamos el vector de punto
  waypoints.clear();
  pose.position.z += d_prepick/2;
  waypoints.push_back(pose);
  pose.position.z += d_prepick/2;
  waypoints.push_back(pose);

  fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(node->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // Si únicamente se ha podido calcular menos del 90% de la trayectoria, termina
  if (fraction < 0.9) {
    RCLCPP_ERROR(node->get_logger(), "Falló la planificación post-pick.");
    return 2;
  }
  
  RCLCPP_INFO(node->get_logger(), "Planificación post-pick exitosa, ejecutando.");
  move_group.execute(trajectory);

  // Definir posición pre-place
  pose.position.x = 0.4;
  pose.position.y = 0.4;
  //Mantenemos la misma posición en altura y orientación hacia abajo

  move_group.setPoseTarget(pose, move_group.getEndEffectorLink().c_str());

  // Planificar
  // moveit::planning_interface::MoveGroupInterface::Plan plan;
  success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // Si falla la planificación termina
  if (!success) {
    RCLCPP_ERROR(node->get_logger(), "Falló la planificación pre-place.");
    rclcpp::shutdown();
    return 1;
  }

  // Si no falla la planificación sigue el código
  RCLCPP_INFO(node->get_logger(), "Planificación pre-place exitosa, ejecutando.");
  move_group.execute(plan);

  // Planificacion cartesian path para acercarse al objeto
  // Vector con los puntos de la trayectoria
  waypoints.clear();
  pose.position.z -= d_prepick/2;
  waypoints.push_back(pose);
  // pose.position.z -= d_prepick/2;
  // waypoints.push_back(pose);

  fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(node->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // Si únicamente se ha podido calcular menos del 90% de la trayectoria, termina
  if (fraction < 0.9) {
    RCLCPP_ERROR(node->get_logger(), "Falló la planificación place.");
    return 2;
  }
  
  RCLCPP_INFO(node->get_logger(), "Planificación place exitosa, ejecutando.");
  move_group.execute(trajectory);

  // Abrir gripper
  gripper_pub->publish(open_msg);
  rclcpp::sleep_for(std::chrono::milliseconds(5000)); // Esperar a que termine de abrirse

  // Separar objeto del robot
  move_group.detachObject("pick_target");

  // Planificacion cartesian path para alejarse del objeto
  // Vector con los puntos de la trayectoria
  waypoints.clear();
  pose.position.z += d_prepick/2;
  waypoints.push_back(pose);
  // pose.position.z += d_prepick/2;
  // waypoints.push_back(pose);

  fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(node->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // Si únicamente se ha podido calcular menos del 90% de la trayectoria, termina
  if (fraction < 0.9) {
    RCLCPP_ERROR(node->get_logger(), "Falló la planificación post-place.");
    return 2;
  }
  
  RCLCPP_INFO(node->get_logger(), "Planificación post-place exitosa, ejecutando.");
  move_group.execute(trajectory);

  // // Cerrar gripper
  // gripper_pub->publish(close_msg);
  // rclcpp::sleep_for(std::chrono::milliseconds(1000)); // Esperar a que termine de abrirse

  move_group.setNamedTarget("home");

  // Planificar
  success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // Si falla la planificación termina
  if (!success) {
    RCLCPP_ERROR(node->get_logger(), "Falló la planificación home.");
    rclcpp::shutdown();
    return 1;
  }

  // Si no falla la planificación sigue el código
  RCLCPP_INFO(node->get_logger(), "Planificación home exitosa, ejecutando.");
  move_group.execute(plan);

  // RCLCPP_INFO(node->get_logger(), "Eliminar objeto");
  // std::vector<std::string> objetos_eliminar;
  // objetos_eliminar.push_back("pick_target");
  // planning_scene_interface.removeCollisionObjects(objetos_eliminar);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
