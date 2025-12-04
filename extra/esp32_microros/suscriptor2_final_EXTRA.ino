// ============================================================================
//  ESP32 – micro-ROS Control Node
//  Control por gestos + evitación de obstáculos + seguridad
// ============================================================================

#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================
#define LED_FORWARD   4
#define LED_BACKWARD  5
#define LED_LEFT      18
#define LED_RIGHT     19
#define LED_STOP      21

#define EMERGENCY_BTN 22     // Botón emergencia (LOW = presionado)

// ============================================================================
// micro-ROS VARIABLES
// ============================================================================
rcl_subscription_t gesture_sub;
rcl_subscription_t dist_sub;
rcl_publisher_t cmd_vel_pub;

std_msgs__msg__String gesture_msg;
std_msgs__msg__Float32MultiArray dist_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

char gesture_buffer[32];

// Variables de distancia
float dist_left = 5.0f;
float dist_center = 5.0f;
float dist_right = 5.0f;

// Estados internos
unsigned long last_gesture_time = 0;
const unsigned long TIMEOUT_MS = 2500;

bool emergency_mode = false;

// ============================================================================
// MANEJO DE ERRORES micro-ROS
// ============================================================================
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(rc, __LINE__); }
#define RCSOFTCHECK(fn) fn;

void error_loop(rcl_ret_t rc, int line)
{
  Serial.printf("[FATAL] micro-ROS error %d at line %d\n", rc, line);

  pinMode(LED_STOP, OUTPUT);
  while (1)
  {
    digitalWrite(LED_STOP, LOW);
    delay(200);
    digitalWrite(LED_STOP, HIGH);
    delay(200);
  }
}

// ============================================================================
// UTILIDADES LED
// ============================================================================
void all_leds_off()
{
  digitalWrite(LED_FORWARD, HIGH);
  digitalWrite(LED_BACKWARD, HIGH);
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_RIGHT, HIGH);
  digitalWrite(LED_STOP, HIGH);
}

void publish_stop()
{
  cmd_vel_msg.linear.x = 0.0f;
  cmd_vel_msg.angular.z = 0.0f;

  all_leds_off();
  digitalWrite(LED_STOP, LOW);

  RCSOFTCHECK(rcl_publish(&cmd_vel_pub, &cmd_vel_msg, NULL));
  Serial.println("[STOP] STOP enviado al robot.");
}

// ============================================================================
// CALLBACK: /gesture_command
// ============================================================================
void gesture_callback(const void *msgin)
{
  if (emergency_mode)
  {
    publish_stop();
    return;
  }

  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  String gesture = String(msg->data.data);

  Serial.print("[GESTURE] ");
  Serial.println(gesture);

  last_gesture_time = millis();

  all_leds_off();
  float lin = 0.0f, ang = 0.0f;

  if (gesture == "forward")
  {
    lin = 0.25f;
    digitalWrite(LED_FORWARD, LOW);
  }
  else if (gesture == "backward")
  {
    lin = -0.25f;
    digitalWrite(LED_BACKWARD, LOW);
  }
  else if (gesture == "left")
  {
    ang = 0.6f;
    digitalWrite(LED_LEFT, LOW);
  }
  else if (gesture == "right")
  {
    ang = -0.6f;
    digitalWrite(LED_RIGHT, LOW);
  }
  else if (gesture == "stop")
  {
    publish_stop();
    return;
  }
  else
  {
    Serial.println("[WARN] Gesto desconocido → STOP");
    publish_stop();
    return;
  }

  cmd_vel_msg.linear.x = lin;
  cmd_vel_msg.angular.z = ang;

  RCSOFTCHECK(rcl_publish(&cmd_vel_pub, &cmd_vel_msg, NULL));
}

// ============================================================================
// CALLBACK: /distance_zones  → (EXTRA)
// ============================================================================
void distance_callback(const void *msgin)
{
  if (emergency_mode) return;

  const std_msgs__msg__Float32MultiArray *msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;

  dist_left   = msg->data.data[0];
  dist_center = msg->data.data[1];
  dist_right  = msg->data.data[2];

  Serial.printf("[DIST] L=%.2f  C=%.2f  R=%.2f\n", dist_left, dist_center, dist_right);

  // --- Reglas de seguridad automáticas ---

  // 1. Centro extremadamente cerca → STOP
  if (dist_center < 0.50f)
  {
    Serial.println("[SAFETY] Objeto muy cerca en el centro → STOP");
    publish_stop();
    return;
  }

  // 2. Reducir velocidad si hay un obstáculo moderado al frente
  if (cmd_vel_msg.linear.x > 0 && dist_center < 1.0f)
  {
    cmd_vel_msg.linear.x *= 0.5f;
    Serial.println("[SAFETY] Objeto a <1m → reduciendo velocidad");
  }

  // 3. Esquivar laterales
  if (dist_left < 0.7f)
  {
    Serial.println("[SAFETY] Objeto izquierda → giro derecha");
    cmd_vel_msg.angular.z = -0.4f;
  }
  else if (dist_right < 0.7f)
  {
    Serial.println("[SAFETY] Objeto derecha → giro izquierda");
    cmd_vel_msg.angular.z = 0.4f;
  }

  RCSOFTCHECK(rcl_publish(&cmd_vel_pub, &cmd_vel_msg, NULL));
}

// ============================================================================
// SETUP
// ============================================================================
void setup()
{
  Serial.begin(115200);
  delay(2000);

  pinMode(LED_FORWARD, OUTPUT);
  pinMode(LED_BACKWARD, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_STOP, OUTPUT);
  pinMode(EMERGENCY_BTN, INPUT_PULLUP);

  all_leds_off();
  digitalWrite(LED_STOP, LOW);

  // Transporte micro-ROS USB/Serial
  set_microros_transports();

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_control_node", "", &support));

  // QoS BEST-EFFORT (ROS 2 → micro-ROS)
  rmw_qos_profile_t qos_micro = rmw_qos_profile_sensor_data;
  qos_micro.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  qos_micro.depth = 1;

  // Estructura de String
  gesture_msg.data.data = gesture_buffer;
  gesture_msg.data.size = 0;
  gesture_msg.data.capacity = sizeof(gesture_buffer);

  // Suscripciones
  RCCHECK(rclc_subscription_init(
    &gesture_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/gesture_command", &qos_micro));

  RCCHECK(rclc_subscription_init(
    &dist_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/distance_zones", &qos_micro));

  // Publicador /cmd_vel
  RCCHECK(rclc_publisher_init_default(
    &cmd_vel_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &gesture_sub, &gesture_msg, &gesture_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &dist_sub, &dist_msg, &distance_callback, ON_NEW_DATA));

  Serial.println("[INFO] ESP32 micro-ROS listo (Arduino IDE).");
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================
void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Modo emergencia
  if (digitalRead(EMERGENCY_BTN) == LOW && !emergency_mode)
  {
    emergency_mode = true;
    Serial.println("[ALERT] Emergencia activada.");
    publish_stop();
  }

  // Timeout por falta de gestos
  if (!emergency_mode && (millis() - last_gesture_time) > TIMEOUT_MS)
  {
    Serial.println("[TIMEOUT] Sin gestos → STOP.");
    publish_stop();
  }

  delay(5);
}
