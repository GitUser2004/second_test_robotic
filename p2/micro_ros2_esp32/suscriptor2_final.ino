#include <micro_ros_arduino.h>
#include <Arduino.h>
//#include <micro_ros_platformio.h>  // ← Correcto para PlatformIO

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>


#define LED_FORWARD   4      // LED avance
#define LED_BACKWARD  5      // LED retroceso
#define LED_LEFT      18      // LED giro izquierda
#define LED_RIGHT     19     // LED giro derecha
#define LED_STOP      21     // LED detener

#define EMERGENCY_BTN 22     // Botón de emergencia (activo en LOW)


rcl_subscription_t gesture_sub;
rcl_publisher_t   cmd_vel_pub;

std_msgs__msg__String      gesture_msg;
geometry_msgs__msg__Twist  cmd_vel_msg;

rclc_executor_t  executor;
rcl_node_t       node;
rcl_allocator_t  allocator;
rclc_support_t   support;

// Buffer estático para el String recibido
// (evitamos malloc y posibles cuelgues)
char gesture_buffer[32];


unsigned long last_gesture_time = 0;
const unsigned long TIMEOUT_MS  = 2000;   // 2 segundos sin gestos = STOP

bool emergency_mode       = false;
bool stopped_by_timeout   = false;


#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(rc, __LINE__); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// Bucle de error: parpadea LED_STOP para indicar fallo crítico
void error_loop(rcl_ret_t rc, int line)
{
  Serial.print("[ERROR] rcl/rclc code: ");
  Serial.print(rc);
  Serial.print(" at line ");
  Serial.println(line);

  pinMode(LED_STOP, OUTPUT);
  while (1)
  {
    digitalWrite(LED_STOP, LOW);
    delay(200);
    digitalWrite(LED_STOP, HIGH);
    delay(200);
  }
}

void all_leds_off()
{
  digitalWrite(LED_FORWARD, HIGH);
  digitalWrite(LED_BACKWARD, HIGH);
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_RIGHT, HIGH);
  digitalWrite(LED_STOP, HIGH);
}

// Publicar STOP solo si no estaba ya en STOP
void publish_stop()
{
  // Si ya está en STOP, no repitas
  if (cmd_vel_msg.linear.x == 0.0 && cmd_vel_msg.angular.z == 0.0 && stopped_by_timeout)
    return;

  cmd_vel_msg.linear.x  = 0.0f;
  cmd_vel_msg.angular.z = 0.0f;
  RCSOFTCHECK( rcl_publish(&cmd_vel_pub, &cmd_vel_msg, NULL) );

  all_leds_off();
  digitalWrite(LED_STOP, LOW);

  stopped_by_timeout = true;
  Serial.println("[INFO] STOP publicado");
}


void gesture_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;

  // En modo emergencia, se ignoran los gestos pero se mantiene STOP
  if (emergency_mode)
  {
    publish_stop();
    return;
  }

  // Convertir a String de Arduino para comparar más fácil
  String gesture = String(msg->data.data);

  Serial.print("[INFO] Gesto recibido: ");
  Serial.println(gesture);

  last_gesture_time   = millis();   // reinicia el timeout
  stopped_by_timeout  = false;      // hay actividad de nuevo

  all_leds_off();  // apagar LEDs previos

  // ==========================
  // CLASIFICACIÓN DEL GESTO
  // ==========================
  if (gesture == "forward")
  {
    cmd_vel_msg.linear.x  = 0.2f;
    cmd_vel_msg.angular.z = 0.0f;
    digitalWrite(LED_FORWARD, LOW);
  }
  else if (gesture == "backward")
  {
    cmd_vel_msg.linear.x  = -0.2f;
    cmd_vel_msg.angular.z = 0.0f;
    digitalWrite(LED_BACKWARD, LOW);
  }
  else if (gesture == "left")
  {
    cmd_vel_msg.linear.x  = 0.0f;
    cmd_vel_msg.angular.z = 0.5f;
    digitalWrite(LED_LEFT, LOW);
  }
  else if (gesture == "right")
  {
    cmd_vel_msg.linear.x  = 0.0f;
    cmd_vel_msg.angular.z = -0.5f;
    digitalWrite(LED_RIGHT, LOW);
  }
  else if (gesture == "stop")
  {
    // stop explícito desde la percepción
    publish_stop();
    return;
  }
  else
  {
    // gesto desconocido → STOP por seguridad
    Serial.println("[WARN] Gesto desconocido, forzando STOP");
    publish_stop();
    return;
  }

  // Límite de seguridad de velocidad lineal
  if (cmd_vel_msg.linear.x > 0.3f)  cmd_vel_msg.linear.x  = 0.3f;
  if (cmd_vel_msg.linear.x < -0.3f) cmd_vel_msg.linear.x  = -0.3f;

  RCSOFTCHECK( rcl_publish(&cmd_vel_pub, &cmd_vel_msg, NULL) );
}

// ==========================
// SETUP
// ==========================
void setup()
{
  // Serial para depuración
  Serial.begin(115200);
  delay(2000);   // dar tiempo a que se abra el puerto
  Serial.println("\n[INFO] Iniciando ESP32 micro-ROS...");

  // Configurar pines
  pinMode(LED_FORWARD,  OUTPUT);
  pinMode(LED_BACKWARD, OUTPUT);
  pinMode(LED_LEFT,     OUTPUT);
  pinMode(LED_RIGHT,    OUTPUT);
  pinMode(LED_STOP,     OUTPUT);
  pinMode(EMERGENCY_BTN, INPUT_PULLUP);

  all_leds_off();
  digitalWrite(LED_STOP, LOW);

  // Configurar transporte micro-ROS (por serie en tu caso)
  set_microros_transports();

  allocator = rcl_get_default_allocator();

  // Inicializar soporte micro-ROS
  RCCHECK( rclc_support_init(&support, 0, NULL, &allocator) );

  // Crear nodo
  RCCHECK( rclc_node_init_default(
              &node,
              "esp32_control_node",
              "",
              &support) );

  // Inicializar buffer del mensaje String (suscriptor)
  gesture_msg.data.data     = gesture_buffer;
  gesture_msg.data.capacity = sizeof(gesture_buffer);
  gesture_msg.data.size     = 0;

  // QoS compatible: BEST_EFFORT, parecido a sensor_data
  rmw_qos_profile_t qos_gesture = rmw_qos_profile_sensor_data;
  qos_gesture.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  qos_gesture.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos_gesture.depth       = 1;

  // Crear suscriptor de /gesture_command
  RCCHECK( rclc_subscription_init(
              &gesture_sub,
              &node,
              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
              "/gesture_command",
              &qos_gesture) );

  // Crear publicador de /cmd_vel
  RCCHECK( rclc_publisher_init_default(
              &cmd_vel_pub,
              &node,
              ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
              "/cmd_vel") );

  // Inicializar executor
  RCCHECK( rclc_executor_init(&executor, &support.context, 1, &allocator) );
  RCCHECK( rclc_executor_add_subscription(
              &executor,
              &gesture_sub,
              &gesture_msg,
              &gesture_callback,
              ON_NEW_DATA) );

  last_gesture_time  = millis();
  stopped_by_timeout = false;

  Serial.println("[INFO] ESP32 micro-ROS listo.");
}


void loop()
{
  // Atiende callbacks de micro-ROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // --------------------------
  // BOTÓN DE EMERGENCIA
  // --------------------------
  bool emergency_pressed = (digitalRead(EMERGENCY_BTN) == LOW);

  if (emergency_pressed && !emergency_mode)
  {
    emergency_mode = true;
    Serial.println("[ALERT] Modo EMERGENCIA activado, publicando STOP.");
    publish_stop();
  }

  // En emergencia: solo mantener STOP, pero seguir atendiendo executor
  if (emergency_mode)
  {
    delay(10);
    return;
  }


  if ( (millis() - last_gesture_time) > TIMEOUT_MS )
  {
    if (!stopped_by_timeout)
    {
      Serial.println("[WARN] Timeout de gestos, enviando STOP.");
      publish_stop();
    }
  }

  delay(5);  // pequeño respiro para evitar saturar la CPU
}
