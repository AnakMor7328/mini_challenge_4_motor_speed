#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <math.h>

// Pines L298N y encoder
#define PIN_PWM 14      // Salida PWM al driver
#define PIN_IN1 26      // Control dirección A
#define PIN_IN2 15      // Control dirección B
#define ENC_A 19        // Encoder canal A (interrupción)
#define ENC_B 5         // Encoder canal B

// PWM ESP32
#define PWM_CHANNEL 0
#define PWM_FREQ 980     // Frecuencia estándar para DC motors
#define PWM_RESOLUTION 8 // 8 bits = 0-255
#define PWM_MAX 255

// Parámetros del encoder
#define ENCODER_PPR 500.0f  // Pulsos por revolución (ajustar según encoder)

// Objetos micro-ROS
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_subscription_t subscriber;      // Para /cmd_pwm
rcl_publisher_t speed_publisher;    // Para /motor_speed_y
rclc_executor_t executor;
std_msgs__msg__Float32 msg;         // Mensaje para cmd_pwm
std_msgs__msg__Float32 speed_msg;   // Mensaje para velocidad

// Variables del encoder
volatile long encoder_count = 0;    // Volátil por la ISR
volatile int encoder_dir = 0;        // Dirección estimada
long last_encoder_count = 0;         // Para calcular delta
float motor_rpm = 0.0f;

// Máquina de estados para reconexión automática
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Macros de utilidad
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis(); } \
  if (uxr_millis() - init > (MS)) { X; init = uxr_millis(); } \
} while (0)

// Callback del comando PWM
void cmd_pwm_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * m = (const std_msgs__msg__Float32 *)msgin;
  float cmd = m->data;

  // Saturar al rango válido [-1, 1]
  if (cmd > 1.0f) cmd = 1.0f;
  if (cmd < -1.0f) cmd = -1.0f;

  int duty = (int)(fabsf(cmd) * PWM_MAX);

  // Control de dirección según signo
  if (cmd > 0.0f) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
  else if (cmd < 0.0f) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  }
  else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    duty = 0;
  }

  ledcWrite(PWM_CHANNEL, duty);
}

// Crear entidades micro-ROS
bool create_entities()
{
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/cmd_pwm"
  ));
  RCCHECK(rclc_publisher_init_default(
    &speed_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/motor_speed_y"
  ));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &cmd_pwm_callback,
    ON_NEW_DATA
  ));

  return true;
}

// Destruir entidades
void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&speed_publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ISR del encoder (se ejecuta en cada cambio de canal A)
void IRAM_ATTR encoder_isr()
{
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);

  // Decodificación simple de cuadratura
  if (a == b) {
    encoder_count++;
    encoder_dir = 1;
  } else {
    encoder_count--;
    encoder_dir = -1;
  }
}

// Configuración inicial
void setup()
{
  Serial.begin(115200);
  Serial.println("=== INICIO DEL PROGRAMA ===");
  
  set_microros_transports();

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoder_isr, CHANGE);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_PWM, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);

  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);

  msg.data = 0.0f;
  speed_msg.data = 0.0f;
  state = WAITING_AGENT;
}

// Bucle principal
void loop()
{
  switch (state) {

    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
          ? AGENT_AVAILABLE
          : WAITING_AGENT;
      );
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
          ? AGENT_CONNECTED
          : AGENT_DISCONNECTED;
      );

      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        
        // Calcular y publicar velocidad cada 100ms
        EXECUTE_EVERY_N_MS(100,
          long current_count = encoder_count;
          long delta_counts = current_count - last_encoder_count;
          last_encoder_count = current_count;

          float dt = 0.1f;  // 100ms en segundos
          float revs = delta_counts / ENCODER_PPR;
          motor_rpm = (revs / dt) * 60.0f;  // Convertir a RPM

          speed_msg.data = motor_rpm;
          RCSOFTCHECK(rcl_publish(&speed_publisher, &speed_msg, NULL));

          // Debug por serial
          Serial.print("Count: ");
          Serial.print(current_count);
          Serial.print(" | RPM: ");
          Serial.println(motor_rpm);
        );
        
        // Monitoreo menos frecuente
        EXECUTE_EVERY_N_MS(500,
          Serial.print("Encoder count: ");
          Serial.print(encoder_count);
          Serial.print(" | Dir: ");
          Serial.println(encoder_dir);
        );
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      // Detener motor por seguridad
      digitalWrite(PIN_IN1, LOW);
      digitalWrite(PIN_IN2, LOW);
      ledcWrite(PWM_CHANNEL, 0);
      state = WAITING_AGENT;
      break;

    default:
      break;
  }
}