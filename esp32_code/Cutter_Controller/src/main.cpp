#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/empty.h> // Empty for watchdog
#include <driver/twai.h>

#define CAN_TX_PIN GPIO_NUM_21
#define CAN_RX_PIN GPIO_NUM_22

// Autoro Single ESC (Cutter VESC ID)
#define VESC_CUTTER_ID 3

#include <std_msgs/msg/float32.h>

rcl_subscription_t subscriber;
rcl_subscription_t heartbeat_sub;           // Heartbeat watchdog subscription
rcl_publisher_t status_pub;                 //
rcl_publisher_t rpm_pub;                    //
rcl_publisher_t current_pub;                //
std_msgs__msg__Int32 msg_speed;
std_msgs__msg__Empty heartbeat_msg;         // Heartbeat message
std_msgs__msg__Int32 status_msg;            //
std_msgs__msg__Int32 rpm_msg;               //
std_msgs__msg__Float32 current_msg;         //
rclc_executor_t executor;

// The cutter is disabled when the main computer stops sending its heartbeat.
// Cutter watchdog to detect computer failure
volatile unsigned long last_heartbeat_time = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 500;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

int32_t expected_erpm = 0;                  //
int32_t actual_rpm = 0;                     //
float actual_current = 0.0;                 //
bool cutter_blocked = false;                // Overload / blocked flag
unsigned long last_status_publish = 0;

void init_twai() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
    }
}

// Set the cutting blade RPM (ERPM)
void set_cutter_rpm(int32_t erpm) {
    twai_message_t message;
    message.identifier = (0x03 << 8) | VESC_CUTTER_ID; // CAN_PACKET_SET_RPM
    message.extd = 1;
    message.data_length_code = 4;

    message.data[0] = (erpm >> 24) & 0xFF;
    message.data[1] = (erpm >> 16) & 0xFF;
    message.data[2] = (erpm >> 8) & 0xFF;
    message.data[3] = erpm & 0xFF;

    twai_transmit(&message, pdMS_TO_TICKS(10));
}

// Read VESC status frames and publish cutter health telemetry once per second.
void read_vesc_telemetry() {
    twai_message_t rx_msg;
    // Read CAN frames
    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        uint32_t id = rx_msg.identifier;
        uint8_t cmd = (id >> 8) & 0xFF;
        uint8_t sender_id = id & 0xFF;

        if (sender_id == VESC_CUTTER_ID) {
            if (cmd == 0x09) { // CAN_PACKET_STATUS_1 (RPM, Current, Duty)
                int32_t rpm = (rx_msg.data[0] << 24) | (rx_msg.data[1] << 16) | (rx_msg.data[2] << 8) | rx_msg.data[3];
                float current = ((int16_t)((rx_msg.data[4] << 8) | rx_msg.data[5])) / 10.0;

                // Store values for measurement
                actual_rpm = rpm / 10; // VESC reports ERPM; divide by 10 (for a 10-pole motor) to get real RPM
                actual_current = current;

                // BLOCKING TEST: If the blade is supposed to run (ERPM > 1000),
                // but the actual RPM is below 100 (blade is idle/stuck)
                // and the current is simultaneously very high (> 15.0A), then the cutter is blocked!
                if (expected_erpm > 1000 && abs(rpm) < 100 && current > 15.0) {
                    cutter_blocked = true;
                    set_cutter_rpm(0); // Immediate emergency stop to protect the motor and drivetrain
                    actual_rpm = 0;
                    actual_current = 0.0;
                }
            }
        }
    }

    // Publish cutter status, RPM, and current to ROS 2 once per second
    unsigned long now = millis();
    if (now - last_status_publish >= 1000) {
        last_status_publish = now;

        // Publish status, RPM, and current
        status_msg.data = cutter_blocked ? 2 : (expected_erpm > 0 ? 1 : 0);
        rpm_msg.data = actual_rpm;
        current_msg.data = actual_current;

        rcl_publish(&status_pub, &status_msg, NULL);
        rcl_publish(&rpm_pub, &rpm_msg, NULL);
        rcl_publish(&current_pub, &current_msg, NULL);
    }
}

// Heartbeat watchdog callback for the cutter
void heartbeat_callback(const void * msvgin) {
    last_heartbeat_time = millis();
}

void subscription_callback(const void * msvgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msvgin;
    expected_erpm = msg->data;

    // If we receive a speed command of 0 (stop the cutter),
    // we can re-arm the system and clear the blocked condition.
    if (expected_erpm == 0) {
        cutter_blocked = false;
    }

    if (!cutter_blocked) {
        set_cutter_rpm(expected_erpm);
    } else {
        set_cutter_rpm(0); // Stay off until the system is reset by the user (by sending 0)
    }
}

void setup() {
    // Initialize the optional BNO085 safety sensor over its dedicated I2C pins.
    Wire.begin(BNO_SDA_PIN, BNO_SCL_PIN);
    if (myIMU.begin() == false) {
        Serial.println("Warning: BNO085 IMU not found! Check wiring on GPIO 25/26.");
        imu_present = false;
    } else {
        // Enable rotation vector (Roll, Pitch, Yaw) with 50 ms polling speed (20 Hz)
        myIMU.enableRotationVector(50);
        Serial.println("BNO085 IMU initialized and running with stable sensor fusion!");
        imu_present = true;
    }

    init_twai();
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "cutter_controller", "", &support);

    // Initialize ROS 2 subscription on /cutter/speed
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/cutter/speed"
    );

    // Initialize ROS 2 publisher on /cutter/status
    rclc_publisher_init_default(
        &status_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/cutter/status"
    );

    // Initialize ROS 2 publisher on /cutter/rpm
    rclc_publisher_init_default(
        &rpm_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/cutter/rpm"
    );

    // Initialize ROS 2 publisher on /cutter/current
    rclc_publisher_init_default(
        &current_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/cutter/current"
    );

    // Reserve one executor handle for the speed command and one for the heartbeat.
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg_speed, &subscription_callback, ON_NEW_DATA);

    // Initialize and add the heartbeat watchdog subscription for the cutter
    rclc_subscription_init_default(
        &heartbeat_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "/mower/heartbeat"
    );
    rclc_executor_add_subscription(&executor, &heartbeat_sub, &heartbeat_msg, &heartbeat_callback, ON_NEW_DATA);
}

void loop() {
    // Watchdog check. If the computer freezes, the blade is immediately switched off!
    unsigned long now_ms = millis();
    if (last_heartbeat_time > 0 && (now_ms - last_heartbeat_time > WATCHDOG_TIMEOUT_MS)) {
        expected_erpm = 0;
        set_cutter_rpm(0); // Switch blade off immediately!
        actual_rpm = 0;
        actual_current = 0.0;
        Serial.println("FATAL: CUTTER WATCHDOG TIMEOUT! Stopped cutting blade immediately!");
    }

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    read_vesc_telemetry(); //
    delay(10);
}