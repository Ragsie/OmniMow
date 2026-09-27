#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/battery_state.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/empty.h> // Empty message type for watchdog
#include <driver/twai.h>
#include <Wire.h>               // I2C for BNO085 IMU
#include <SparkFun_BNO080_Arduino_Library.h> // SparkFun BNO085 Library
#include <std_msgs/msg/bool.h>               // Bool message type for rain sensor
#include <sensor_msgs/msg/imu.h>

#define CAN_TX_PIN GPIO_NUM_21
#define CAN_RX_PIN GPIO_NUM_22
#define BUMPER_PIN 15
#define RELAY_PIN  4

// Dedicated I2C pins for the BNO085 IMU (since GPIO 21/22 are used by the CAN bus)
#define BNO_SDA_PIN 25
#define BNO_SCL_PIN 26

// Digital rain sensor pin with internal pull-up
#define RAIN_PIN 32

// Daly BMS UART pins (Serial2 on ESP32)
#define BMS_RX_PIN 16
#define BMS_TX_PIN 17

// Autoro VESC IDs on the CAN bus (with #ifndef guards to prevent redefinition warnings from platformio.ini)
#ifndef VESC_LEFT_ID
#define VESC_LEFT_ID  1
#endif

#ifndef VESC_RIGHT_ID
#define VESC_RIGHT_ID 2
#endif

rcl_subscription_t subscriber;
rcl_subscription_t heartbeat_sub;           // Heartbeat receiver
rcl_publisher_t battery_pub;                //
rcl_publisher_t state_pub;                  //
rcl_publisher_t cycles_pub;                 //
rcl_publisher_t drive_current_pub;          //
rcl_publisher_t rain_pub;                   // ROS 2 rain sensor publisher
rcl_publisher_t imu_pub;
geometry_msgs__msg__Twist msg_twist;
std_msgs__msg__Empty heartbeat_msg;         // Heartbeat message
sensor_msgs__msg__BatteryState battery_msg;  //
std_msgs__msg__Int32 state_msg;             //
std_msgs__msg__Int32 cycles_msg;            //
std_msgs__msg__Float32 drive_current_msg;   //
std_msgs__msg__Bool rain_msg;               // ROS 2 rain sensor message
sensor_msgs__msg__Imu imu_msg;
rclc_executor_t executor;

// The Radxa must refresh this timestamp; otherwise the hardware relay removes motor power.
// Watchdog variable to catch a frozen or disconnected Radxa computer
volatile unsigned long last_heartbeat_time = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 500;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

volatile bool emergency_stop = false;
unsigned long last_bms_request = 0;         // BMS polling timer
const unsigned long BMS_INTERVAL = 1000;    // Request data every second

int32_t expected_erpm_left = 0;
int32_t expected_erpm_right = 0;
bool drive_stuck = false;

// BNO085 IMU instance for tilt/lift detection
BNO080 myIMU;
bool imu_present = false;
unsigned long last_state_publish = 0;
float left_current = 0.0;
float right_current = 0.0;

// Hardware interrupt: triggers instantly on physical collision or STOP button
void IRAM_ATTR handleBumper() {
    emergency_stop = true;
    digitalWrite(RELAY_PIN, LOW); // Cut power to the 40A automotive relay immediately!
    twai_stop();                 // Stop the CAN bus to prevent any motor rotation
}

// Initialize the TWAI driver at 500 kbps
void init_twai() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
        Serial.println("TWAI (CAN) driver installed and started.");
    } else {
        Serial.println("Unable to install the TWAI driver.");
    }
}

// Send an ERPM command to the VESC over CAN using extended frames (VESC-specific protocol)
void send_vesc_erpm(uint8_t controller_id, int32_t erpm) {
    if (emergency_stop) return;

    twai_message_t message;
    message.identifier = (0x03 << 8) | controller_id; // CAN_PACKET_SET_RPM ID
    message.extd = 1; // Extended frame
    message.data_length_code = 4;

    message.data[0] = (erpm >> 24) & 0xFF;
    message.data[1] = (erpm >> 16) & 0xFF;
    message.data[2] = (erpm >> 8) & 0xFF;
    message.data[3] = erpm & 0xFF;

    twai_transmit(&message, pdMS_TO_TICKS(10));
}

// Daly BMS requests use a 13-byte frame: header, command, payload, and additive checksum.
void request_bms_data(uint8_t cmd_type) {
    uint8_t cmd[13] = {0xA5, 0x40, cmd_type, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t checksum = 0;
    for (int i = 0; i < 12; i++) {
        checksum += cmd[i];
    }
    cmd[12] = checksum;
    Serial2.write(cmd, 13);
}

// Parse complete BMS frames and publish the decoded measurement on its ROS 2 topic.
void read_bms_data() {
    if (Serial2.available() >= 13) {
        while (Serial2.available() >= 13 && Serial2.peek() != 0xA5) {
            Serial2.read();
        }

        if (Serial2.available() < 13) return;

        uint8_t buffer[13];
        Serial2.readBytes(buffer, 13);

        uint8_t checksum = 0;
        for (int i = 0; i < 12; i++) {
            checksum += buffer[i];
        }

        if (checksum == buffer[12]) {
            if (buffer[1] == 0x80) {
                if (buffer[2] == 0x90) { // Voltage, current, SOC
                    float voltage = ((buffer[4] << 8) | buffer[5]) / 10.0;
                    float current = (((buffer[8] << 8) | buffer[9]) - 30000) / 10.0;
                    float soc = ((buffer[10] << 8) | buffer[11]) / 10.0;

                    battery_msg.voltage = voltage;
                    battery_msg.current = current;
                    battery_msg.percentage = soc / 100.0;
                    battery_msg.present = true;
                    rcl_publish(&battery_pub, &battery_msg, NULL);
                }
                else if (buffer[2] == 0x91) { // Charge cycles
                    uint16_t cycles = (buffer[8] << 8) | buffer[9];
                    cycles_msg.data = cycles;
                    rcl_publish(&cycles_pub, &cycles_msg, NULL);
                }
                else if (buffer[2] == 0x92) { // Maximum temperature
                    int8_t max_temp = buffer[4] - 40; // 40 °C offset
                    battery_msg.temperature = max_temp;
                    battery_msg.present = true;
                    rcl_publish(&battery_pub, &battery_msg, NULL);
                }
            }
        }
    }
}

// Heartbeat callback resets the watchdog timer
void heartbeat_callback(const void * msvgin) {
    last_heartbeat_time = millis();
}

// Refresh the watchdog whenever the ROS 2 heartbeat reaches the drive controller.
void heartbeat_callback(const void * msvgin) {
    last_heartbeat_time = millis();
}

void subscription_callback(const void * msvgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msvgin;

    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;

    // ANTI-LAWN-TEARING FILTER
    if (abs(angular_z) > 0.5 && abs(linear_x) < 0.05) {
        angular_z = (angular_z > 0) ? 0.2 : -0.2;
        linear_x = 0.08;
    }

    // TRACK_WIDTH must be defined during compilation via build_flags in platformio.ini
    double track_width = TRACK_WIDTH;
    double speed_left = linear_x - (angular_z * track_width / 2.0);
    double speed_right = linear_x + (angular_z * track_width / 2.0);

    expected_erpm_left = speed_left * 3000.0;
    expected_erpm_right = speed_right * 3000.0;

    if (abs(linear_x) < 0.01 && abs(angular_z) < 0.01) {
        drive_stuck = false;
    }

    if (!drive_stuck) {
        send_vesc_erpm(VESC_LEFT_ID, expected_erpm_left);
        send_vesc_erpm(VESC_RIGHT_ID, -expected_erpm_right);
    } else {
        send_vesc_erpm(VESC_LEFT_ID, 0);
        send_vesc_erpm(VESC_RIGHT_ID, 0);
    }
}

// Decode VESC status frames and publish current, rain, and state telemetry once per second.
void read_drive_telemetry() {
    twai_message_t rx_msg;
    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        uint32_t id = rx_msg.identifier;
        uint8_t cmd = (id >> 8) & 0xFF;
        uint8_t sender_id = id & 0xFF;

        if (cmd == 0x09) { // CAN_PACKET_STATUS_1
            int32_t rpm = (rx_msg.data[0] << 24) | (rx_msg.data[1] << 16) | (rx_msg.data[2] << 8) | rx_msg.data[3];
            float current = ((int16_t)((rx_msg.data[4] << 8) | rx_msg.data[5])) / 10.0;

            if (sender_id == VESC_LEFT_ID) {
                left_current = abs(current);
                if (abs(expected_erpm_left) > 1000 && abs(rpm) < 50 && left_current > 15.0) {
                    drive_stuck = true;
                }
            }
            if (sender_id == VESC_RIGHT_ID) {
                right_current = abs(current);
                if (abs(expected_erpm_right) > 1000 && abs(rpm) < 50 && right_current > 15.0) {
                    drive_stuck = true;
                }
            }
        }
    }

    unsigned long now = millis();
    if (now - last_state_publish >= 1000) {
        last_state_publish = now;

        // Read rain sensor status (active LOW when it is raining)
        bool is_raining = (digitalRead(RAIN_PIN) == LOW);
        rain_msg.data = is_raining;
        rcl_publish(&rain_pub, &rain_msg, NULL);

        // Publish total drive-wheel current
        drive_current_msg.data = left_current + right_current;
        rcl_publish(&drive_current_pub, &drive_current_msg, NULL);

        if (emergency_stop) {
            state_msg.data = 5; // EMERGENCY_STOP / BUMPER
        } else if (is_raining) {
            state_msg.data = 8; // RAIN - makes the robot automatically return to the docking station
        } else if (drive_stuck) {
            state_msg.data = 4; // STUCK
        } else if (abs(expected_erpm_left) > 0 || abs(expected_erpm_right) > 0) {
            state_msg.data = 1; // MOWING / MOVING
        } else {
            state_msg.data = 0; // STOP
        }
        rcl_publish(&state_pub, &state_msg, NULL);
    }
}

void setup() {
    Serial.begin(115200);

    // Configure Daly BMS UART (Serial2)
    Serial2.begin(9600, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
    Serial.println("Daly Smart BMS UART2 (9600 baud) started.");

    pinMode(BUMPER_PIN, INPUT_PULLUP);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);

    attachInterrupt(digitalPinToInterrupt(BUMPER_PIN), handleBumper, FALLING);

    // Configure rain sensor pin
    pinMode(RAIN_PIN, INPUT_PULLUP);

    // Initialize BNO085 IMU over I2C on dedicated pins
    Wire.begin(BNO_SDA_PIN, BNO_SCL_PIN);
    if (myIMU.begin() == false) {
        Serial.println("Warning: BNO085 IMU not found! Check wiring on GPIO 25/26.");
        imu_present = false;
    } else {
        // Enable rotation vector (Roll, Pitch, Yaw) at 50 ms polling rate (20 Hz)
        myIMU.enableRotationVector(50);
        Serial.println("BNO085 IMU initialized and running with stable sensor fusion!");
        imu_present = true;
    }

    init_twai();
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "drive_controller", "", &support);

    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"
    );

    rclc_publisher_init_default(
        &battery_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
        "/battery_state"
    );

    rclc_publisher_init_default(
        &state_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/mower/state"
    );

    rclc_publisher_init_default(
        &cycles_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/battery/charge_cycles"
    );

    rclc_publisher_init_default(
        &drive_current_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/drive/current"
    );

    // Initialize rain sensor publisher
    rclc_publisher_init_default(
        &rain_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/mower/rain"
    );

    // Reserve one executor handle for /cmd_vel and one for /mower/heartbeat.
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg_twist, &subscription_callback, ON_NEW_DATA);

    // Initialize and add the heartbeat watchdog subscription
    rclc_subscription_init_default(
        &heartbeat_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "/mower/heartbeat"
    );
    rclc_executor_add_subscription(&executor, &heartbeat_sub, &heartbeat_msg, &heartbeat_callback, ON_NEW_DATA);
}

void loop() {
    // Watchdog hardware check. If the heartbeat was previously received but is now missing for > 500 ms
    unsigned long now_ms = millis();
    if (!emergency_stop && last_heartbeat_time > 0 && (now_ms - last_heartbeat_time > WATCHDOG_TIMEOUT_MS)) {
        emergency_stop = true;
        digitalWrite(RELAY_PIN, LOW); // Immediate physical emergency stop of the 40A automotive relay!
        twai_stop();                 // Shut down the CAN bus immediately to stop the motors
        Serial.println("FATAL: WATCHDOG TIMEOUT! Radxa computer stopped responding. Power cut!");
    }

    // BNO085 tilt (roll) and lift monitoring for extreme physical safety
    if (!emergency_stop && imu_present && myIMU.dataAvailable() == true) {
        float roll = (myIMU.getRoll()) * 180.0 / PI;   // Convert Pitch/Roll radians to degrees
        float pitch = (myIMU.getPitch()) * 180.0 / PI;

        imu_msg.header.frame_id.data = (char*)"imu_link";
        imu_msg.header.stamp.sec = millis() / 1000;
        imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

        imu_msg.orientation.x = myIMU.getQuatI();
        imu_msg.orientation.y = myIMU.getQuatJ();
        imu_msg.orientation.z = myIMU.getQuatK();
        imu_msg.orientation.w = myIMU.getQuatReal();

        // Standard high covariance if not reporting raw linear acceleration/angular velocity
        for (int i = 0; i < 9; i++) {
            imu_msg.orientation_covariance[i] = 0.001;
            imu_msg.angular_velocity_covariance[i] = 0.01;
            imu_msg.linear_acceleration_covariance[i] = 0.1;
        }

        if (!emergency_stop) {
            rcl_publish(&imu_pub, &imu_msg, NULL);
        }

        // Safety lock: if lawnmower tilts more than 35 degrees or is lifted
        if (!emergency_stop && (abs(roll) > 35.0 || abs(pitch) > 35.0)) {
            emergency_stop = true;
            digitalWrite(RELAY_PIN, LOW); // Cut power to 40A main relay immediately!
            twai_stop();                 // Stop CAN bus to halt motor rotation
            Serial.print("FATAL: EMERGENCY STOP! Lift/Tilt detected! ");
            Serial.print("Roll: "); Serial.print(roll, 1);
            Serial.print("deg | Pitch: "); Serial.print(pitch, 1); Serial.println("deg");
        }
    }

    if (!emergency_stop) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        // Continue polling sensors and publishing telemetry while the safety lock is active.
        unsigned long now = millis();
        if (now - last_bms_request >= BMS_INTERVAL) {
            last_bms_request = now;
            static uint8_t bms_state = 0;
            if (bms_state == 0) {
                request_bms_data(0x90);
                bms_state = 1;
            } else if (bms_state == 1) {
                request_bms_data(0x91);
                bms_state = 2;
            } else {
                request_bms_data(0x92);
                bms_state = 0;
            }
        }

        read_bms_data();
        read_drive_telemetry();
    } else {
        unsigned long now = millis();
        if (now - last_state_publish >= 1000) {
            last_state_publish = now;
            state_msg.data = 5;
            rcl_publish(&state_pub, &state_msg, NULL);
        }
        delay(100);
    }
}