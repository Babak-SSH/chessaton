#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>

// #include <Servo.h>
#include <ESP32Servo.h> 

#include <stdio.h>

#define JOINT_DOUBLE_LEN 20
#define ARRAY_LEN 200


rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// publisher
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg_heartbeat;
rclc_executor_t executor_pub;
rcl_timer_t timer;

// subscriber
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg_led;
sensor_msgs__msg__JointState joint_state_msg;
rclc_executor_t executor_sub;
rosidl_runtime_c__String__Sequence name__string_sequence;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

double rad2deg(double in) {
    return 180.0 * (in / M_PI);
}

double dist2deg(double in) {
    return 180.0 * (in / 0.03);
}

static const int servoPin1 = 4;
static const int servoPin2 = 16;
static const int servoPin3 = 17;
static const int servoPin4 = 5;
static const int servoPin5 = 18;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

const int min_pulse = 544;
const int max_pulse = 2400;

// Error handle loop
void error_loop() {
    while(1) {
        delay(100);
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        Serial.println("timer_callback");
        Serial.println(msg_heartbeat.data);
        Serial.println("[APP] Free memory: " + String(esp_get_free_heap_size()) + " bytes");
        RCSOFTCHECK(rcl_publish(&publisher, &msg_heartbeat, NULL));
        msg_heartbeat.data++;
    }
}

void subscription_callback(const void * msgin) {
    // Serial.println("subscription_callback");
    const sensor_msgs__msg__JointState * joint_state_msg = (const sensor_msgs__msg__JointState *)msgin;

    double joint = 0.0;

    Serial.println("--------------------------------------");

    Serial.println(rad2deg(joint_state_msg->position.data[0]), 5);
    joint = rad2deg(joint_state_msg->position.data[0]) + 90.0;
    servo1.write(joint);

    Serial.println(rad2deg(joint_state_msg->position.data[1]), 5);
    joint = rad2deg(joint_state_msg->position.data[1]) + 90.0;
    servo2.write(joint);

    Serial.println(rad2deg(joint_state_msg->position.data[2]), 5);
    joint = rad2deg(joint_state_msg->position.data[2]) + 90.0;
    servo3.write(joint);

    Serial.println(rad2deg(joint_state_msg->position.data[3]), 5);
    joint = rad2deg(joint_state_msg->position.data[3]) + 90.0;
    servo4.write(joint);

    Serial.println(joint_state_msg->position.data[4], 5);
    Serial.println(joint_state_msg->position.data[5], 5);
    joint = dist2deg(joint_state_msg->position.data[5]);
    servo5.write(joint);
    
    Serial.println("--------------------------------------");
}

void setup() {
	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

    // Configure serial transport
    Serial.begin(115200);

    servo1.attach(servoPin1, 544, 2400);
    servo2.attach(servoPin2, 544, 2400);
    servo3.attach(servoPin3, 544, 2400);
    servo4.attach(servoPin4, 544, 2400);
    servo5.attach(servoPin5, 544, 2400);

    Serial.println("connecting to wifi...");
    IPAddress agent_ip(192, 168, 0, 100);
    size_t agent_port = 8888;
    char SSID[] = "YOUR_WIFI_SSID";
    char SSID_PW[]= "YOUR_WIFI_PASS";

    set_microros_wifi_transports(SSID, SSID_PW, agent_ip, agent_port);
    delay(2000);
    Serial.println("connected to wifi.");
    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));
    Serial.println("created node.");

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/esp_publisher"));

    Serial.println("Created publisher.");
    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/joint_states"));
    Serial.println("Created subscriber.");

    // create timer,
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));
    Serial.println("Created Timer.");

    //Allocate memory
    bool success = rosidl_runtime_c__String__Sequence__init(&name__string_sequence, 20);
    joint_state_msg.name = name__string_sequence;
    success = rosidl_runtime_c__String__assignn(&joint_state_msg.name.data[0], "Servo1", 40);
    success = rosidl_runtime_c__String__assignn(&joint_state_msg.name.data[1], "Servo2", 40);
    success = rosidl_runtime_c__String__assignn(&joint_state_msg.name.data[2], "Servo3", 40);
    success = rosidl_runtime_c__String__assignn(&joint_state_msg.name.data[3], "Servo4", 40);
    success = rosidl_runtime_c__String__assignn(&joint_state_msg.name.data[4], "Servo5", 40);
    success = rosidl_runtime_c__String__assignn(&joint_state_msg.name.data[5], "Servo6", 40);
    success = rosidl_runtime_c__String__assignn(&joint_state_msg.name.data[6], "Servo7", 40);
    success = rosidl_runtime_c__String__assignn(&joint_state_msg.name.data[7], "Servo8", 40);

    Serial.println("Allocated memory.");

    // Assigning dynamic memory to POSITION rosidl_runtime_c__double__Sequence 
    joint_state_msg.position.capacity = 26;
    joint_state_msg.position.size = 18;
    joint_state_msg.position.data = (double*) malloc(joint_state_msg.position.capacity * sizeof(double));

    // Assigning dynamic memory to EFFORT rosidl_runtime_c__double__Sequence 
    joint_state_msg.effort.capacity = 26;
    joint_state_msg.effort.size = 18;
    joint_state_msg.effort.data = (double*) malloc(joint_state_msg.position.capacity * sizeof(double));

    // Assigning dynamic memory to VELOCITY rosidl_runtime_c__double__Sequence 
    joint_state_msg.velocity.capacity = 26;
    joint_state_msg.velocity.size = 18;
    joint_state_msg.velocity.data = (double*) malloc(joint_state_msg.position.capacity * sizeof(double));


    // Header
    // Assigning dynamic memory to the frame_id char sequence
    joint_state_msg.header.frame_id.capacity = 100;
    joint_state_msg.header.frame_id.data = (char*) malloc(joint_state_msg.header.frame_id.capacity * sizeof(char));
    joint_state_msg.header.frame_id.size = 0;

    // Assigning value to the frame_id char sequence
    strcpy(joint_state_msg.header.frame_id.data, "frame id");
    joint_state_msg.header.frame_id.size = strlen(joint_state_msg.header.frame_id.data);

    // Assigning time stamp
    joint_state_msg.header.stamp.sec = (uint16_t)(rmw_uros_epoch_millis()/1000);
    joint_state_msg.header.stamp.nanosec = (uint32_t)rmw_uros_epoch_nanos();
    Serial.println("finished assignments.");

    // create executor
    RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

    RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &joint_state_msg, &subscription_callback, ON_NEW_DATA));
    Serial.println("executor for sub.");

    msg_heartbeat.data = 0;

    Serial.println("Finished setup.");
}

void loop() {
    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
    RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}
