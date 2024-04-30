#include <stdio.h>

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

// #include <ESP32Servo.h> // use this library for sending commands from esp32
#include <Adafruit_PWMServoDriver.h> // this library is used to communicate with pca9685


// called this way, it uses the default address 0x40 for the first board's address.
Adafruit_PWMServoDriver pwmBoard = Adafruit_PWMServoDriver(0x40);
static const int servoMin = 150; // servo minimum pulse
static const int servoMax = 600; // servo maximum pulse 
static const int servoFrequency = 50; // analo servo frequency

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

// use these two function to debug code and read errors
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){\
//    printf("Failed status on line %d: %d. Message: %s, Aborting.\n",__LINE__,(int)temp_rc, rcl_get_error_string().str);\
//   rcutils_reset_error();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// radians to degrees conversion
double rad2deg(double in) {
    return 180.0 * (in / M_PI);
}

// convert grippers traveled distance to pulse width for servo
double dist2pulse(double in) {
    return map(int(in*1000), 0, 23, servoMin, servoMax);
}

// radians to pulse width conversion
int rad2pulse(double in) { // -1* is for reverse direction in hardware
    return map(int(-1*rad2deg(in)*10), -900, 900, servoMin, servoMax);
}

// Error handle loop
void error_loop() {
    while(1) {
        delay(100);
    }
}

// place holder for publisher callback (it can be used to send feedback or status)
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
    const sensor_msgs__msg__JointState * joint_state_msg = (const sensor_msgs__msg__JointState *)msgin;

    int pulse = 0;

    Serial.println("--------------------------------------");
    for (int i=0; i < 8; i++) {
        String joint_name = joint_state_msg->name.data[i].data;
        if (joint_name == "chessaton_joint1") {
            // Serial.println("joint1");
            // Serial.println(rad2deg(joint_state_msg->position.data[i]), 5);
            pulse = rad2pulse(joint_state_msg->position.data[i]);
            Serial.println(pulse);
            pwmBoard.setPWM(0, 0, pulse);
        }
        else if (joint_name == "chessaton_joint2") {
            // Serial.println("joint2");
            // Serial.println(rad2deg(joint_state_msg->position.data[i]), 5);
            pulse = rad2pulse(joint_state_msg->position.data[i]);
            Serial.println(pulse);
            pwmBoard.setPWM(1, 0, pulse);
        }
        else if (joint_name == "chessaton_joint3") {
            // Serial.println("joint3");
            // Serial.println(rad2deg(-joint_state_msg->position.data[i]), 5);
            pulse = rad2pulse(-joint_state_msg->position.data[i]);
            Serial.println(pulse);
            pwmBoard.setPWM(2, 0, pulse);
        }
        else if (joint_name == "chessaton_joint4") {
            // Serial.println("joint4");
            // Serial.println(rad2deg(-joint_state_msg->position.data[i]), 5);
            pulse = rad2pulse(-joint_state_msg->position.data[i]);
            Serial.println(pulse);
            pwmBoard.setPWM(3, 0, pulse);
        }
        else if (joint_name == "left_finger_joint") {
            // Serial.println("joint5");
            // Serial.println(joint_state_msg->position.data[i], 5);
            Serial.println(joint_state_msg->position.data[i+1], 5);
            pulse = dist2pulse(joint_state_msg->position.data[i]);
            pwmBoard.setPWM(4, 0, pulse);
        }
    }
}

void setup() {
    // Configure serial transport
    Serial.begin(115200);

    pwmBoard.begin();
    pwmBoard.setOscillatorFrequency(27000000);    //Set the PWM oscillator frequency, used for fine calibration
    pwmBoard.setPWMFreq(servoFrequency);          //Set the servo operating frequency

    Serial.println("connecting to wifi...");
    IPAddress agent_ip(192, 168, 0, 101);
    size_t agent_port = 8888;
    char SSID[] = "dlink_2.4G";
    char SSID_PW[]= "ujitgr12";

    set_microros_wifi_transports(SSID, SSID_PW, agent_ip, agent_port);
    delay(2000);
    Serial.println("connected to wifi.");

    rclc_support_t support;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_node_t node;

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    Serial.println("init options");

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
    delay(10);
    // uncomment to enable publisher
    // RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
    RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(1)));
}
