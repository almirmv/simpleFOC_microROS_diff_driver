/* Author:                      Almir MV
 * website:                     www.omecatronico.com.br
 * Description:                 microROS node to control 2 brushless motors (hoverboard) with SimpleFOC Arduino and ROS2.
 * Target ROS:                  ROS2 HUMBLE
 * Communication:               USB or wifi. microROS on ESP32. 
 * Subscribe to:                cmd_vel
 * publishes to:                left_ticks
 *                              right_ticks
 * Run micro_ros_agent before connecting ESP32 
 *
*/

#include <SimpleFOC.h>

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/Int16.h>

//===========DEFINIÇÕES======================
//Constantes
#define LED_BOARD_PIN     2           //LED on ESP32
#define LED_ERROR_PIN     22          //LED off board
#define FAIL_SAFE_PERIOD  500         //stop motors if no command was received in this period. Value in miliseconds
#define NUM_CHARS_BUFFER  32          //number of chars in buffer (buffer size in chars)
#define L_P               0.47        //Left motor Proportional value
#define L_I               2.2         //Left motor Integrative value
#define L_D               0.001       //Left motor Derivative value
#define L_VOLT_RAMP       300         //Left motor jerk control using voltage voltage ramp // default value is 300 volts per sec  ~ 0.3V per millisecond
#define R_P               0.47
#define R_I               2.2
#define R_D               0.001
#define R_VOLT_RAMP       300         

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

//Constantes
#define WHEEL_RADIUS      0.085       //(diameter 17cm so 8,5cm radius) Wheel radius in meters
#define WHEEL_BASE        0.36        //(36cm) Distance from center of the left tire to the center of the right tire in m 
// Number of ticks a wheel makes moving a linear distance of 1 meter
#define TICKS_PER_METER   3100        // This value was measured manually. Originally 2880
#define PWM_TURN          80          // Turning PWM output (0 = min, 255 = max for PWM values)
// Set maximum and minimum limits for the PWM values
#define PWM_MIN           80          // about 0.1 m/s
#define PWM_MAX           100         // about 0.172 m/s

const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
const unsigned int pub_timer_timeout = 1000;          //defines de pub rate. In ms. Publish every 1000ms 

//Variaveis--------------------------------
float targetVelocityL = 0;                            //L target velocity in rad/s 
float targetVelocityR = 0;                            //R target velocity in rad/s
unsigned long failSafeTimer = 0;                      //count miliseconds for the fail safe feature
float left_wheel_vel = 0;                             //calculated velocity in m/s
float right_wheel_vel = 0;                            //calculated velocity in m/s
//Instanciando Objetos---------------------
BLDCMotor motorL = BLDCMotor( 15 );                   //motor instance //(15 pole pairs, 0.17 ohms phase)
BLDCMotor motorR = BLDCMotor( 15 );                   //motor instance //(15 pole pairs, 0.17 ohms phase)
BLDCDriver3PWM driverL = BLDCDriver3PWM(4,0, 2, 15);  //driver instance
BLDCDriver3PWM driverR = BLDCDriver3PWM(4,0, 2, 15);  //driver instance
HallSensor sensorL = HallSensor(35,32,33,15);         //(pinA,pinB,pinC, pole pairs)
HallSensor sensorR = HallSensor(35,32,33,15);         //(pinA,pinB,pinC, pole pairs)

rcl_node_t node;                                      //node
rclc_support_t support;                               //support
rcl_allocator_t allocator;                            //allocator
rclc_executor_t executor_sub;                         //subscriber executor
rclc_executor_t executor_pub;                         //publisher executor
rcl_timer_t timer;                                    //timer for publisher

//subscriber to Twist msg
rcl_subscription_t subscriber;                        //subscriber
geometry_msgs__msg__Twist msg_twist;                  //Twist msg

//publisher to left_tick_count
rcl_publisher_t publisher_left;                       //publisher left 
std_msgs__msg__Int16 msg_left;

//publisher to right_tick_count
rcl_publisher_t publisher_right;                      //publiher right
std_msgs__msg__Int16 msg_right;


//SETUP===================================================================================
void setup() {
 pinMode(LED_BOARD_PIN, OUTPUT);
 pinMode(LED_ERROR_PIN, OUTPUT);
 digitalWrite(LED_BOARD_PIN, HIGH);
 digitalWrite(LED_ERROR_PIN, HIGH);  
 
 initMicroRos();                                      //init all microROS related stuff 
 delay(2000);  
 initMotors();
 digitalWrite(LED_BOARD_PIN, LOW);
 digitalWrite(LED_ERROR_PIN, LOW);                    //turn off error led  
}

//LOOP===================================================================================
void loop(){
  //test fail safe
  if(millis() - failSafeTimer  > FAIL_SAFE_PERIOD){
    targetVelocityL = 0;
    targetVelocityR = 0;
    setMotorsSpeed();         //updateMotorsVelocity();
    failSafeTimer = millis();
  }
  loopSimpleFOC();            //main FOC algorithm function
  loopMicroRos();             //main microROS algorithm function

}
