//loop to indicate error with blinking LED
void error_loop(){
  while(1){
    digitalWrite(LED_ERROR_PIN, !digitalRead(LED_ERROR_PIN));
    digitalWrite(LED_BOARD_PIN, digitalRead(LED_ERROR_PIN));
    delay(100);
  }
}

//Timer callback. For the publiher executor. 
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    RCSOFTCHECK(rcl_publish(&publisher_left, &msg_left, NULL));      //publish left tick count
    RCSOFTCHECK(rcl_publish(&publisher_right, &msg_right, NULL));    //publish right tick count
    msg_left.data = sensorL.getAngle() * 100;
    msg_right.data = sensorR.getAngle() * 100;
    //getAngle*100 gets me the exact number I needed to avoid modifying the code :slight_smile:
  }
}


//Subscriber call back. "Received Twist message"
void subscription_callback(const void *msgin) {
  //digitalWrite(LED_ERROR_PIN, !digitalRead(LED_ERROR_PIN));
  failSafeTimer = millis();                                                           //clear fail safe. If no twist message are received, fail safe will stop motors. 
  const geometry_msgs__msg__Twist * msg_twist = (const geometry_msgs__msg__Twist *)msgin;
  digitalWrite(LED_ERROR_PIN, (msg_twist->linear.x == 0) ? LOW : HIGH);                           // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  //calculate motors velocities
  //reference https://snapcraft.io/blog/your-first-robot-the-driver-4-5
  left_wheel_vel = msg_twist->linear.x - ((msg_twist->angular.z * WHEEL_BASE) / 2.0);       //speed in m/s
  right_wheel_vel = msg_twist->linear.x + ((msg_twist->angular.z * WHEEL_BASE) / 2.0);      //speed in m/s
  //comvert m/s in rad/s to use in slimpleFOC. W = V/R
  targetVelocityL = left_wheel_vel / WHEEL_RADIUS;
  targetVelocityR = right_wheel_vel / WHEEL_RADIUS;
  setMotorsSpeed();     //set new motors velocities
  
}

void initMicroRos(){
  set_microros_transports();  //if USB or Serial
  //set_microros_wifi_transports("SKYNET", "@modeloT1000", "192.168.3.104", 8888); // if WIFI. (ssid, psk, agent_ip, agent_port); //WIFI
 
  allocator = rcl_get_default_allocator();                                              //create allocator
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));                            //init options
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));       //init node

  // create subscriber to topic: cmd_vel
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));                                                              //init executor_sub 
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg_twist, &subscription_callback, ON_NEW_DATA));                     //add subscription to cmd_vel
  
  // create Publisher to topic left_tick_count and... Publisher to topic right_tick_count
  RCCHECK(rclc_publisher_init_default(&publisher_left, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "left_ticks"));
  RCCHECK(rclc_publisher_init_default(&publisher_right, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "right_ticks"));

  // create timer, called every 1000 ms to publish "left_tick_count" and "right_tick_count"
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(pub_timer_timeout), timer_callback));              //init timer
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));                                      //init executor_pub
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));                                                          //add timer to pub executor
}

void loopMicroRos(){
  //delay(100);
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
}
