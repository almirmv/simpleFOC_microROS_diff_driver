//Callback Interrupts. HALL Sensors---------------
void doAL(){sensorL.handleA();}
void doBL(){sensorL.handleB();}
void doCL(){sensorL.handleC();}

void doAR(){sensorR.handleA();}
void doBR(){sensorR.handleB();}
void doCR(){sensorR.handleC();}

//-------------------------------------------------
void setMotorsSpeed(){
  //Motion control function
  motorL.move(targetVelocityL); //2 Rad/s ~ 20rpm
  motorR.move(targetVelocityR);
}

//INIT MOTORS--------------------------------------
void initMotors(){
  // initialize magnetic sensor hardware
  sensorL.init();
  sensorR.init();
  sensorL.enableInterrupts(doAL, doBL, doCL);  // hardware interrupt enable
  sensorR.enableInterrupts(doAR, doBR, doCR);  // hardware interrupt enable
  // link the motor to the sensor
  motorL.linkSensor(&sensorL);
  motorR.linkSensor(&sensorR);
  // driver config
  driverL.init();
  driverR.init();
  motorL.linkDriver(&driverL);
  motorR.linkDriver(&driverR);
  // set motion control loop to be used
  motorL.controller = MotionControlType::velocity;
  motorR.controller = MotionControlType::velocity;
  // controller configuration 
  // default parameters in defaults.h
  // controller configuration based on the control type 
  // velocity PID controller parameters
  // default P=0.5 I = 10 D =0
  motorL.PID_velocity.P = L_P; //0.47
  motorL.PID_velocity.I = L_I; //2.2
  motorL.PID_velocity.D = L_D;
  
  motorR.PID_velocity.P = R_P; //0.47
  motorR.PID_velocity.I = R_I; //2.2
  motorR.PID_velocity.D = R_D;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motorL.PID_velocity.output_ramp = L_VOLT_RAMP;
  motorR.PID_velocity.output_ramp = R_VOLT_RAMP;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motorL.LPF_velocity.Tf = 0.015;
  motorR.LPF_velocity.Tf = 0.015;
  // since the phase resistance is provided we set the current limit not voltage
  // default 0.2
  motorL.current_limit = 3; // Amps
  motorR.current_limit = 3; // Amps
  
  // use monitoring with serial 
  //motorL.useMonitoring(Serial); // comment out if not needed
  //motorR.useMonitoring(Serial); // comment out if not needed
  /*
  printInfoSerial();
  Serial.println("L motor init");
  motorL.init();    // initialize motors
  printInfoSerial();
  Serial.println("L motor: align sensor and start FOC");
  motorL.initFOC(); // align sensor and start FOC
  printInfoSerial();
  Serial.println("R motor init");
  motorR.init();
  printInfoSerial();
  Serial.println("R motor: align sensor and start FOC");
  motorR.initFOC();
  */
  // define the motor id
 // command.add('A', onMotor, "motor");
  }

//MAIN LOO SIMPLE FOC ----------------------------------------------------
void loopSimpleFOC(){
  motorL.loopFOC();
  motorR.loopFOC();
}
