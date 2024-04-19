#include <SimpleFOC.h>
// #include <SimpleFOCCAN.h>

#define CAN_TX 16
#define CAN_RX 17

TaskHandle_t moveMotors;
void moveMotorsfun( void * pvParameters);
// Stepper motor instance
StepperMotor motor = StepperMotor(50);
// Stepper driver instance
StepperDriver4PWM driver = StepperDriver4PWM(26,25,33,32, 19, 23);

MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 15);
SPIClass* hspi = new SPIClass(HSPI); 

// CANDriver can = CANDriver(CAN_TX, CAN_RX);
// CANCommander canCommand = CANCommander(can);
Commander command = Commander(Serial);

void onMotor(char* cmd){ command.motor(&motor, cmd); }
// void doCommanderCAN(char* cmd) { canCommand.motor(&motor, cmd); }

void setup() {
  Serial.begin(115200);
  Serial.print("RUNNING ON CORE: ");
  Serial.println(xPortGetCoreID());

  disableCore0WDT();
  xTaskCreatePinnedToCore(
                      moveMotorsfun,   /* Task function. */
                      "Command Motors",     /* name of task. */
                      100000,       /* Stack size of task */
                      NULL,        /* parameter of the task */
                      2,           /* priority of the task */
                      &moveMotors,      /* Task handle to keep track of created task */
                      0);          /* pin task to core 0 */                  
  delay(500); 

  // define the motor id
    // comment out if not needed
  motor.useMonitoring(Serial);
  command.add('M', onMotor, "motor");
  // canCommand.add('M', doCommanderCAN, (char*)"motor");

  _delay(1000);
}


void moveMotorsfun( void * pvParameters) {
  motor.voltage_sensor_align = 5;
  // pwm frequency to be used [Hz]
  // driver.pwm_frequency = 20000;
  // power supply voltage [V]
  driver.voltage_power_supply = 19;
  // Max DC voltage allowed - default voltage_power_supply

  // ##################### ENCODER CONFIG
  // encoder.quadrature = Quadrature::ON;

  // encoder.pullup = Pullup::USE_INTERN;
  // encoder.init();
  // encoder.enableInterrupts(doA, doB);
  // motor.linkSensor(&encoder);

  // ##################### I2C CONFIG
  // sensor.init();
  // motor.linkSensor(&sensor);

  // ##################### SPI CONFIG
  sensor.init(hspi);
  motor.linkSensor(&sensor);


  // choose FOC modulation
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::angle;

  // power supply voltage [V]
  driver.voltage_power_supply = 19;
  // motor.motion_downsample = 2;
  driver.init();
  // link the motor to the sensor
  motor.linkDriver(&driver);

  // set control loop type to be used]
  
  driver.pwm_frequency = 20000;
  driver.voltage_limit = driver.voltage_power_supply / 2;
  motor.voltage_limit = driver.voltage_power_supply / 2;
  // controller configuration based on the control type 
  motor.PID_velocity.P = 2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = 0.009;

  // angle loop controller
  motor.P_angle.P = 20;
  motor.P_angle.P = 0;
  motor.P_angle.D = 0;
  // angle loop velocity limit
  motor.velocity_limit = 30;

  // use monitoring with serial for motor init
  // monitoring port
  

  motor.monitor_downsample = 1000;
  // initialise motor
  motor.monitor_variables =  _MON_TARGET | _MON_VOLT_Q | _MON_VOLT_D | _MON_VEL | _MON_ANGLE; 
  motor.init();
  // align encoder and start FOC
  motor.initFOC();
    for(;;){
        //This function keeps motors spinning and must be run as fast as possible
        motor.move();
        motor.loopFOC();
    }
}

void loop() {
  command.run();
  motor.monitor();
  // canCommand.runWithCAN();
}
