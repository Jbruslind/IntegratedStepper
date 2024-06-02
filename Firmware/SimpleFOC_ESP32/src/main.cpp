#include <SimpleFOC.h>
// #include <SimpleFOCCAN.h>

#define CAN_TX 16
#define CAN_RX 17

#define nSLEEP 32
#define nOFF 2
#define DRVOFF 2

#define nFAULT_1 25
#define nFAULT_2 18

#define IPROPI_1 33
#define IPROPI_2 35

#define A_p 1/3070
#define R_IPROP 820

TaskHandle_t moveMotors;
void moveMotorsfun( void * pvParameters);
// Stepper motor instance
StepperMotor motor = StepperMotor(50);
// Stepper driver instance
StepperDriver4PWM driver = StepperDriver4PWM(27,26,5,4);

MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 15);
SPIClass* hspi = new SPIClass(HSPI); 

LowsideCurrentSense current_sense = LowsideCurrentSense(A_p, R_IPROP, IPROPI_1, IPROPI_2);

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
  
  
  pinMode(nSLEEP, OUTPUT);
  pinMode(nOFF, OUTPUT);
  pinMode(DRVOFF, OUTPUT);

  pinMode(nFAULT_1, INPUT);
  pinMode(nFAULT_2, INPUT);

  digitalWrite(nOFF, HIGH);
  digitalWrite(DRVOFF, LOW);

  digitalWrite(nSLEEP, LOW);
  delayMicroseconds(30);
  digitalWrite(nSLEEP, HIGH);

  // ##################### SPI CONFIG
  sensor.init(hspi);
  motor.linkSensor(&sensor);


  // choose FOC modulation
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::angle_openloop;

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
  
  // current_sense.linkDriver(&driver);
  // current_sense.init();

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
int monitor_downsample = motor.monitor_downsample;
int monitor_cnt = 0;
int nfault_1 = 1;
int nfault_2 = 1;

void loop() {
  command.run();
  motor.monitor();
  if( !monitor_downsample || monitor_cnt++ < (monitor_downsample-1) ) 
  { 

  }
  else{
    monitor_cnt = 0; 
    nfault_1 = digitalRead(nFAULT_1);
    nfault_2 = digitalRead(nFAULT_2);
    // printf("NFAULT_1 : %d NFAULT_2 : %d \n", nfault_1 , nfault_2);
    if(!nfault_1 || !nfault_2){
      digitalWrite(nSLEEP, LOW);
      delayMicroseconds(30);
      digitalWrite(nSLEEP, HIGH);
    }
  }
  // canCommand.runWithCAN();
}
