#include "drv824x_driver.h"

DRV824X_4PH::DRV824X_4PH(int ph1A,int ph1B,int ph2A,int ph2B,int en1, int en2, int nsleep1, int nsleep2,
int nCS, SPISettings settings){
  // Pin initialization
  pwm1A = ph1A;
  pwm1B = ph1B;
  pwm2A = ph2A;
  pwm2B = ph2B;

  // enable_pin pins
  enable_pin1 = en1;
  enable_pin2 = en2;

  // nsleep pins
  nsleep_pin1 = nsleep1;
  nsleep_pin2 = nsleep2;
  
  nCS = nCS;
  settings = settings;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  pwm_frequency = NOT_SET;

}

// enable motor driver
void  DRV824X_4PH::enable(){
    // enable_pin the driver - if enable_pin pin available
    if ( _isset(enable_pin1) ) digitalWrite(enable_pin1, HIGH);
    if ( _isset(enable_pin2) ) digitalWrite(enable_pin2, HIGH);
    // set zero to PWM
    setPwm(0,0);
}

// disable motor driver
void DRV824X_4PH::disable()
{
  // set zero to PWM
  setPwm(0, 0);
  // disable the driver - if enable_pin pin available
  if ( _isset(enable_pin1) ) digitalWrite(enable_pin1, LOW);
  if ( _isset(enable_pin2) ) digitalWrite(enable_pin2, LOW);

}

// Clear motor faults using specialized function for driver
void  DRV824X_4PH::clear(){
    // Use "clear faults" pin function of nSleep pin
    if ( _isset(nsleep_pin1) ) digitalWrite(nsleep_pin1, LOW); 
    if ( _isset(nsleep_pin2) ) digitalWrite(nsleep_pin2, LOW); 
    delayMicroseconds(30);
    if ( _isset(nsleep_pin1) ) digitalWrite(nsleep_pin1, HIGH); 
    if ( _isset(nsleep_pin2) ) digitalWrite(nsleep_pin2, HIGH); 
}

// init hardware pins
int DRV824X_4PH::init() {

  // PWM pins
  pinMode(pwm1A, OUTPUT);
  pinMode(pwm1B, OUTPUT);
  pinMode(pwm2A, OUTPUT);
  pinMode(pwm2B, OUTPUT);

  if( _isset(enable_pin1) ) pinMode(enable_pin1, OUTPUT);
  if( _isset(enable_pin2) ) pinMode(enable_pin2, OUTPUT);

  if( _isset(nsleep_pin1) ) pinMode(nsleep_pin1, OUTPUT);
  if( _isset(nsleep_pin2) ) pinMode(nsleep_pin2, OUTPUT);

  if( _isset(nCS) ) pinMode(nCS, OUTPUT);

  // sanity check for the voltage limit configuration
  if( !_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  // Set the pwm frequency to the pins
  // hardware specific function - depending on driver and mcu
  params = _configure4PWM(pwm_frequency, pwm1A, pwm1B, pwm2A, pwm2B);
  initialized = (params!=SIMPLEFOC_DRIVER_INIT_FAILED);  
  return params!=SIMPLEFOC_DRIVER_INIT_FAILED;
}


// Set voltage to the pwm pin
void DRV824X_4PH::setPwm(float Ualpha, float Ubeta) {
  float duty_cycle1A(0.0f),duty_cycle1B(0.0f),duty_cycle2A(0.0f),duty_cycle2B(0.0f);
  // limit the voltage in driver
  Ualpha = _constrain(Ualpha, -voltage_limit, voltage_limit);
  Ubeta = _constrain(Ubeta, -voltage_limit, voltage_limit);
  // hardware specific writing
  if( Ualpha > 0 )
    duty_cycle1B = _constrain(abs(Ualpha)/voltage_power_supply,0.0f,1.0f);
  else
    duty_cycle1A = _constrain(abs(Ualpha)/voltage_power_supply,0.0f,1.0f);

  if( Ubeta > 0 )
    duty_cycle2B = _constrain(abs(Ubeta)/voltage_power_supply,0.0f,1.0f);
  else
    duty_cycle2A = _constrain(abs(Ubeta)/voltage_power_supply,0.0f,1.0f);
  // write to hardware
  _writeDutyCycle4PWM(duty_cycle1A, duty_cycle1B, duty_cycle2A, duty_cycle2B, params);
}

byte DRV824X_4PH::getDeviceID(){
	uint16_t command = DRV824X_DEVID_REG | DRV824X_RW; // set r=1
	/*uint16_t cmdresult =*/ spi_transfer16(command);
	uint16_t return_frame = nop16();
    // Upper byte has fault status info, but ignore that for DeviceID
	return (return_frame >> 0) & 0xFF;
}

DRV824xResult DRV824X_4PH::getStatus1(){
    DRV824xResult result;
    uint16_t command = DRV824X_STATUS1_REG | DRV824X_RW; // set r=1
    spi_transfer16(command);
    uint16_t return_frame = nop16();
    // Bit shift our frame to get the "status" byte, then convert it using getFaultSummary
    result.status = getFaultSummary((return_frame >> 1) & 0xFF);
    result.data = (return_frame >> 0) & 0xFF;
}

DRV824xResult DRV824X_4PH::getStatus2(){
    DRV824xResult result;
    uint16_t command = DRV824X_STATUS2_REG | DRV824X_RW; // set r=1
    spi_transfer16(command);
    uint16_t return_frame = nop16();
    // Bit shift our frame to get the "status" byte, then convert it using getFaultSummary
    result.status = getFaultSummary((return_frame >> 1) & 0xFF);
    result.data = (return_frame >> 0) & 0xFF;
}

void DRV824X_4PH::setCommand(uint8_t command){
    uint16_t frame_command = DRV824X_COMMAND_REG | 0x0000 | command; // set r=0 indicating Write
    spi_transfer16(frame_command);
}

void DRV824X_4PH::setSPIin(uint8_t spiIn){
    uint16_t frame_command = DRV824X_SPIIN_REG | 0x0000 | spiIn; // set r=0 indicating Write
    spi_transfer16(frame_command);
}

void DRV824X_4PH::setConfig1(uint8_t config1){
    uint16_t frame_command = DRV824X_CONFIG1_REG | 0x0000 | config1; // set r=0 indicating Write
    spi_transfer16(frame_command);
}

void DRV824X_4PH::setConfig2(uint8_t config2){
    uint16_t frame_command = DRV824X_CONFIG2_REG | 0x0000 | config2; // set r=0 indicating Write
    spi_transfer16(frame_command);
}

void DRV824X_4PH::setConfig3(uint8_t config3){
    uint16_t frame_command = DRV824X_CONFIG3_REG | 0x0000 | config3; // set r=0 indicating Write
    spi_transfer16(frame_command);
}

void DRV824X_4PH::setConfig4(uint8_t config4){
    uint16_t frame_command = DRV824X_CONFIG4_REG | 0x0000 | config4; // set r=0 indicating Write
    spi_transfer16(frame_command);
}


DRV824xFault DRV824X_4PH::getFaultSummary(uint8_t status){
    DRV824xFault result;

    // Use Bitset to easily convert from byte to individual Bools
	std::bitset<8> bits(status);
    result.SPI_ERR = bits[0];
    result.TSD = bits[1];
    result.OCP = bits[2];
    result.VMUV = bits[3];
    result.VMOV = bits[4];
    result.FAULT = bits[5];

	return result;
}

void clear(){
    uint8_t command = DRV824X_COMMAND_REG | DRV824X_RW;
}


uint16_t DRV824X_4PH::nop16(){
	uint16_t result = spi_transfer16(0xFFFF); // using 0xFFFF as nop instead of 0x0000, then next call to fastAngle will return an angle
	return result&DRV824X_RESULT_MASK;
}

uint8_t DRV824X_4PH::nop(){
	uint16_t result = spi_transfer(0xFF); // using 0xFFFF as nop instead of 0x0000, then next call to fastAngle will return an angle
	return result&DRV824X_RESULT_MASK;
}

uint16_t DRV824X_4PH::spi_transfer16(uint16_t outdata) {
	if _isset(nCS)
		digitalWrite(nCS, 0);
	spi->beginTransaction(settings);
	uint16_t result = spi->transfer16(outdata);
	spi->endTransaction();
	if _isset(nCS)
		digitalWrite(nCS, 1);
	// TODO check parity
	// errorflag = ((result&AS5048A_ERRFLG)>0);
	return result;
}

uint16_t DRV824X_4PH::spi_transfer(uint8_t outdata) {
	if _isset(nCS)
		digitalWrite(nCS, 0);
	spi->beginTransaction(settings);
	uint16_t result = spi->transfer(outdata);
	spi->endTransaction();
	if _isset(nCS)
		digitalWrite(nCS, 1);
	// TODO check parity
	// errorflag = ((result&AS5048A_ERRFLG)>0);
	return result;
}