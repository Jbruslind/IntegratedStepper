#include "drv824x_driver.h"

DRV824X_2PH::DRV824X_2PH(int ph1A,int ph1B, int en, int nsleep, int nCS, SPISettings settings, SPIClass* _spi){ 
    // Pin initialization
    pwm1A = ph1A;
    pwm1B = ph1B;
    // enable_pin pins
    enable_pin = en;
    // nsleep pins
    nsleep_pin = nsleep;
    cs = nCS;
    spisettings = settings;

    spi = _spi;
	//SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
	spi->begin();
}

DRV824X_2PH::DRV824X_2PH(int ph1A,int ph1B, int nCS, SPISettings settings, SPIClass* _spi){ 
    // Pin initialization
    pwm1A = ph1A;
    pwm1B = ph1B;
    // enable_pin pins
    cs = nCS;
    spisettings = settings;

    spi = _spi;
	//SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
	spi->begin();
}

DRV824X_2PH::DRV824X_2PH() {
}

DRV824X_4PH::DRV824X_4PH(DRV824X_2PH drv1, DRV824X_2PH drv2){

  // default power-supply value

  driver1 = drv1;
  driver2 = drv2;

  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  pwm_frequency = NOT_SET;

}

void  DRV824X_4PH::enable(){
    driver1.enable();
    driver2.enable();
    setPwm(0,0);
}

void  DRV824X_4PH::disable(){
    setPwm(0,0);
    driver1.disable();
    driver2.disable();   
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

int DRV824X_4PH::clear() {
    driver1.clear();
    driver2.clear();

    if(driver1.getStatus1().status.FAULT == 0 && driver2.getStatus1().status.FAULT == 0)
    {
        return 1;
    }
    return 0;
}

int DRV824X_4PH::init() {
    // Check both return 1 for successfully init
    if(driver1.init() && driver2.init()){}
    else{
        printf("INIT DRIVERS FAILED");
        return -1;
    }

    if(driver1.getStatus1().status.FAULT){
        printf("DRIVER 1 FAULT: %h", driver1.getFault());
    }

    if(driver2.getStatus1().status.FAULT){
        printf("DRIVER 2 FAULT: %h", driver2.getFault());
    }
    // sanity check for the voltage limit configuration
    if( !_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

    // Set the pwm frequency to the pins
    // hardware specific function - depending on driver and mcu
    params = _configure4PWM(pwm_frequency, driver1.pwm1A, driver1.pwm1B, driver2.pwm1A, driver2.pwm1B);
    initialized = (params!=SIMPLEFOC_DRIVER_INIT_FAILED);  
    return params!=SIMPLEFOC_DRIVER_INIT_FAILED;
}

// enable motor driver
void  DRV824X_2PH::enable(){
    // Reset Faults and Unlock registers
    setCommand(0b10010000);
    // Enable drivers 
    setSPIin(0b00000000);
    // Relock registers
    setCommand(0b00000010);
    // enable_pin the driver - if enable_pin pin available
    if ( _isset(enable_pin) ) digitalWrite(enable_pin, HIGH);
}

// disable motor driver
void DRV824X_2PH::disable()
{
    // Reset Faults and Unlock registers
    setCommand(0b10010000);
    // Disable drivers 
    setSPIin(0b00001100);
    // Relock registers
    setCommand(0b00000010);
  // disable the driver - if enable_pin pin available
  if ( _isset(enable_pin) ) digitalWrite(enable_pin, LOW);

}

// Clear motor faults using specialized function for driver
void  DRV824X_2PH::clear(){
    // send the "Clear Fault" command + Lock SPIIN and CONFIG registers
    setCommand(uint8_t(0x9A));
}

// init hardware pins
int DRV824X_2PH::init() {

    // PWM pins
    pinMode(pwm1A, OUTPUT);
    pinMode(pwm1B, OUTPUT);

    if( _isset(enable_pin) ) pinMode(enable_pin, OUTPUT);

    if( _isset(nsleep_pin) ) pinMode(nsleep_pin, OUTPUT);

    if( _isset(cs) ) pinMode(cs, OUTPUT);

     if( _isset(nsleep_pin) ) digitalWrite(nsleep_pin, HIGH);
    // Reset Faults and Unlock registers
    setCommand(0b10010000);
    // Config 2 sets ITRIP and Diag behavior
    setConfig2(0b01100111);
    // Config 3 sets Slew rate and Mode
    setConfig3(0b00011101);
    // // Config 4 sets tOCP and iOCP
    // setConfig4(0b11000000);
    // Enable drivers 
    setSPIin(0b00000000);
    // Relock registers
    setCommand(0b00000010);

    return 1;
}

byte DRV824X_2PH::getDeviceID(){
	uint16_t command = DRV824X_DEVID_REG | DRV824X_RW; // set r=1
	uint16_t return_frame = spi_transfer16(command);
    // Upper byte has fault status info, but ignore that for DeviceID
    // printf("getDeviceID: %x DeviceID: %x \n", command, return_frame);
	return (return_frame >> 0) & 0xFF;
}

byte DRV824X_2PH::getFault(){
	uint16_t command = DRV824X_FAULT_REG | DRV824X_RW; // set r=1
	uint16_t return_frame = spi_transfer16(command);
    // Upper byte has fault status info, but ignore that for DeviceID
    // printf("getFault %x Fault: %x \n", command, return_frame);
	return (return_frame >> 0) & 0xFF;
}

DRV824xResult DRV824X_2PH::getStatus1(){
    DRV824xResult result;
    uint16_t command = DRV824X_STATUS1_REG | DRV824X_RW; // set r=1
    uint16_t return_frame = spi_transfer16(command);
    // uint16_t return_frame = nop16();
    // printf("getStatus1: %x fault1: %x \n", command, (return_frame >> 8) & 0xFF);
    // Bit shift our frame to get the "status" byte, then convert it using getFaultSummary
    result.status = getFaultSummary((return_frame >> 8) & 0xFF);
    result.data = (return_frame >> 0) & 0xFF;
    return result;
}

DRV824xResult DRV824X_2PH::getStatus2(){
    DRV824xResult result;
    uint16_t command = DRV824X_STATUS2_REG | DRV824X_RW; // set r=1
    spi_transfer16(command);
    uint16_t return_frame = nop16();
    // printf("getStatus2: %x Status2: %x \n", command, return_frame);
    // Bit shift our frame to get the "status" byte, then convert it using getFaultSummary
    result.status = getFaultSummary((return_frame >> 8) & 0xFF);
    result.data = (return_frame >> 0) & 0xFF;
    return result;
}

DRV824xResult DRV824X_2PH::getConfig1(){
    DRV824xResult result;
    uint16_t command = DRV824X_CONFIG1_REG << 8 | DRV824X_RW; // set r=1
    uint16_t return_frame = spi_transfer16(command);
    // printf("getConfig1: %x Config1: %x \n", command, return_frame);
    // Bit shift our frame to get the "status" byte, then convert it using getFaultSummary
    result.status = getFaultSummary((return_frame >> 8) & 0xFF);
    result.data = (return_frame >> 0) & 0xFF;
    return result;
}

DRV824xResult DRV824X_2PH::getConfig2(){
    DRV824xResult result;
    uint16_t command = DRV824X_CONFIG2_REG << 8 | DRV824X_RW; // set r=1
    uint16_t return_frame = spi_transfer16(command);
    // printf("getConfig1: %x Config1: %x \n", command, return_frame);
    // Bit shift our frame to get the "status" byte, then convert it using getFaultSummary
    result.status = getFaultSummary((return_frame >> 8) & 0xFF);
    result.data = (return_frame >> 0) & 0xFF;
    return result;
}

DRV824xResult DRV824X_2PH::getConfig3(){
    DRV824xResult result;
    uint16_t command = DRV824X_CONFIG3_REG << 8 | DRV824X_RW; // set r=1
    uint16_t return_frame = spi_transfer16(command);
    // printf("getConfig1: %x Config1: %x \n", command, return_frame);
    // Bit shift our frame to get the "status" byte, then convert it using getFaultSummary
    result.status = getFaultSummary((return_frame >> 8) & 0xFF);
    result.data = (return_frame >> 0) & 0xFF;
    return result;
}

DRV824xResult DRV824X_2PH::getConfig4(){
    DRV824xResult result;
    uint16_t command = DRV824X_CONFIG4_REG << 8 | DRV824X_RW; // set r=1
    uint16_t return_frame = spi_transfer16(command);
    // printf("getConfig1: %x Config1: %x \n", command, return_frame);
    // Bit shift our frame to get the "status" byte, then convert it using getFaultSummary
    result.status = getFaultSummary((return_frame >> 8) & 0xFF);
    result.data = (return_frame >> 0) & 0xFF;
    return result;
}

void DRV824X_2PH::setCommand(uint8_t command){
    uint16_t frame_command = DRV824X_COMMAND_REG << 8 | 0x0000 | command; // set r=0 indicating Write
    spi_transfer16(frame_command);
}

void DRV824X_2PH::setSPIin(uint8_t spiIn){
    uint16_t frame_command = DRV824X_SPIIN_REG << 8 | 0x0000 | spiIn; // set r=0 indicating Write
    spi_transfer16(frame_command);
}

// CONFIG1 Register (Address = 0Ah) [reset = 10h]
void DRV824X_2PH::setConfig1(uint8_t config1){
    uint16_t frame_command = DRV824X_CONFIG1_REG << 8 | 0x0000 | config1; // set r=0 indicating Write
    spi_transfer16(frame_command);
}
// CONFIG2 Register (Address = 0Bh) [reset = 00h]
void DRV824X_2PH::setConfig2(uint8_t config2){
    uint16_t frame_command = DRV824X_CONFIG2_REG << 8 | 0x0000 | config2; // set r=0 indicating Write
    spi_transfer16(frame_command);
}
// CONFIG3 Register (Address = 0Ch) [reset = 40h]
void DRV824X_2PH::setConfig3(uint8_t config3){
    uint16_t frame_command = DRV824X_CONFIG3_REG << 8 | 0x0000 | config3; // set r=0 indicating Write
    spi_transfer16(frame_command);
}
// CONFIG4 Register (Address = 0Dh) [reset = 04h]
void DRV824X_2PH::setConfig4(uint8_t config4){
    uint16_t frame_command = DRV824X_CONFIG4_REG << 8 | 0x0000 | config4; // set r=0 indicating Write
    spi_transfer16(frame_command);
}


DRV824xFault DRV824X_2PH::getFaultSummary(uint8_t status){
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


uint16_t DRV824X_2PH::nop16(){
	uint16_t result = spi_transfer16(0x0000); // using 0xFFFF as nop instead of 0x0000, then next call to fastAngle will return an angle
	return result;
}

uint8_t DRV824X_2PH::nop(){
	uint8_t result = spi_transfer(0x00); // using 0xFFFF as nop instead of 0x0000, then next call to fastAngle will return an angle
	return result;
}

uint16_t DRV824X_2PH::spi_transfer16(uint16_t outdata) {
	if _isset(cs)
		digitalWrite(cs, 0);
	spi->beginTransaction(spisettings);
	uint16_t result = spi->transfer16(outdata);
	spi->endTransaction();
	if _isset(cs)
		digitalWrite(cs, 1);
	// TODO check parity
	// errorflag = ((result&AS5048A_ERRFLG)>0);
	return result;
}

uint16_t DRV824X_2PH::spi_transfer(uint8_t outdata) {
	if _isset(cs)
		digitalWrite(cs, 0);
	spi->beginTransaction(spisettings);
	uint16_t result = spi->transfer(outdata);
	spi->endTransaction();
	if _isset(cs)
		digitalWrite(cs, 1);
	// TODO check parity
	// errorflag = ((result&AS5048A_ERRFLG)>0);
	return result;
}