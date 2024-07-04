/*
 * DRV824X.h
 *
 *  Created on: 7 July 2024
 *      Author: jbruslind
 */

#ifndef DRV824X_DRIVER_h
#define DRV824X_DRIVER_h

#include "common/base_classes/StepperDriver.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"
#include "common/defaults.h"
#include "hardware_api.h"
#include "SPI.h"
#include <bitset>

/**
 4 pwm stepper driver class
*/

struct DRV824xFault {
	bool SPI_ERR; //!< SPI Transaction Error
	// bool POR; //!< Power on Reset Detected
	bool FAULT; //!< Some Fault detected (OR operation on other faults)
  bool VMOV; //!< VMotor Over Voltage
  bool VMUV; //!< VMotor Under Voltage
  bool OCP; //!< Overcurrent event detected
  bool TSD; //!< Thermal Shut Down 
  // bool OLA; // Open Load Condition during the ACTIVE state
};

struct DRV824xResult {
  uint8_t data; //!< Data byte from SPI Frame
  DRV824xFault status; //!< Status from SPI Frame
};

#define DRV824X_RW 0x4000
#define DRV824X_DEVID_REG 0x0000
#define DRV824X_FAULT_REG 0x0100
#define DRV824X_STATUS1_REG 0x0200
#define DRV824X_STATUS2_REG 0x0300

#define DRV824X_COMMAND_REG 0x08
#define DRV824X_SPIIN_REG 0x09
#define DRV824X_CONFIG1_REG 0x0A
#define DRV824X_CONFIG2_REG 0x0B
#define DRV824X_CONFIG3_REG 0x0C
#define DRV824X_CONFIG4_REG 0x0D

#define DRV824X_RESULT_MASK 0xFF

#define DRV8245_BITORDER MSBFIRST


static SPISettings DRV824xSPISettings(8000000, DRV8245_BITORDER, SPI_MODE1); // @suppress("Invalid arguments")

class DRV824X_2PH
{
  public:
    /**
      StepperMotor class constructor
      @param ph1A 1A phase pwm pin
      @param ph1B 1B phase pwm pin
      @param en1 enable pin phase 1 (optional input)
      @param nsleep1 sleep pin phase 1 
      @param nCS CS pin for Driver
    */
   DRV824X_2PH();
   DRV824X_2PH(int ph1A,int ph1B, int en = NOT_SET, int nsleep = NOT_SET, int nCS = NOT_SET,
    SPISettings settings = DRV824xSPISettings);

    int pwm1A; //!< phase 1A pwm pin number
  	int pwm1B; //!< phase 1B pwm pin number
    int enable_pin; //!< enable pin number phase 1
    int nsleep_pin; //!< nSleep pin number for clearing faults on phase 1
    int cs;

    byte getDeviceID(); // return dev id from IC
    DRV824xFault getFaultSummary(uint8_t status); // return fault summary from device

    DRV824xResult getStatus1(); 
    DRV824xResult getStatus2(); 

    void setCommand(uint8_t command); 
    void setSPIin(uint8_t spiIn);
    void setConfig1(uint8_t config1);
    void setConfig2(uint8_t config2);
    void setConfig3(uint8_t config3);
    void setConfig4(uint8_t config4);

    /**  Motor hardware init function */
  	int init();
    /** Motor disable function */
  	void disable();
    /** Motor enable function */
    void enable();

    void clear();


  private:
    uint8_t nop();
    uint16_t nop16();
    uint16_t spi_transfer16(uint16_t outdata);
    uint16_t spi_transfer(uint8_t outdata);
    SPIClass* spi;
    SPISettings spisettings;
    bool errorflag = false;
    int nCS = -1;
};

class DRV824X_4PH: public StepperDriver
{
  public:
    /**
      StepperMotor class constructor
      @param ph1A 1A phase pwm pin
      @param ph1B 1B phase pwm pin
      @param ph2A 2A phase pwm pin
      @param ph2B 2B phase pwm pin
      @param en1 enable pin phase 1 (optional input)
      @param en2 enable pin phase 2 (optional input)
      @param nsleep1 sleep pin phase 1 
      @param nsleep2 sleep pin phase 2 
      @param nCS CS pin for Driver
    */
    DRV824X_4PH(DRV824X_2PH drv1, DRV824X_2PH drv2);
    
    /**  Motor hardware init function */
  	int init() override;
    /** Motor disable function */
  	void disable() override;
    /** Motor enable function */
    void enable() override;

    // // hardware variables

    DRV824X_2PH driver1; 
    DRV824X_2PH driver2;


    /** 
     * Set phase voltages to the harware 
     * 
     * @param Ua phase A voltage
     * @param Ub phase B voltage
    */
    void setPwm(float Ua, float Ub) override;

    /**  Motor hardware clear function */
    // 1 = Clear faults successfully, 0 = faults are still present
  	int clear();    

  private:

        
};


#endif
