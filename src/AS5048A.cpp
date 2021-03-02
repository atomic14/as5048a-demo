#include "Arduino.h"

#include "AS5048A.h"

static const uint16_t AS5048A_CLEAR_ERROR_FLAG = 0x0001;
static const uint16_t AS5048A_PROGRAMMING_CONTROL = 0x0003;
static const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_HIGH = 0x0016;
static const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_LOW = 0x0017;
static const uint16_t AS5048A_DIAG_AGC = 0x3FFD;
static const uint16_t AS5048A_MAGNITUDE = 0x3FFE;
static const uint16_t AS5048A_ANGLE = 0x3FFF;

static const uint8_t AS5048A_AGC_FLAG = 0xFF;
static const uint8_t AS5048A_ERROR_PARITY_FLAG = 0x04;
static const uint8_t AS5048A_ERROR_COMMAND_INVALID_FLAG = 0x02;
static const uint8_t AS5048A_ERROR_FRAMING_FLAG = 0x01;

static const uint16_t AS5048A_DIAG_COMP_HIGH = 0x2000;
static const uint16_t AS5048A_DIAG_COMP_LOW = 0x1000;
static const uint16_t AS5048A_DIAG_COF = 0x0800;
static const uint16_t AS5048A_DIAG_OCF = 0x0400;

static const double AS5048A_MAX_VALUE = 8191.0;

SPIClass *vspi = NULL;

/**
 * Constructor
 */
AS5048A::AS5048A(byte VSPI_SS, byte VSPI_MISO, byte VSPI_MOSI, byte VSPI_SCLK)
    : _VSPI_SS(VSPI_SS), _VSPI_MISO(VSPI_MISO), _VSPI_MOSI(VSPI_MOSI), _VSPI_SCLK(VSPI_SCLK), errorFlag(false), ocfFlag(false), position(0)
{
}

/**
 * Initialiser
 * Sets up the SPI interface
 */
void AS5048A::begin()
{
  setDelay();

  // 3MHz clock (AMS should be able to accept up to 10MHz)
  this->settings = SPISettings(3000000, MSBFIRST, SPI_MODE1);

  vspi = new SPIClass();

  //setup pins
  pinMode(this->_VSPI_SS, OUTPUT);

  //SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
  vspi->begin(this->_VSPI_SCLK, this->_VSPI_MISO, this->_VSPI_MOSI, this->_VSPI_SS); //SCLK, MISO, MOSI, SS
}

/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each begin()-call the close() function must be called exactly 1 time
 */
void AS5048A::close()
{
  vspi->end();
}

/**
 * Utility function used to calculate even parity of an unigned 16 bit integer
 */
uint8_t AS5048A::spiCalcEvenParity(uint16_t value)
{
  uint8_t cnt = 0;

  for (uint8_t i = 0; i < 16; i++)
  {
    if (value & 0x1)
    {
      cnt++;
    }
    value >>= 1;
  }
  return cnt & 0x1;
}

/**
 * Get the rotation of the sensor relative to the zero position.
 *
 * @return {int16_t} between -2^13 and 2^13
 */
int16_t AS5048A::getRotation()
{
  uint16_t data;
  int16_t rotation;

  data = AS5048A::getRawRotation();
  rotation = static_cast<int16_t>(data) - static_cast<int16_t>(this->position);
  if (rotation > AS5048A_MAX_VALUE)
    rotation = -((0x3FFF) - rotation); //more than -180

  return rotation;
}

/**
 * Returns the raw angle directly from the sensor
 */
int16_t AS5048A::getRawRotation()
{
  return AS5048A::read(AS5048A_ANGLE);
}

/**
  * Get the rotation of the sensor relative to the zero position in degrees.
  *
  * @return {double} between 0 and 360
  */

double AS5048A::getRotationInDegrees()
{
  int16_t rotation = getRotation();
  double degrees = 360.0 * (rotation + AS5048A_MAX_VALUE) / (AS5048A_MAX_VALUE * 2.0);
  return degrees;
}

/**
  * Get the rotation of the sensor relative to the zero position in radians.
  *
  * @return {double} between 0 and 2 * PI
  */

double AS5048A::getRotationInRadians()
{
  int16_t rotation = getRotation();
  double radians = PI * (rotation + AS5048A_MAX_VALUE) / AS5048A_MAX_VALUE;
  return radians;
}

/**
 * returns the value of the state register
 * @return unsigned 16 bit integer containing flags
 */
uint16_t AS5048A::getState()
{
  return AS5048A::read(AS5048A_DIAG_AGC);
}

/**
 * Print the diagnostic register of the sensor
 */
void AS5048A::printState()
{

#ifdef DEBUG
  uint16_t data = AS5048A::getState();
  if (AS5048A::error())
  {
    Serial.print("Error bit was set!");
  }
  Serial.println(data, BIN);
#endif
}

/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 */
uint8_t AS5048A::getGain()
{
  uint16_t data = AS5048A::getState();
  return static_cast<uint8_t>(data & AS5048A_AGC_FLAG);
}

/**
 * Get diagnostic
 */
String AS5048A::getDiagnostic()
{
  uint16_t data = AS5048A::getState();
  if (data & AS5048A_DIAG_COMP_HIGH)
  {
    return "COMP high";
  }
  if (data & AS5048A_DIAG_COMP_LOW)
  {
    return "COMP low";
  }
  if (data & AS5048A_DIAG_COF)
  {
    return "CORDIC overflow";
  }
  if (data & AS5048A_DIAG_OCF && ocfFlag == false)
  {
    ocfFlag = true;
    return "Offset compensation finished";
  }
  return "";
}

/*
 * Get and clear the error register by reading it
 */
String AS5048A::getErrors()
{
  uint16_t error = AS5048A::read(AS5048A_CLEAR_ERROR_FLAG);
  if (error & AS5048A_ERROR_PARITY_FLAG)
  {
    return "Parity Error";
  }
  if (error & AS5048A_ERROR_COMMAND_INVALID_FLAG)
  {
    return "Command invalid";
  }
  if (error & AS5048A_ERROR_FRAMING_FLAG)
  {
    return "Framing error";
  }
  return "";
}

/*
 * Set the zero position
 */
void AS5048A::setZeroPosition(uint16_t position)
{
  this->position = position % 0x3FFF;
}

/*
 * Returns the current zero position
 */
uint16_t AS5048A::getZeroPosition()
{
  return this->position;
}

/*
 * Check if an error has been encountered.
 */
bool AS5048A::error()
{
  return this->errorFlag;
}

/*
 * Read a register from the sensor
 * Takes the address of the register as an unsigned 16 bit
 * Returns the value of the register
 */
uint16_t AS5048A::read(uint16_t registerAddress)
{
  uint16_t command = 0x4000; // PAR=0 R/W=R
  command = command | registerAddress;

  //Add a parity bit on the the MSB
  command |= static_cast<uint16_t>(spiCalcEvenParity(command) << 0xF);

#ifdef DEBUG
  Serial.print("Read (0x");
  Serial.print(registerAddress, HEX);
  Serial.print(") with command: 0b");
  Serial.println(command, BIN);
#endif

  //SPI - begin transaction
  vspi->beginTransaction(this->settings);

  //Send the command
  digitalWrite(this->_VSPI_SS, LOW);
  vspi->transfer16(command);
  digitalWrite(this->_VSPI_SS, HIGH);

  vTaskDelay(this->esp32_delay);

  //Now read the response
  digitalWrite(this->_VSPI_SS, LOW);
  uint16_t response = vspi->transfer16(0x00);
  digitalWrite(this->_VSPI_SS, HIGH);

  //SPI - end transaction
  vspi->endTransaction();

#ifdef DEBUG
  Serial.print("Read returned: ");
  Serial.println(command, BIN);
#endif

  //Check if the error bit is set
  if (response & 0x4000)
  {
#ifdef DEBUG
    Serial.println("Setting error bit");
#endif
    this->errorFlag = true;
  }
  else
  {
    this->errorFlag = false;
  }

  //Return the data, stripping the parity and error bits
  return response & ~0xC000;
}

/*
 * Write to a register
 * Takes the 16-bit  address of the target register and the unsigned 16 bit of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
uint16_t AS5048A::write(uint16_t registerAddress, uint16_t data)
{

  uint16_t command = 0x0000; // PAR=0 R/W=W
  command |= registerAddress;

  //Add a parity bit on the the MSB
  command |= static_cast<uint16_t>(spiCalcEvenParity(command) << 0xF);

#ifdef DEBUG
  Serial.print("Write (0x");
  Serial.print(registerAddress, HEX);
  Serial.print(") with command: 0b");
  Serial.println(command, BIN);
#endif

  //SPI - begin transaction
  vspi->beginTransaction(this->settings);

  //Start the write command with the target address
  digitalWrite(this->_VSPI_SS, LOW);
  vspi->transfer16(command);
  digitalWrite(this->_VSPI_SS, HIGH);

  uint16_t dataToSend = 0x0000;
  dataToSend |= data;

  //Craft another packet including the data and parity
  dataToSend |= static_cast<uint16_t>(spiCalcEvenParity(dataToSend) << 0xF);

#ifdef DEBUG
  Serial.print("Sending data to write: ");
  Serial.println(dataToSend, BIN);
#endif

  //Now send the data packet
  digitalWrite(this->_VSPI_SS, LOW);
  vspi->transfer16(dataToSend);
  digitalWrite(this->_VSPI_SS, HIGH);

  vTaskDelay(this->esp32_delay);

  digitalWrite(this->_VSPI_SS, LOW);
  uint16_t response = vspi->transfer16(0x0000);
  digitalWrite(this->_VSPI_SS, HIGH);

  //SPI - end transaction
  vspi->endTransaction();

  //Return the data, stripping the parity and error bits
  return response & ~0xC000;
}

/**
 * Set the delay acording to the microcontroller architecture
 */
void AS5048A::setDelay()
{
  this->esp32_delay = 5;
#ifdef DEBUG
  Serial.println("AS5048A working with ESP32");
#endif
}