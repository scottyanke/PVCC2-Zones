/**
 * This is a very modified verson of the ModbusRtu.h file from 
 * the Modbus-Master-Slave-for-Arduino library.  There isn't actually
 * a cpp part, as everything is in this header file.  It's been stripped
 * to remove the SoftwareSerial code, remove all slave stuff, and be
 * as minimal as possible to limit memory usage.  It's just the master
 * side of the world, as far as modbus goes.
 */
/**
 * @file 	ModbusRtu.h
 * @version     1.21
 * @date        2016.02.21
 * @author 	Samuel Marco i Armengol
 * @contact     sammarcoarmengol@gmail.com
 * @contribution Helium6072
 * @contribution gabrielsan
 *
 * @description
 *  Arduino library for communicating with Modbus devices
 *  over RS232/USB/485 via RTU protocol.
 *
 *  Further information:
 *  http://modbus.org/
 *  http://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
 *
 * @license
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; version
 *  2.1 of the License.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * @defgroup setup Modbus Object Instantiation/Initialization
 * @defgroup loop Modbus Object Management
 * @defgroup buffer Modbus Buffer Management
 * @defgroup discrete Modbus Function Codes for Discrete Coils/Inputs
 * @defgroup register Modbus Function Codes for Holding/Input Registers
 *
 */

#include <inttypes.h>

/**
 * @struct modbus_t
 * @brief
 * Master query structure:
 * This includes all the necessary fields to make the Master generate a Modbus query.
 * A Master may keep several of these structures and send them cyclically or
 * use them according to program needs.
 */
typedef struct
{
  uint8_t u8id;          /*!< Slave address between 1 and 247. 0 means broadcast */
  uint8_t u8fct;         /*!< Function code: 1, 2, 3, 4, 5, 6, 15 or 16 */
  uint16_t u16RegAdd;    /*!< Address of the first register to access at slave/s */
  uint16_t u16CoilsNo;   /*!< Number of coils or registers to access */
  uint16_t *au16reg;     /*!< Pointer to memory image in master */
}
modbus_t;

enum
{
  RESPONSE_SIZE = 6,
  EXCEPTION_SIZE = 3,
  CHECKSUM_SIZE = 2
};

/**
 * @enum MESSAGE
 * @brief
 * Indexes to telegram frame positions
 */
enum MESSAGE
{
  ID = 0, //!< ID field
  FUNC, //!< Function code position
  ADD_HI, //!< Address high byte
  ADD_LO, //!< Address low byte
  NB_HI, //!< Number of coils or registers high byte
  NB_LO, //!< Number of coils or registers low byte
  BYTE_CNT  //!< byte counter
};

/**
 * @enum MB_FC
 * @brief
 * Modbus function codes summary.
 * These are the implement function codes either for Master or for Slave.
 *
 * @see also fctsupported
 * @see also modbus_t
 */
enum MB_FC
{
  MB_FC_NONE                     = 0,   /*!< null operator */
  MB_FC_READ_COILS               = 1,	/*!< FCT=1 -> read coils or digital outputs */
  MB_FC_READ_DISCRETE_INPUT      = 2,	/*!< FCT=2 -> read digital inputs */
  MB_FC_READ_REGISTERS           = 3,	/*!< FCT=3 -> read registers or analog outputs */
  MB_FC_READ_INPUT_REGISTER      = 4,	/*!< FCT=4 -> read analog inputs */
  MB_FC_WRITE_COIL               = 5,	/*!< FCT=5 -> write single coil or output */
  MB_FC_WRITE_REGISTER           = 6,	/*!< FCT=6 -> write single register */
  MB_FC_WRITE_MULTIPLE_COILS     = 15,	/*!< FCT=15 -> write multiple coils or outputs */
  MB_FC_WRITE_MULTIPLE_REGISTERS = 16	/*!< FCT=16 -> write multiple registers */
};

enum COM_STATES
{
  COM_IDLE                     = 0,
  COM_WAITING                  = 1
};

enum ERR_LIST
{
  ERR_NOT_MASTER                = -1,
  ERR_POLLING                   = -2,
  ERR_BUFF_OVERFLOW             = -3,
  ERR_BAD_CRC                   = -4,
  ERR_EXCEPTION                 = -5
};

enum
{
  NO_REPLY = 255,
  EXC_FUNC_CODE = 1,
  EXC_ADDR_RANGE = 2,
  EXC_REGS_QUANT = 3,
  EXC_EXECUTE = 4
};

const unsigned char fctsupported[] =
{
  MB_FC_READ_COILS,
  MB_FC_READ_DISCRETE_INPUT,
  MB_FC_READ_REGISTERS,
  MB_FC_READ_INPUT_REGISTER,
  MB_FC_WRITE_COIL,
  MB_FC_WRITE_REGISTER,
  MB_FC_WRITE_MULTIPLE_COILS,
  MB_FC_WRITE_MULTIPLE_REGISTERS
};

#define T35  5
//!< maximum size for the communication buffer in bytes
#define  MAX_BUFFER  256	

/**
 * @class Modbus
 * @brief
 * Arduino class library for communicating with Modbus devices over
 * USB/RS232/485 (via RTU protocol).
 */
class Modbus
{
private:
    HardwareSerial *port; //!< Pointer to Serial class object
    uint8_t u8id; //!< 0=master, 1..247=slave number
    uint8_t u8serno; //!< serial port: 0-Serial, 1..3-Serial1..Serial3; 4: use software serial
    uint8_t u8txenpin; //!< flow control pin: 0=USB or RS-232 mode, >0=RS-485 mode
    uint8_t u8state;
    uint8_t u8lastError;
    uint8_t au8Buffer[MAX_BUFFER];
    uint8_t u8BufferSize;
    uint8_t u8lastRec;
    uint16_t *au16regs;
    uint16_t u16InCnt, u16OutCnt, u16errCnt;
    uint16_t u16timeOut;
    uint32_t u32time, u32timeOut, u32overTime;
    uint8_t u8regsize;

    void init(uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin);
	  void init(uint8_t u8id);
    void sendTxBuffer();
    int8_t getRxBuffer();
    uint16_t calcCRC(uint8_t u8length);
    uint8_t validateAnswer();
    uint8_t validateRequest();
    void get_FC1();
    void get_FC3();
    int8_t process_FC3( uint16_t *regs, uint8_t u8size );
    int8_t process_FC5( uint16_t *regs, uint8_t u8size );
    int8_t process_FC6( uint16_t *regs, uint8_t u8size );
    int8_t process_FC15( uint16_t *regs, uint8_t u8size );
    int8_t process_FC16( uint16_t *regs, uint8_t u8size );
    void buildException( uint8_t u8exception ); // build exception message

public:
    Modbus();
    Modbus(uint8_t u8id, uint8_t u8serno);
    Modbus(uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin);
    Modbus(uint8_t u8id);
    void begin(long u32speed);
    void begin();
    void setTimeOut( uint16_t u16timeOut); //!<write communication watch-dog timer
    uint16_t getTimeOut(); //!<get communication watch-dog timer value
    bool getTimeOutState(); //!<get communication watch-dog timer state
    int8_t query( modbus_t telegram ); //!<only for master
    int8_t poll(); //!<cyclic poll for master
    int8_t poll( uint16_t *regs, uint8_t u8size ); //!<cyclic poll for slave
    uint16_t getInCnt(); //!<number of incoming messages
    uint16_t getOutCnt(); //!<number of outcoming messages
    uint16_t getErrCnt(); //!<error counter
    uint8_t getID(); //!<get slave ID between 1 and 247
    uint8_t getState();
    uint8_t getLastError(); //!<get last error message
    void setID( uint8_t u8id ); //!<write new ID for the slave
    void setTxendPinOverTime( uint32_t u32overTime );
    void end(); //!<finish any communication and release serial communication port
};

/* _____PUBLIC FUNCTIONS_____________________________________________________ */

/**
 * @brief
 * Full constructor for a Master/Slave through USB/RS232C/RS485
 * It needs a pin for flow control only for RS485 mode
 *
 * @param u8id   node address 0=master, 1..247=slave
 * @param u8serno  serial port used 0..3
 * @param u8txenpin pin for txen RS-485 (=0 means USB/RS232C mode)
 * @ingroup setup
 * @overload Modbus::Modbus(uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin)
 * @overload Modbus::Modbus(uint8_t u8id)
 * @overload Modbus::Modbus()
 */

Modbus::Modbus(uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin)
{
    init(u8id, u8serno, u8txenpin);
}

/**
 * @brief
 * Initialize class object.
 *
 * Sets up the serial port using specified baud rate.
 * Call once class has been instantiated, typically within setup().
 *
 * @see http://arduino.cc/en/Serial/Begin#.Uy4CJ6aKlHY
 * @param speed   baud rate, in standard increments (300..115200)
 * @ingroup setup
 */
void Modbus::begin(long u32speed)
{
  port = &Serial;

  port->begin(u32speed);

  while(port->read() >= 0) ESP.wdtFeed();
  u8lastRec = u8BufferSize = 0;
  u16InCnt = u16OutCnt = u16errCnt = 0;
}

/**
 * @brief
 * Method to write the overtime count for txend pin.
 * It waits until count reaches 0 after the transfer is done.
 * With this, you can extend the time between txempty and
 * the falling edge if needed.
 *
 * @param 	uint32_t	overtime count for txend pin
 * @ingroup setup
 */
void Modbus::setTxendPinOverTime( uint32_t u32overTime )
{
    this->u32overTime = u32overTime;
}

/**
 * @brief
 * Initialize time-out parameter
 *
 * Call once class has been instantiated, typically within setup().
 * The time-out timer is reset each time that there is a successful communication
 * between Master and Slave. It works for both.
 *
 * @param time-out value (ms)
 * @ingroup setup
 */
void Modbus::setTimeOut( uint16_t u16timeOut)
{
    this->u16timeOut = u16timeOut;
}

/**
 * @brief
 * Return communication Watchdog state.
 * It could be usefull to reset outputs if the watchdog is fired.
 *
 * @return TRUE if millis() > u32timeOut
 * @ingroup loop
 */
bool Modbus::getTimeOutState()
{
    return ((unsigned long)(millis() -u32timeOut) > (unsigned long)u16timeOut);
}

/**
 * Get modbus master state
 *
 * @return = 0 IDLE, = 1 WAITING FOR ANSWER
 * @ingroup buffer
 */
uint8_t Modbus::getState()
{
    return u8state;
}

/**
 * @brief
 * *** Only Modbus Master ***
 * Generate a query to an slave with a modbus_t telegram structure
 * The Master must be in COM_IDLE mode. After it, its state would be COM_WAITING.
 * This method has to be called only in loop() section.
 *
 * @see modbus_t
 * @param modbus_t  modbus telegram structure (id, fct, ...)
 * @ingroup loop
 * @todo finish function 15
 */
int8_t Modbus::query( modbus_t telegram )
{
  if (u8id!=0) return -2;
  if (u8state != COM_IDLE) return -1;

  if ((telegram.u8id==0) || (telegram.u8id>247)) return -3;

  au16regs = telegram.au16reg;

  // telegram header
  au8Buffer[ ID ]         = telegram.u8id;
  au8Buffer[ FUNC ]       = telegram.u8fct;
  au8Buffer[ ADD_HI ]     = highByte(telegram.u16RegAdd );
  au8Buffer[ ADD_LO ]     = lowByte( telegram.u16RegAdd );

  switch( telegram.u8fct )
  {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
    case MB_FC_READ_REGISTERS:
    case MB_FC_READ_INPUT_REGISTER:
      au8Buffer[ NB_HI ]      = highByte(telegram.u16CoilsNo );
      au8Buffer[ NB_LO ]      = lowByte( telegram.u16CoilsNo );
      u8BufferSize = 6;
      break;
    case MB_FC_WRITE_REGISTER:
      au8Buffer[ NB_HI ]      = highByte(au16regs[0]);
      au8Buffer[ NB_LO ]      = lowByte(au16regs[0]);
      u8BufferSize = 6;
      break;
    case MB_FC_WRITE_MULTIPLE_REGISTERS:
      au8Buffer[ NB_HI ]      = highByte(telegram.u16CoilsNo );
      au8Buffer[ NB_LO ]      = lowByte( telegram.u16CoilsNo );
      au8Buffer[ BYTE_CNT ]    = (uint8_t) ( telegram.u16CoilsNo * 2 );
      u8BufferSize = 7;
      for (uint16_t i=0; i< telegram.u16CoilsNo; i++)
      {
        au8Buffer[ u8BufferSize ] = highByte( au16regs[ i ] );
        u8BufferSize++;
        au8Buffer[ u8BufferSize ] = lowByte( au16regs[ i ] );
        u8BufferSize++;
      }
      break;
  }

  sendTxBuffer();
  u8state = COM_WAITING;
  u8lastError = 0;
  return 0;
}

/**
 * @brief *** Only for Modbus Master ***
 * This method checks if there is any incoming answer if pending.
 * If there is no answer, it would change Master state to COM_IDLE.
 * This method must be called only at loop section.
 * Avoid any delay() function.
 *
 * Any incoming data would be redirected to au16regs pointer,
 * as defined in its modbus_t query telegram.
 *
 * @params	nothing
 * @return errors counter
 * @ingroup loop
 */
int8_t Modbus::poll()
{
    // check if there is any incoming frame
	uint8_t u8current;
  u8current = port->available();

  if ((unsigned long)(millis() -u32timeOut) > (unsigned long)u16timeOut)
  {
    u8state = COM_IDLE;
    u8lastError = NO_REPLY;
    u16errCnt++;
    return 0;
  }

  if (u8current == 0) return 0;

  // check T35 after frame end or still no frame end
  if (u8current != u8lastRec)
  {
    u8lastRec = u8current;
    u32time = millis();
    return 0;
  }
  if ((unsigned long)(millis() -u32time) < (unsigned long)T35) return 0;

  // transfer Serial buffer frame to auBuffer
  u8lastRec = 0;
  int8_t i8state = getRxBuffer();
  if (i8state < 6) //7 was incorrect for functions 1 and 2 the smallest frame could be 6 bytes long
  {
    u8state = COM_IDLE;
    u16errCnt++;
    return i8state;
  }

  // validate message: id, CRC, FCT, exception
  uint8_t u8exception = validateAnswer();
  if (u8exception != 0)
  {
    u8state = COM_IDLE;
    return u8exception;
  }

  // process answer
  switch( au8Buffer[ FUNC ] )
  {
  case MB_FC_READ_INPUT_REGISTER:
  case MB_FC_READ_REGISTERS :
    // call get_FC3 to transfer the incoming message to au16regs buffer
    get_FC3( );
    break;
  }
  u8state = COM_IDLE;
  return u8BufferSize;
}

/* _____PRIVATE FUNCTIONS_____________________________________________________ */

void Modbus::init(uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin)
{
  this->u8id = u8id;
  this->u8serno = u8serno;
  this->u8txenpin = u8txenpin;
  this->u16timeOut = 1000;
  this->u32overTime = 0;
}
/**
 * @brief
 * This method moves Serial buffer data to the Modbus au8Buffer.
 *
 * @return buffer size if OK, ERR_BUFF_OVERFLOW if u8BufferSize >= MAX_BUFFER
 * @ingroup buffer
 */
int8_t Modbus::getRxBuffer()
{
  bool bBuffOverflow = false;

  u8BufferSize = 0;
  while ( port->available() )
  {
    au8Buffer[ u8BufferSize ] = port->read();
    u8BufferSize ++;

    if (u8BufferSize >= MAX_BUFFER) bBuffOverflow = true;
    ESP.wdtFeed();
  }
  u16InCnt++;

  if (bBuffOverflow)
  {
    u16errCnt++;
    return ERR_BUFF_OVERFLOW;
  }
  return u8BufferSize;
}

/**
 * @brief
 * This method transmits au8Buffer to Serial line.
 * Only if u8txenpin != 0, there is a flow handling in order to keep
 * the RS485 transceiver in output state as long as the message is being sent.
 * This is done with UCSRxA register.
 * The CRC is appended to the buffer before starting to send it.
 *
 * @param nothing
 * @return nothing
 * @ingroup buffer
 */
void Modbus::sendTxBuffer()
{
  // append CRC to message
  uint16_t u16crc = calcCRC( u8BufferSize );
  au8Buffer[ u8BufferSize ] = u16crc >> 8;
  u8BufferSize++;
  au8Buffer[ u8BufferSize ] = u16crc & 0x00ff;
  u8BufferSize++;

  // transfer buffer to serial line
  delay(100);     // added because it was responding faster than the line direction could be switched
  port->write( au8Buffer, u8BufferSize );

  while(port->read() >= 0) ESP.wdtFeed();
  u8BufferSize = 0;
  // set time-out for master
  u32timeOut = millis();
  // increase message counter
  u16OutCnt++;
}

/**
 * @brief
 * This method calculates CRC
 *
 * @return uint16_t calculated CRC value for the message
 * @ingroup buffer
 */
uint16_t Modbus::calcCRC(uint8_t u8length)
{
  unsigned int temp, temp2, flag;
  temp = 0xFFFF;
  for (unsigned char i = 0; i < u8length; i++)
  {
    temp = temp ^ au8Buffer[i];
    for (unsigned char j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>=1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  // Reverse byte order.
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;
  // the returned value is already swapped
  // crcLo byte is first & crcHi byte is last
  return temp;
}

/**
 * @brief
 * This method validates master incoming messages
 *
 * @return 0 if OK, EXCEPTION if anything fails
 * @ingroup buffer
 */
uint8_t Modbus::validateAnswer()
{
  // check message crc vs calculated crc
  uint16_t u16MsgCRC =
      ((au8Buffer[u8BufferSize - 2] << 8)
       | au8Buffer[u8BufferSize - 1]); // combine the crc Low & High bytes
  if ( calcCRC( u8BufferSize-2 ) != u16MsgCRC )
  {
    u16errCnt ++;
    return NO_REPLY;
  }

  // check exception
  if ((au8Buffer[ FUNC ] & 0x80) != 0)
  {
    u16errCnt ++;
    return ERR_EXCEPTION;
  }

  // check fct code
  bool isSupported = false;
  for (uint8_t i = 0; i< sizeof( fctsupported ); i++)
  {
    if (fctsupported[i] == au8Buffer[FUNC])
    {
      isSupported = 1;
      break;
    }
  }
  if (!isSupported)
  {
    u16errCnt ++;
    return EXC_FUNC_CODE;
  }
  return 0; // OK, no exception code thrown
}

/**
 * This method processes functions 3 & 4 (for master)
 * This method puts the slave answer into master data buffer
 *
 * @ingroup register
 */
void Modbus::get_FC3()
{
  uint8_t u8byte, i;
  u8byte = 3;

  for (i=0; i< au8Buffer[ 2 ] /2; i++)
  {
    au16regs[ i ] = word(au8Buffer[ u8byte ],au8Buffer[ u8byte +1 ]);
    u8byte += 2;
  }
}
