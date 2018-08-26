// CoolDumper на Arduino MEGA2560
// Версия 1.0 бета, дампинг проверен на CoolBoyб 32МБ без ошибок
// Нужно проверить запись

// Дефайны и опции
#define F_CPU 16000000UL
//#define F_CPU 11059200UL
//#define UART_BAUD 38400UL
#define UART_BAUD 250000UL
#define RECV_BUFFER 1050
//#define SEND_DELAY 100

#define LED_RED_ON PORTB |= (1<<7)
#define LED_RED_OFF PORTB &= ~(1<<7)
#define LED_GREEN_ON PORTB |= (1<<6)
#define LED_GREEN_OFF PORTB &= ~(1<<6)
#define ROMSEL_HI PORTF |= (1<<1)
#define ROMSEL_LOW PORTF &= ~(1<<1)
#define PHI2_HI PORTF |= (1<<0)
#define PHI2_LOW PORTF &= ~(1<<0)
#define MODE_READ { PORTL = 0xFF; DDRL = 0; }
#define MODE_WRITE DDRL = 0xFF
#define PRG_READ PORTF |= (1<<7)
#define PRG_WRITE PORTF &= ~(1<<7)
#define CHR_READ_HI PORTF |= (1<<5)
#define CHR_READ_LOW PORTF &= ~(1<<5)
#define CHR_WRITE_HI PORTF |= (1<<2)
#define CHR_WRITE_LOW PORTF &= ~(1<<2)

#define JTAG_PORT B
#define TMS_PIN 2
#define TCK_PIN 4
#define TDO_PIN 3
#define TDI_PIN 1

#define COMMAND_PRG_STARTED 0
#define COMMAND_CHR_STARTED 1
#define COMMAND_ERROR_INVALID 2
#define COMMAND_ERROR_CRC 3
#define COMMAND_ERROR_OVERFLOW 4
#define COMMAND_PRG_INIT 5
#define COMMAND_CHR_INIT 6
#define COMMAND_PRG_READ_REQUEST 7
#define COMMAND_PRG_READ_RESULT 8
#define COMMAND_PRG_WRITE_REQUEST 9
#define COMMAND_PRG_WRITE_DONE 10
#define COMMAND_CHR_READ_REQUEST 11
#define COMMAND_CHR_READ_RESULT 12
#define COMMAND_CHR_WRITE_REQUEST 13
#define COMMAND_CHR_WRITE_DONE 14
#define COMMAND_PHI2_INIT 15
#define COMMAND_PHI2_INIT_DONE 16
#define COMMAND_MIRRORING_REQUEST 17
#define COMMAND_MIRRORING_RESULT 18
#define COMMAND_RESET 19
#define COMMAND_RESET_ACK 20
#define COMMAND_PRG_EPROM_WRITE_REQUEST 21
#define COMMAND_CHR_EPROM_WRITE_REQUEST 22
#define COMMAND_EPROM_PREPARE 23
#define COMMAND_PRG_FLASH_ERASE_REQUEST 24
#define COMMAND_PRG_FLASH_WRITE_REQUEST 25
#define COMMAND_CHR_FLASH_ERASE_REQUEST 26
#define COMMAND_CHR_FLASH_WRITE_REQUEST 27
#define COMMAND_JTAG_SETUP 28
#define COMMAND_JTAG_SHUTDOWN 29
#define COMMAND_JTAG_EXECUTE 30
#define COMMAND_JTAG_RESULT 31
#define COMMAND_TEST_SET 32
#define COMMAND_TEST_RESULT 33
#define COMMAND_COOLBOY_READ_REQUEST 34
#define COMMAND_COOLBOY_ERASE_REQUEST 35
#define COMMAND_COOLBOY_WRITE_REQUEST 36
#define COMMAND_COOLGIRL_ERASE_SECTOR_REQUEST 37
#define COMMAND_COOLGIRL_WRITE_REQUEST 38
#define COMMAND_PRG_CRC_READ_REQUEST 39
#define COMMAND_CHR_CRC_READ_REQUEST 40

//define COMMAND_BOOTLOADER 0xFE
#define COMMAND_DEBUG 0xFF

//=====================================================================
// UART
void USART_init( void )
{
  unsigned int bd = (F_CPU / (16UL * UART_BAUD)) - 1;
  UBRR0L = bd & 0xFF;
  UBRR0H = bd >> 8;

  UCSR0B = _BV(TXEN0) | _BV(RXEN0) | _BV(RXCIE0); /* tx/rx enable */
//  UCSRC = 1<<URSEL|1<<UCSZ0|1<<UCSZ1;
  UCSR0C |= /*_BV(UMSEL0) |*/ _BV(UCSZ01) | _BV(UCSZ00);
  //UCSRA = _BV(U2X);
}
void USART_TransmitByte( unsigned char data )
{
  /* Wait for empty transmit buffer */
  while ( !( UCSR0A & (1<<UDRE0)) );
  /* Put data into buffer, sends the data */
  UDR0 = data;
}
void USART_TransmitHex( unsigned char data )
{
  unsigned char h = data>>4;
  char ho = (h < 10) ? (h+'0') : (h+'A'-10);
  unsigned char l = data & 0xF;
  char lo = (l < 10) ? (l+'0') : (l+'A'-10);
  while ( !( UCSR0A & (1<<UDRE0)) );
  UDR0 = ho;
  while ( !( UCSR0A & (1<<UDRE0)) );
  UDR0 = lo;
}
void USART_TransmitText(char* data)
{
  while (*data != 0)
  {
    /* Wait for empty transmit buffer */
    while ( !( UCSR0A & (1<<UDRE0)) );
    /* Put data into buffer, sends the data */
    UDR0 = *data;
    data++;
  }
}
void USART_Transmit(void* p, unsigned long int len)
{
  unsigned char* buff = (unsigned char*)p;
  unsigned long int b;
  for (b = 0; b < len; b++) USART_TransmitByte(buff[b]);
}

//=====================================================================
// JTAG
#define JTAG_PULSE_TCK_DELAY 0xFD
// Дальше 1 байт: 0бит - TMS, 1бит - нужно ли дёргать TCK, 2бит - жужна ли задержка. Если нужно дёргать TCK, потом 4 байта - сколько раз. Если нужна задержка - ещё 4 байта usec
#define JTAG_PULSE_TCK_MULTI 0xFE
// Дальше 1 байт: кол-во элементов. Каждый элемент это либо:
  // число 1-127 означающее кол-во повторений и байт: 0бит - TMS, 1бит изменять ли TDI, 2бит значение TDI, 3бит - проверять ли TDO, 4бит значение TDO
  // Либо 7бит равный единице и 1бит изменять ли TDI, 2бит значение TDI, 3бит - проверять ли TDI, 4бит значение TDO
#define CLUJTAG_CONCAT(a, b)            a ## b
#define CLUJTAG_OUTPORT(name)           CLUJTAG_CONCAT(PORT, name)
#define CLUJTAG_INPORT(name)            CLUJTAG_CONCAT(PIN, name)
#define CLUJTAG_DDRPORT(name)           CLUJTAG_CONCAT(DDR, name)
#define PORT CLUJTAG_OUTPORT(JTAG_PORT)
#define PORT_DDR CLUJTAG_DDRPORT(JTAG_PORT)
#define PORT_PIN CLUJTAG_INPORT(JTAG_PORT)
#define PORT_LED CLUJTAG_OUTPORT(LED_PORT)
#define PORT_LED_DDR CLUJTAG_DDRPORT(LED_PORT)
//
static uint8_t jtag_current_command = 0;
static uint16_t jtag_pos = 0;
static uint32_t jtag_tck_delay_value = 0;
static uint32_t jtag_num_tck = 0;
static uint32_t jtag_usecs = 0;
static uint16_t jtag_multi_count = 0;
static uint16_t jtag_multi_pos = 0;
static uint16_t jtag_multi_multi = 0;
//
static void tck_delay(uint8_t tms, uint32_t num_tck, uint32_t usecs)
{
  if (tms) 
    PORT |= (1<<TMS_PIN);
  else
    PORT &= ~(1<<TMS_PIN);
  PORT |= 1<<TCK_PIN;
  int i;
  for (i = 0; i < num_tck; i++)
  {
    PORT ^= 1<<TCK_PIN;
    //_delay_us(1);
    PORT ^= 1<<TCK_PIN;
    //_delay_us(1);
  }
  for (i = 0; i < usecs; i++)
    _delay_us(1);
}
static uint8_t pulse_tck(int tms, int tdi, int tdo)
{
  if (tms)
    PORT |= (1<<TMS_PIN);
  else
    PORT &= ~(1<<TMS_PIN);
  if (tdi >= 0)
  {
    if (tdi)
      PORT |= (1<<TDI_PIN);
    else
      PORT &= ~(1<<TDI_PIN);
  }
  PORT &= ~(1<<TCK_PIN);
  //_delay_us(1);
  PORT |= 1<<TCK_PIN;
  //_delay_us(1);
  if (tdo < 0) return 1;
  return ((PORT_PIN >> TDO_PIN) & 1) == tdo;
}
static int jtag_parse_byte(uint8_t data)
{
  int i, tms, tdi, tdo;
  switch (jtag_current_command)
  {
    case 0:
      jtag_current_command = data;
      jtag_pos = 0;
      break;
      
    case JTAG_PULSE_TCK_DELAY:
      if (jtag_pos == 0)
      {
        jtag_num_tck = 0;
        jtag_usecs = 0;
        jtag_tck_delay_value = data;
      } else {
        if ((jtag_pos < 4) && (jtag_tck_delay_value & 0b10)) // num_tck
        {
          jtag_num_tck |= (uint32_t)data << (8*(jtag_pos-1));
        } else if ((jtag_pos < 4) && !(jtag_tck_delay_value & 0b10)) // usecs
        {
          jtag_usecs |= (uint32_t)data << (8*(jtag_pos-1));
        } else {
          jtag_usecs |= (uint32_t)data << (8*(jtag_pos-5));
        }
      }
      if ( ((jtag_tck_delay_value & 0b110) == 0)
        || (((((jtag_tck_delay_value & 0b110) == 0b100) || (jtag_tck_delay_value & 0b110) == 0b010)) && jtag_pos == 4)
        || (((jtag_tck_delay_value & 0b110) == 0b110) && jtag_pos == 8)
      )
      {
        tck_delay(jtag_tck_delay_value&1, jtag_num_tck, jtag_usecs);
        jtag_current_command = 0;     
      }
      jtag_pos++;     
      break;
      
    case JTAG_PULSE_TCK_MULTI:
      if (jtag_pos == 0)
      {
        jtag_multi_count = data;
        jtag_multi_pos = 0;
        jtag_multi_multi = 0;
      } else if (jtag_pos == 1)
      {
        jtag_multi_count |= (uint16_t)data << 8;
      } else {
        if (!jtag_multi_multi && !(data & 0x80))
        {
          jtag_multi_multi = data;
        } else {
          if (!jtag_multi_multi) jtag_multi_multi = 1;
          tms = data&1;
          tdi = (data>>1)&1;
          tdo = -1;
          if (data & (1<<2))
          {
            tdo = (data>>3)&1;
          }
          for (i = 0; i < jtag_multi_multi; i++)
          {
            if (!pulse_tck(tms, tdi, tdo)) return 0;
            jtag_multi_pos++;
          }
          jtag_multi_multi = 0;
        }
        if (jtag_multi_pos >= jtag_multi_count)
          jtag_current_command = 0;
      }
      jtag_pos++;
      break;
      
    /*
    case JTAG_PULSE_TCK:
      tms = data&1;
      tdi = (data>>1)&1;
      tdo = -1;
      if (data & (1<<2))
      {
        tdo = (data>>3)&1;
      }
      if (!pulse_tck(tms, tdi, tdo)) return 0;
      jtag_current_command = 0;
      break;
    */
    default:
      return 0;
  }
  return 1;
}
void jtag_setup()
{
  PORT &= ~((1<<TMS_PIN) | (1<<TDO_PIN) | (1<<TDI_PIN)) | (1<<TCK_PIN);
  PORT_DDR |= (1<<TMS_PIN) | (1<<TCK_PIN) | (1<<TDI_PIN);
  PORT_DDR &= ~(1<<TDO_PIN);
  jtag_current_command = 0;
}
void jtag_shutdown()
{
  PORT &= ~((1<<TMS_PIN) | (1<<TCK_PIN) | (1<<TDO_PIN) | (1<<TDI_PIN));
  PORT_DDR &= ~((1<<TMS_PIN) | (1<<TCK_PIN) | (1<<TDO_PIN) | (1<<TDI_PIN));
}
int jtag_execute(int count, uint8_t* data)
{
  int i;
  for (i = 0; i < count; i++)
  {
    if (!jtag_parse_byte(data[i]))
    {
      jtag_shutdown();
      return 0;
    }
  }
  return 1;
}

//=====================================================================
// Коммуникации
static uint8_t comm_send_crc;
static unsigned int comm_send_length;
static unsigned int comm_send_pos;

static int comm_recv_pos;
static uint8_t comm_recv_crc;
static uint8_t comm_recv_error;

volatile uint8_t comm_recv_command;
volatile unsigned int comm_recv_length;
volatile uint8_t recv_buffer[RECV_BUFFER];
volatile uint8_t comm_recv_done;

static void comm_calc_send_crc(uint8_t inbyte)
{
  uint8_t j;
  for (j=0;j<8;j++) 
  {
    uint8_t mix = (comm_send_crc ^ inbyte) & 0x01;
    comm_send_crc >>= 1;
    if (mix) 
      comm_send_crc ^= 0x8C;                  
    inbyte >>= 1;
  }
}
static void comm_calc_recv_crc(uint8_t inbyte)
{
  uint8_t j;
  for (j=0;j<8;j++) 
  {
    uint8_t mix = (comm_recv_crc ^ inbyte) & 0x01;
    comm_recv_crc >>= 1;
    if (mix) 
      comm_recv_crc ^= 0x8C;                  
    inbyte >>= 1;
  }
}
static void comm_send_and_calc(uint8_t data)
{
  comm_calc_send_crc(data);
  USART_TransmitByte(data);
#ifdef SEND_DELAY
  _delay_us(SEND_DELAY);
#endif
}
void comm_start(uint8_t command, unsigned int length)
{
  comm_send_crc = 0;
  comm_send_and_calc('F');
  comm_send_and_calc(command);
  comm_send_and_calc(length & 0xff);
  comm_send_and_calc((length >> 8) & 0xff);
  comm_send_length = length;
  comm_send_pos = 0;

  if (!comm_send_length)
    USART_TransmitByte(comm_send_crc);
}
void comm_send_byte(uint8_t data)
{
  comm_send_and_calc(data);
  comm_send_pos++;
  
  if (comm_send_pos == comm_send_length)
    USART_TransmitByte(comm_send_crc);
}
void comm_proceed(uint8_t data)
{
  if (comm_recv_error && data != 'F') return;
  comm_recv_error = 0;
  if (!comm_recv_pos)
  {
    comm_recv_crc = 0;
    comm_recv_done = 0;
  }
  comm_calc_recv_crc(data);
  unsigned int l = comm_recv_pos-4;
  switch (comm_recv_pos)
  {
    case 0:
      if (data != 'F')
      {
        comm_recv_error = 1;
        comm_start(COMMAND_ERROR_INVALID, 0);
      }
      break;
    case 1:
      comm_recv_command = data;
      break;
    case 2:
      comm_recv_length = data;
      break;
    case 3:
      comm_recv_length |= (uint16_t)data << 8;
      break;
    default:
      if (l >= RECV_BUFFER)
      {
        comm_recv_pos = 0;
        comm_recv_error = 1;
        comm_start(COMMAND_ERROR_OVERFLOW, 0);
        return;
      }     
      else if (l < comm_recv_length)
      {
        recv_buffer[l] = data;        
      } else if (l == comm_recv_length)
      {
        if (!comm_recv_crc)
        {
          comm_recv_done = 1;
        } 
        else
        {
          comm_recv_error = 1;
          comm_start(COMMAND_ERROR_CRC, 0);
        }
        comm_recv_pos = 0;
        return;
      }
      break;
  }
  comm_recv_pos++;
}
void comm_init()
{
  comm_recv_pos = 0;
  comm_recv_done = 0; 
  comm_recv_error = 0;
}

//=====================================================================
// Код дампера

ISR(USART0_RX_vect)
{
  unsigned char b;
  while (UCSR0A & (1<<RXC0))
  {
    b = UDR0;
    comm_proceed(b);
  }
}

static void phi2_init()
{
  int i = 0x80;
  unsigned char h = PORTF |= (1<<0);
  unsigned char l = PORTF &= ~(1<<0);
  while(i != 0){
    PORTA = l;
    PORTA = h;
    i--;
  }
}

static void set_address(unsigned int address)
{
  unsigned char l = address & 0xFF;
  unsigned char h = address>>8;
  
  PORTA = l;
  PORTC = h;
  
  // PPU /A13
  if ((address >> 13) & 1)
    PORTF &= ~(1<<4);
  else
    PORTF |= 1<<4;
}

static void set_romsel(unsigned int address)
{
  if (address & 0x8000)
  {
    ROMSEL_LOW;
  } else {
    ROMSEL_HI;
  }
}

static unsigned char read_prg_byte(unsigned int address)
{
  MODE_READ;
  PRG_READ; 
  set_address(address);
  PHI2_HI;
  set_romsel(address);
  _delay_us(1);
  return PINL;
}

static unsigned char read_chr_byte(unsigned int address)
{
  MODE_READ;
  PHI2_HI;
  ROMSEL_HI;
  set_address(address);
  CHR_READ_LOW;
  
  _delay_us(1);
  
  uint8_t result = PINL;
  
  CHR_READ_HI;
  return result;
}

static unsigned char read_coolboy_byte(unsigned int address)
{
  MODE_READ;
  PRG_READ; 
  set_address(address);
  PHI2_HI;
  ROMSEL_LOW;
  PORTB |= 1<<TDO_PIN;
  PORTB &= ~(1<<TCK_PIN);
  _delay_us(1);
  return PINL;
}

static void read_prg_send(unsigned int address, unsigned int len)
{
  LED_GREEN_ON;
  comm_start(COMMAND_PRG_READ_RESULT, len);
  while (len > 0)
  {
    comm_send_byte(read_prg_byte(address));
    len--;
    address++;
  }
  set_address(0);
  PHI2_HI;
  ROMSEL_HI;
  LED_GREEN_OFF;
}

static void read_chr_send(unsigned int address, unsigned int len)
{
  LED_GREEN_ON;
  comm_start(COMMAND_CHR_READ_RESULT, len);
  while (len > 0)
  {
    comm_send_byte(read_chr_byte(address));
    len--;
    address++;
  }
  set_address(0);
  PHI2_HI;
  ROMSEL_HI;
  LED_GREEN_OFF;
}

static uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  int i;
  crc ^= a;
  for (i = 0; i < 8; ++i)
  {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc = (crc >> 1);
  }
  return crc;
}

static void read_prg_crc_send(unsigned int address, unsigned int len)
{
  LED_GREEN_ON;
  uint16_t crc = 0;
  read_prg_byte(address);
  while (len > 0)
  {
    unsigned char l = address & 0xFF;
    unsigned char h = address>>8; 
    PORTA = l;
    PORTC = h;
    _delay_us(1);
    crc = crc16_update(crc, PINL);
    len--;
    address++;
  }
  set_address(0);
  PHI2_HI;
  ROMSEL_HI;
  comm_start(COMMAND_PRG_READ_RESULT, 2);
  comm_send_byte(crc & 0xFF);
  comm_send_byte((crc >> 8) & 0xFF);
  LED_GREEN_OFF;
}

static void read_chr_crc_send(unsigned int address, unsigned int len)
{
  LED_GREEN_ON;
  uint16_t crc = 0;
  while (len > 0)
  {
    crc = crc16_update(crc, read_chr_byte(address));
    len--;
    address++;
  }
  set_address(0);
  PHI2_HI;
  ROMSEL_HI;
  comm_start(COMMAND_CHR_READ_RESULT, 2);
  comm_send_byte(crc & 0xFF);
  comm_send_byte((crc >> 8) & 0xFF);
  LED_GREEN_OFF;
}

static void read_coolboy_send(unsigned int address, unsigned int len)
{
  LED_GREEN_ON;
  PORTB |= 1<<TCK_PIN;
  PORTB |= 1<<TDO_PIN;
  DDRB |= 1<<TCK_PIN;
  DDRB |= 1<<TDO_PIN;
  comm_start(COMMAND_PRG_READ_RESULT, len);
  while (len > 0)
  {
    comm_send_byte(read_coolboy_byte(address));
    len--;
    address++;
  }
  set_address(0);
  ROMSEL_HI;
  PORTB |= 1<<TCK_PIN;
  PORTB |= 1<<TDO_PIN;
  
  jtag_shutdown();
  LED_GREEN_OFF;
}

static void write_prg_byte(unsigned int address, uint8_t data)
{
  PHI2_LOW;
  ROMSEL_HI;
  MODE_WRITE;
  PRG_WRITE;
  PORTL = data;
  set_address(address); // PHI2 low, ROMSEL always HIGH
//  _delay_us(1);
  
  PHI2_HI;
  //_delay_us(10);
  set_romsel(address); // ROMSEL is low if need, PHI2 high  
  
  _delay_us(1); // WRITING
  //_delay_ms(1); // WRITING
  
  // PHI2 low, ROMSEL high
  PHI2_LOW;
  _delay_us(1);
  ROMSEL_HI;
  
  // Back to read mode
//  _delay_us(1);
  PRG_READ;
  MODE_READ;
  set_address(0);

  // Set phi2 to high state to keep cartridge unreseted
//  _delay_us(1);
  PHI2_HI;

//  _delay_us(1);
}

static void write_chr_byte(unsigned int address, uint8_t data)
{
  PHI2_LOW;
  ROMSEL_HI;
  MODE_WRITE;
  PORTL = data; 
  set_address(address); // PHI2 low, ROMSEL always HIGH
  //_delay_us(10);
  
  CHR_WRITE_LOW;
    
  _delay_us(1); // WRITING
  //_delay_ms(1); // WRITING
  
  CHR_WRITE_HI;
  
  //_delay_us(1);
  
  MODE_READ;
  set_address(0);
  PHI2_HI;
  
  //_delay_us(1);
}

static void write_prg(unsigned int address, unsigned int len, uint8_t* data)
{
  LED_RED_ON;
  while (len > 0)
  {
    write_prg_byte(address, *data);
    address++;
    len--;
    data++;
  }
  //_delay_ms(1);
  LED_RED_OFF;
}

static void write_chr(unsigned int address, unsigned int len, uint8_t* data)
{
  LED_RED_ON;
  while (len > 0)
  {
    write_chr_byte(address, *data);
    address++;
    len--;
    data++;
  }
  //_delay_ms(1);
  LED_RED_OFF;
}

static void write_prg_flash_command(unsigned int address, uint8_t data)
{
  write_prg_byte(address | 0x8000, data);
}

static void write_coolboy_flash_command(unsigned int address, uint8_t data)
{
  PORTB |= 1<<TCK_PIN;
  PORTB |= 1<<TDO_PIN;
  ROMSEL_HI;
  PRG_READ; 
  set_address(address);
  MODE_WRITE;
  PORTL = data;
  PHI2_HI;
  ROMSEL_LOW;
  _delay_us(1);
  
  PORTB &= ~(1<<TDO_PIN);
  
  _delay_us(1);

  PORTB |= 1<<TDO_PIN;
  set_address(0);
  ROMSEL_HI;
  MODE_READ;
}

static void write_chr_flash_command(unsigned int address, uint8_t data)
{
  write_chr_byte(address, data);
}

static int write_prg_flash_byte(unsigned int address, uint8_t data)
{
  write_prg_flash_command(0x0000, 0xF0);
  write_prg_flash_command(0x0555, 0xAA);
  write_prg_flash_command(0x02AA, 0x55);
  write_prg_flash_command(0x0555, 0xA0);
  
  write_prg_flash_command(address, data);
  _delay_us(50);

  int timeout = 0;
  uint8_t res, last_res = 0;
  while (timeout < 200)
  {
    res = read_prg_byte(address | 0x8000);
    ROMSEL_HI;
    if (res == last_res && last_res == data) break;
    last_res = res;
    _delay_us(50);
    timeout++;
  }
  //PHI2_LOW;
  ROMSEL_HI;
  set_address(0);
  
  return timeout < 10;
}


static int write_chr_flash_byte(unsigned int address, uint8_t data)
{
  write_chr_flash_command(0x0000, 0xF0);
  write_chr_flash_command(0x0555, 0xAA);
  write_chr_flash_command(0x02AA, 0x55);
  write_chr_flash_command(0x0555, 0xA0);
  write_chr_flash_command(address, data);

  int timeout = 0;
  while (read_chr_byte(address | 0x8000) != data && timeout < 10)
  {
    _delay_us(100);
    timeout++;
  }
  set_address(0);
  PHI2_LOW;
  ROMSEL_HI;
  
  return timeout < 10;
}

static int erase_prg_flash()
{
  LED_RED_ON;
  write_prg_flash_command(0x0000, 0xF0);
  write_prg_flash_command(0x0AAA, 0xAA);
  write_prg_flash_command(0x0555, 0x55);
  write_prg_flash_command(0x0AAA, 0x80);
  write_prg_flash_command(0x0AAA, 0xAA);
  write_prg_flash_command(0x0555, 0x55);
  write_prg_flash_command(0x0AAA, 0x10);
  
  int timeout = 0;
  while ((read_prg_byte(0x8000) != 0xFF) && (timeout < 300000))
  {
    _delay_ms(1);
    timeout++;
  }
  set_address(0);
  PHI2_HI;
  ROMSEL_HI;

  LED_RED_OFF;
  return timeout < 300000;
}

static int erase_coolboy_sector()
{
  LED_RED_ON;
  PORTB |= 1<<TCK_PIN;
  PORTB |= 1<<TDO_PIN;
  DDRB |= 1<<TCK_PIN;
  DDRB |= 1<<TDO_PIN;
  ROMSEL_HI;

  write_coolboy_flash_command(0x0000, 0xF0);
  write_coolboy_flash_command(0x0AAA, 0xAA);
  write_coolboy_flash_command(0x0555, 0x55);
  write_coolboy_flash_command(0x0AAA, 0x80);
  write_coolboy_flash_command(0x0AAA, 0xAA);
  write_coolboy_flash_command(0x0555, 0x55);
  write_coolboy_flash_command(0x0000, 0x30);  
  
  int timeout = 0;
  uint8_t debug;
  while (((debug = read_coolboy_byte(0x8000)) != 0xFF) && (timeout < 3000))
  {
    //comm_start(0xFF, 1);
    //comm_send_byte(debug);
    _delay_ms(1);
    timeout++;
  }
  
  set_address(0);
  ROMSEL_HI;

  jtag_shutdown();
  LED_RED_OFF;
  return timeout < 3000;
}

static int erase_coolgirl_sector()
{
  LED_RED_ON;
  write_prg_flash_command(0x0000, 0xF0);
  write_prg_flash_command(0x0AAA, 0xAA);
  write_prg_flash_command(0x0555, 0x55);
  write_prg_flash_command(0x0AAA, 0x80);
  write_prg_flash_command(0x0AAA, 0xAA);
  write_prg_flash_command(0x0555, 0x55);
  write_prg_flash_command(0x0000, 0x30);
  
  int timeout = 0;
  uint8_t debug;
  while (((debug = read_prg_byte(0x8000)) != 0xFF) && (timeout < 3000))
  {
    //comm_start(0xFF, 1);
    //comm_send_byte(debug);
    _delay_ms(1);
    timeout++;
  }
  set_address(0);
  PHI2_HI;
  ROMSEL_HI;

  LED_RED_OFF;
  return timeout < 3000;
}

static int erase_chr_flash()
{
  LED_RED_ON;
  write_chr_flash_command(0x0000, 0xF0);
  write_chr_flash_command(0x0555, 0xAA);
  write_chr_flash_command(0x02AA, 0x55);
  write_chr_flash_command(0x0555, 0x80);
  write_chr_flash_command(0x0555, 0xAA);
  write_chr_flash_command(0x02AA, 0x55);
  write_chr_flash_command(0x0555, 0x10);
  
  int timeout = 0;
  while ((read_chr_byte(0) != 0xFF) && (timeout < 10000))
  {
    _delay_ms(1);
    timeout++;
  }
  set_address(0);
  PHI2_HI;
  ROMSEL_HI;

  LED_RED_OFF;
  return timeout < 10000;
}

static int write_prg_flash(unsigned int address, unsigned int len, uint8_t* data)
{
  LED_RED_ON;
  int ok = 1;
  while (len > 0)
  {
    if (!write_prg_flash_byte(address, *data))
    {
      ok = 0;
      break;
    }
    address++;
    len--;
    data++;
  }
  LED_RED_OFF;
  return ok;
}

static int write_coolboy(unsigned int address, unsigned int len, uint8_t* data)
{
  LED_RED_ON;
  PORTB |= 1<<TCK_PIN;
  PORTB |= 1<<TDO_PIN;
  DDRB |= 1<<TCK_PIN;
  DDRB |= 1<<TDO_PIN;
  ROMSEL_HI;
  uint8_t ok = 1;
  while (len > 0)
  {
    
    //uint8_t count = len > 16 ? 16 : len;
    uint8_t count = 0;
    uint8_t* d = data;
    unsigned int a = address;
    unsigned int address_base = a & 0xFFE0;
    while (len > 0 && ((a & 0xFFE0) == address_base))
    {
      if (*d != 0xFF) count++;
      a++;
      len--;
      d++;
    }

    if (count)
    {
      //write_prg_flash_command(0x0000, 0xF0);
      write_coolboy_flash_command(0x0AAA, 0xAA);
      write_coolboy_flash_command(0x0555, 0x55);
      write_coolboy_flash_command(0x0000, 0x25);
      write_coolboy_flash_command(0x0000, count-1);

      while (count > 0)
      {
        if (*data != 0xFF)
        {
          write_coolboy_flash_command(address, *data);
          count--;
        }
        address++;
        data++;
      }
    
      write_coolboy_flash_command(0x0000, 0x29);
      _delay_us(10);

      long int timeout = 0;
      uint8_t res, last_res = 0;
      while (timeout < 100000)
      {
        res = read_coolboy_byte((address-1) | 0x8000);
        ROMSEL_HI;
        if (res == last_res && last_res == *(data-1)) break;
        last_res = res;
        _delay_us(10);
        timeout++;
      }
      if (timeout >= 100000)
      {
        ok = 0;
        break;
      }
    }
    
    address = a;
    data = d;
  }
  ROMSEL_HI;
  jtag_shutdown();
  LED_RED_OFF;
  return ok;
}

static int write_coolgirl(unsigned int address, unsigned int len, uint8_t* data)
{
  LED_RED_ON;
  ROMSEL_HI;
  uint8_t ok = 1;
  while (len > 0)
  {
    
    //uint8_t count = len > 16 ? 16 : len;
    uint8_t count = 0;
    uint8_t* d = data;
    unsigned int a = address;
    unsigned int address_base = a & 0xFFE0;
    while (len > 0 && ((a & 0xFFE0) == address_base))
    {
      if (*d != 0xFF) count++;
      a++;
      len--;
      d++;
    }

    if (count)
    {
      //write_prg_flash_command(0x0000, 0xF0);
      write_prg_flash_command(0x0AAA, 0xAA);
      write_prg_flash_command(0x0555, 0x55);
      write_prg_flash_command(0x0000, 0x25);
      write_prg_flash_command(0x0000, count-1);

      while (count > 0)
      {
        if (*data != 0xFF)
        {
          write_prg_flash_command(address, *data);
          count--;
        }
        address++;
        data++;
      }
    
      write_prg_flash_command(0x0000, 0x29);
      _delay_us(10);

      long int timeout = 0;
      uint8_t res, last_res = 0;
      while (timeout < 100000)
      {
        res = read_prg_byte((address-1) | 0x8000);
        ROMSEL_HI;
        if (res == last_res && last_res == *(data-1)) break;
        last_res = res;
        _delay_us(10);
        timeout++;
      }
      if (timeout >= 100000)
      {
        ok = 0;
        break;
      }
    }
    
    address = a;
    data = d;
  }
  ROMSEL_HI;
  LED_RED_OFF;
  return ok;
}

static int write_chr_flash(unsigned int address, unsigned int len, uint8_t* data)
{
  LED_RED_ON;
  if (address >= 0x8000) address -= 0x8000;
  int ok = 1;
  while (len > 0)
  {
    if (!write_chr_flash_byte(address, *data))
    {
      ok = 0;
      break;
    }
    address++;
    len--;
    data++;
  }
  LED_RED_OFF;
  return ok;
}

void get_mirroring()
{
  comm_start(COMMAND_MIRRORING_RESULT, 4);
  LED_GREEN_ON;
  set_address(0);
  _delay_us(1);
  comm_send_byte(PIND & 1);
  set_address(1<<10);
  _delay_us(1);
  comm_send_byte(PIND & 1);
  set_address(1<<11);
  _delay_us(1);
  comm_send_byte(PIND & 1);
  set_address((1<<10) | (1<<11));
  _delay_us(1);
  comm_send_byte(PIND & 1);
  set_address(0);
}

static void init_ports()
{
  DDRB |= (1 << 6) | (1 << 7); // LEDS
  DDRF = 0b10110111; // CPU R/W, IRQ, PPU /RD, PPU /A13, CIRAM /CE, PPU /WR, /ROMSEL, PHI2
  PORTF = 0b11111111; // CPU R/W, IRQ, PPU /RD, PPU /A13, CIRAM /CE, PPU /WR, /ROMSEL, PHI2 
  DDRD &= ~(1<<0); // CIRAM A10
  PORTD |= 1<<0; // CIRAM A10
  MODE_READ;
  set_address(0);
  DDRA = 0xFF; // Address low 
  DDRC = 0xFF; // Address high
}

static void reset_phi2()
{
  LED_RED_ON;
  LED_GREEN_ON;
  PHI2_LOW;
  ROMSEL_HI;
  _delay_ms(100);
  PHI2_HI;
  LED_RED_OFF;
  LED_GREEN_OFF;
}

//====================================
// Глобали
volatile unsigned long int t;
volatile char led_down;
volatile int led_bright;

void setup()
{
  sei();
  USART_init();
  init_ports();
  jtag_shutdown();

  LED_RED_OFF;
  LED_GREEN_OFF;  

  t = 0;
  led_down = 0;
  led_bright = 0;

  comm_init();
  comm_start(COMMAND_PRG_STARTED, 0);  
}

//=====================================================================
void loop()
{
  uint16_t address;
  uint16_t length;

    TCCR1A |= (1<<COM1C1) | (1<<COM1B1) | (1<<WGM10);
    TCCR1B |= (1<<CS10);
    if (t++ >= 5000)
    {
      if (!led_down)
      {
        led_bright++;
        if (led_bright >= 110) led_down = 1;
      } else {
        led_bright--;
        if (!led_bright) led_down = 0;
      }
      if (led_bright >= 100) OCR1B = led_bright - 100;
      if (led_down)
      {
        int led_bright2 = 110-led_bright;
        if (led_bright2 <= 20)
        {
          if (led_bright2 > 10) led_bright2 = 20 - led_bright2;
          OCR1C = led_bright2*2;
        }
      }
      t = 0;
    }
    
    if (comm_recv_done)
    {
      comm_recv_done = 0;
      t = led_down = led_bright = 0;
      TCCR1A = OCR1B = OCR1C = 0;
      
      switch (comm_recv_command)
      {
        case COMMAND_PRG_INIT:
          comm_start(COMMAND_PRG_STARTED, 0);
          break;
          
        case COMMAND_PRG_READ_REQUEST:
          address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
          length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
          read_prg_send(address, length);
          break;

        case COMMAND_PRG_CRC_READ_REQUEST:
          address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
          length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
          read_prg_crc_send(address, length);
          break;

        case COMMAND_PRG_WRITE_REQUEST:
          address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
          length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
          write_prg(address, length, (uint8_t*)&recv_buffer[4]);
          comm_start(COMMAND_PRG_WRITE_DONE, 0);
          break;

        case COMMAND_COOLBOY_READ_REQUEST:
          address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
          length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
          read_coolboy_send(address, length);
          break;

        case COMMAND_PHI2_INIT:
          phi2_init();
          comm_start(COMMAND_PHI2_INIT_DONE, 0);
          break;

        case COMMAND_RESET:
          reset_phi2();
          comm_start(COMMAND_RESET_ACK, 0);
          break;
          
        case COMMAND_PRG_FLASH_ERASE_REQUEST:
          if (erase_prg_flash())
            comm_start(COMMAND_PRG_WRITE_DONE, 0);
          break;

        case COMMAND_PRG_FLASH_WRITE_REQUEST:
          address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
          length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
          if (write_prg_flash(address, length, (uint8_t*)&recv_buffer[4]))
            comm_start(COMMAND_PRG_WRITE_DONE, 0);
          break;
          
        case COMMAND_COOLBOY_ERASE_REQUEST:
          if (erase_coolboy_sector())
            comm_start(COMMAND_PRG_WRITE_DONE, 0);
          break;

        case COMMAND_COOLBOY_WRITE_REQUEST:
          address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
          length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
          if (write_coolboy(address, length, (uint8_t*)&recv_buffer[4]))
            comm_start(COMMAND_PRG_WRITE_DONE, 0);
          break;
          
        case COMMAND_COOLGIRL_ERASE_SECTOR_REQUEST:
          if (erase_coolgirl_sector())
            comm_start(COMMAND_PRG_WRITE_DONE, 0);
          break;

        case COMMAND_COOLGIRL_WRITE_REQUEST:
          address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
          length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
          if (write_coolgirl(address, length, (uint8_t*)&recv_buffer[4]))
            comm_start(COMMAND_PRG_WRITE_DONE, 0);
          break;
          
        case COMMAND_CHR_INIT:
          comm_start(COMMAND_CHR_STARTED, 0);
          break;
          
        case COMMAND_CHR_READ_REQUEST:
          address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
          length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
          read_chr_send(address, length);
          break;

        case COMMAND_CHR_CRC_READ_REQUEST:
          address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
          length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
          read_chr_crc_send(address, length);
          break;

        case COMMAND_CHR_WRITE_REQUEST:
          address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
          length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
          write_chr(address, length, (uint8_t*)&recv_buffer[4]);
          comm_start(COMMAND_CHR_WRITE_DONE, 0);
          break;

        case COMMAND_MIRRORING_REQUEST:
          get_mirroring();
          break;

        /*
        case COMMAND_EPROM_PREPARE:
          write_eprom_prepare();
          break;
        
        case COMMAND_CHR_EPROM_WRITE_REQUEST:
          address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
          length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
          write_eprom(address, length, (uint8_t*)&recv_buffer[4]);
          comm_start(COMMAND_CHR_WRITE_DONE, 0);
          break;
        */
          
        case COMMAND_CHR_FLASH_ERASE_REQUEST:
          if (erase_chr_flash())
            comm_start(COMMAND_CHR_WRITE_DONE, 0);
          break;

        case COMMAND_CHR_FLASH_WRITE_REQUEST:
          address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
          length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
          if (write_chr_flash(address, length, (uint8_t*)&recv_buffer[4]))
            comm_start(COMMAND_CHR_WRITE_DONE, 0);
          break;
          
        case COMMAND_JTAG_SETUP:
          jtag_setup();
          comm_start(COMMAND_JTAG_RESULT, 1);
          comm_send_byte(1);
          break;
          
        case COMMAND_JTAG_SHUTDOWN:
          jtag_shutdown();
          comm_start(COMMAND_JTAG_RESULT, 1);
          comm_send_byte(1);
          break;
        
        case COMMAND_JTAG_EXECUTE:
          address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
          comm_start(COMMAND_JTAG_RESULT, 1);
          LED_RED_ON;
          comm_send_byte(jtag_execute(address, (uint8_t*) &recv_buffer[2]));
          LED_RED_OFF;
          break;
        // Бутлодырь отключен, ибо тут он не нужен
//        case COMMAND_BOOTLOADER:
//          cli();
//          MCUCR = 0; MCUSR = 0;
//          jump_to_bootloader();
      }
    }   
}
