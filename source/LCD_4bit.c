#include "LCD_4bit.h"
#include "delay.h"
#include <stdint.h>

uint8_t lcd_read_status(void)
{
  uint8_t status;

  SET_LCD_DATA_DIR_IN
  SET_LCD_RS(0)
  SET_LCD_RW(1) // read
  Delay(1);
  SET_LCD_E(1)
  Delay(1);
  status  = GET_LCD_DATA_IN << 4; // bits 0->3 move to bits 4-7 (reads bits 4-7)
  SET_LCD_E(0) // prepare for new edge (expecting data)
  Delay(1);
  SET_LCD_E(1) // latch data (detects rising edge)
  Delay(1);
  status |= GET_LCD_DATA_IN; // reads bits 0-3
  SET_LCD_E(0)
  SET_LCD_DATA_DIR_OUT
  return(status); // full upper + lower bits
}

// wait until LCD is ready for next command / data
void wait_while_busy(void)
{
	for( ; lcd_read_status() & LCD_BUSY_FLAG_MASK; ) // bit 7 (1 = busy / 0 = ready)
		;
}

void lcd_write_4bit(uint8_t c)
{
  SET_LCD_RW(0) // set RW pin 8
  SET_LCD_E(1) // waiting for data (writes 1 to pin 7 of PDOR = ON)
  SET_LCD_DATA_OUT(c&0x0F) // c -> 8 bit character & 0x0F = LOWER 4 BITS OF 8 BIT CHARACTER
  Delay(1);
  SET_LCD_E(0) // drives pin 7 of PDOR low = off (LCD latches data)
  Delay(1);
}

void lcd_write_cmd(uint8_t c)
{
  wait_while_busy(); // POLLS BIT 7 OF LCD

  SET_LCD_RS(0) // SETS BIT 9 OF PSOR (SETS BIT 9 IN PDOR = ON)
  lcd_write_4bit(c>>4); //
  lcd_write_4bit(c);
}

static void lcd_write_data(uint8_t c)
{
  wait_while_busy();

  SET_LCD_RS(1) // RS = 1 -> send data command
  lcd_write_4bit(c>>4); // moves (4) upper bits of c to lower 4 bits
  lcd_write_4bit(c); // sends lower (4) bits
}

void lcd_putchar(char c)
{ 
  lcd_write_data(c); // sends 8 bit character into 4 lower & upper
}

void lcd_init_port(void) {
	/* Enable clocks for peripherals        */
  ENABLE_LCD_PORT_CLOCKS                          

	/* Set Pin MUX to GPIO */
  	// WANT THEM AS GPIO BECAUSE WE MANUALLY CONTROL THEM (DRIVE THEM HIGH & LOW)
  	// PIN_DATA_SHIFT = 3
  	// PTC3-6 -> LCD DATA LINES
	PIN_DATA_PORT->PCR[PIN_DATA_SHIFT] = PORT_PCR_MUX(1);   // 3
	PIN_DATA_PORT->PCR[PIN_DATA_SHIFT+1] = PORT_PCR_MUX(1); // 4
	PIN_DATA_PORT->PCR[PIN_DATA_SHIFT+2] = PORT_PCR_MUX(1); // 5
	PIN_DATA_PORT->PCR[PIN_DATA_SHIFT+3] = PORT_PCR_MUX(1); // 6
	PIN_E_PORT->PCR[PIN_E_SHIFT] = PORT_PCR_MUX(1);
	PIN_RW_PORT->PCR[PIN_RW_SHIFT] = PORT_PCR_MUX(1);
	PIN_RS_PORT->PCR[PIN_RS_SHIFT] = PORT_PCR_MUX(1);
}

void Init_LCD(void)
{ 
	/* initialize port(s) for LCD */
	lcd_init_port();
	
  /* Set all pins for LCD as outputs */
  SET_LCD_ALL_DIR_OUT
  Delay(100);
  SET_LCD_RS(0) // RS = 0 = SEND COMMAND
  lcd_write_4bit(0x3); // SENDS 0011 -> CONFIGURES 4 BIT MODE
  Delay(100);
  lcd_write_4bit(0x3);
  Delay(10);
  lcd_write_4bit(0x3);
  lcd_write_4bit(0x2); // SENDS 0010 -> LCD 4 BIT MODE
  lcd_write_cmd(0x28); // 0010 1000
                       // UPPER = 0010 = D5 =
                       // LOWER = 1000 = D3 = 2 LINE DISPLAY MODE SET
  lcd_write_cmd(0x0C); // SENDS 0000 1100
                       // LOWER = 1100 = D2 & D3
  lcd_write_cmd(0x06); // SENDS 0000 0110 = D1 & D2 = ENTRY MODE COMMAND
  lcd_write_cmd(0x80); // SENDS 1000 0000 = D7 = SET DDRAM ADDRESS (move cursor to beginning of first line
}

void Set_Cursor(uint8_t column, uint8_t row)
{
  uint8_t address;

  address =(row * 0x40) + column; // DEPENDING ON ARG (SENDS IT TO EITHER FIRST OR SECOND ROW + COLUMN
	address |= 0x80; // SET DDRAM ADDRESS COMMAND (adds command bit to address (bits 6-0 are the address)
	                 //
  lcd_write_cmd(address);               
}

void Clear_LCD(void)
{
  lcd_write_cmd(0x01); // CLEAR COMMAND FROM INSTRUCTION TABLE
  Set_Cursor(0, 0); // SENDS BACK TO 0x00
}

void Print_LCD(char *string)
{
  while(*string)  {
    lcd_putchar(*string++); // WRITES ASCII TO DDRAM
  }
}
