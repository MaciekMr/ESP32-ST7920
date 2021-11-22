/*
Graphic RAM (GDRAM) 
Graphic Display RAM has 64x256 bits bit-mapped memory space. GDRAM address is set by writing 2 consecutive 
bytes of vertical address and horizontal address. Two-byte data (16 bits) configures one GDRAM horizontal address. 
The Address Counter (AC) will be increased by one automatically after receiving the 16-bit data for the next operation. 
After the horizontal address reaching 0FH, the horizontal address will be set to 00H and the vertical address will not 
change. The procedure is summarized below: 
1. Set vertical address (Y) for GDRAM 
2. Set horizontal address (X) for GDRAM 
3. Write D15~D8 to GDRAM (first byte) 
4. Write D7~D0 to GDRAM (second byte) 
Please refer to Table 7 for Graphic Display RAM mapping.
*/

#define clear_display 0x01
#define return_home   0x02
#define entry_mode    0x04 //0x05,0x06,0x07
#define display_contr 0x08 // +4 -> display on  +2 -> cursor on +1 character blink on
#define instruction_set 0x20 //+4 - extended instruction +0 - basic instruction
#define cgram_addr    0x40 //AC5 - AC0 ->address of CGRAM
#define ddram_addr    0x80 // AC6 = 0 AC5 - AC0 -> address od DDRAM address
#define LCD_CLEAR_SCREEN        0x01    // Clear screen
#define LCD_ADDRESS_RESET       0x02    // The address counter is reset
#define LCD_BASIC_FUNCTION      0x30    // Basic instruction set
#define LCD_EXTEND_FUNCTION     0x34    // Extended instruction set
// #define write_ram     -> D7 - D0 - data set RS=1 RW=0

//Extended instructions
