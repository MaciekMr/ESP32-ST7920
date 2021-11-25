
#define LCD_X_SIZE      128
#define LCD_Y_SIZE      64
#define LCD_SEGMENTS    2
#define LCD_DATA_WIDTH       16

//ST7920 has 2 vertical segments 
//each 8 horizontal segments with 16 bits in each line
//and 32 lines 

enum transmission_mode
{
    par_8bit = 0,
    par_4bit = 1,
    serial = 2
};

enum lcd_mode
{
    text = 0,
    grap = 1
};

struct point{
    uint8_t x;
    uint8_t y;
};

class LCD_ST7920
{
private:
    uint8_t             mode;
    //spi_device_handle_t spi;
    spi_handler*        spi;
    uint16_t*           framebuffer; //For ST7920 - we need to create buffer 128bits x 32 rows x 2 segments-> 1kB and using dma copy to spi, with refresh rate 30Hz (interrupt)

public:
    LCD_ST7920(transmission_mode, spi_handler*);
    ~LCD_ST7920();
    //0 - text mode 1 - graphics mode
    void set_lcd_mode(uint8_t mode); 
    //lcd init text or graphic mode 0 or 1
    void lcd_init(lcd_mode);
    //write text in text mode
    void put_text(const char *text, uint8_t length);
    //Clean the display
    void clear(lcd_mode);
    /*
    in text mode
    first line starts from 0x80
    second line from 0x90
    third line from 0x88
    fourth line from 0x98
    */
    void set_start_address(uint8_t);

    void highlight_menu_item(uint8_t idx, bool fill = true);

    void set_point(point a, point b, uint8_t valuea, uint16_t valueb);

    void draw_line(point a, point b);

    void fill_GDRAM(); //unsigned char *bitmap);

    void set_point_f(point, uint8_t);

    void copy_ram_gram(); //copy framebuffer to display memory - executed when interrrupt running - add semaphore?
};