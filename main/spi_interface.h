

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
public:
    LCD_ST7920(transmission_mode, spi_handler*);
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

    void set_point(point a);

    void draw_line(point a, point b);
};