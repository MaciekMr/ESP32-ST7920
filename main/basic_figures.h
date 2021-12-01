

struct point{
    uint8_t x;
    uint8_t y;
};

class basic_shapes
{
private:

public:
    void draw_box(point a, point b, bool mode, uint8_t width = 1);
    void draw_circle(point a, uint8_t radial, bool mode, uint8_t width = 1);
    void draw_triangle(point a, point b, point c, bool mode, uint8_t width = 1);
};