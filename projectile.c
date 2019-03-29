#include <stdlib.h>
volatile int pixel_buffer_start; // global variable

void clear_screen();
void draw_line(int x0, int y0, int x1, int y1, short int color);
void plot_pixel(int x, int y, short int line_color);
void wait();
void swap(int * x, int * y);

int main(void)
{
    volatile int * pixel_ctrl_ptr = (int *)0xFF203020;
    short int colourlist[] = {0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF};
    int direction_line = 50;
    int power_line = 10;
    int projectile_line = 20;


    /* set front pixel buffer to start of FPGA On-chip memory */
    *(pixel_ctrl_ptr + 1) = 0xC8000000; // first store the address in the back buffer
    /* now, swap the front/back buffers, to set the front buffer location */
    wait();
    /* initialize a pointer to the pixel buffer, used by drawing functions */
    pixel_buffer_start = *pixel_ctrl_ptr;
    clear_screen(); // pixel_buffer_start points to the pixel buffer

    /* set back pixel buffer to start of SDRAM memory */
    *(pixel_ctrl_ptr + 1) = 0xC0000000;
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); // we draw on the back buffer

    while (1)
    {
        /* Erase any boxes and lines that were drawn in the last iteration */
        clear_screen();

        wait(); // swap front and back buffers on VGA vertical sync
        pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer
    }
}

void swap(int * x, int * y){
    int temp = *x;
    *x = *y;
    *y = temp;   
}

void wait(){
    volatile int * pixel_ctrl_ptr = (int *)0xFF203020;
    volatile int * status =(int *)0xFF20302C;

    *pixel_ctrl_ptr = 1;

    while((*status & 0x01) != 0) status = status; //keep reading status
    
    //exit when S is 1
    return;
}  

void clear_screen() {
    int x;
    int y;
    for (x = 0; x < 320; x++)
        for (y = 0; y < 240; y++)
            plot_pixel(x, y, 0x0000);   
}

void plot_pixel(int x, int y, short int line_color){
    *(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = line_color;
}

void draw_line(int x0, int y0, int x1, int y1, short int color) {
    int is_steep = 0; //initialize to false
    int abs_y = abs(y1-y0);
    int abs_x = abs(x1-x0);
    
    if (abs_y > abs_x) is_steep=1; //TRUE
    
    if (is_steep) {
        swap(&x0, &y0);
        swap(&x1, &y1);
    }
   
    if (x0>x1) {
        swap(&x0, &x1);
        swap(&y0, &y1);
    }
    
    int deltax = x1 - x0;
    int deltay = abs(y1-y0);
    int error = -(deltax / 2);
    int y = y0;
    int y_step;
        
    if (y0 < y1) y_step =1;
    else y_step = -1;
    
    for(int x=x0; x<=x1; x++) {
        if (is_steep) plot_pixel(y,x, color);
        else plot_pixel(x,y, color);
        
        error += deltay;
        
        if (error>=0) {
            y +=y_step;
            error -= deltax;
        }
    } 
}






