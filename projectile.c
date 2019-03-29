#include <stdlib.h>
volatile int pixel_buffer_start; // global variable
int direction_position [2] = {0,0};
int power_position[2] = {0,0};

void clear_screen();
void draw_line(int x0, int y0, int x1, int y1, short int color);
void plot_pixel(int x, int y, short int line_color);
void wait();
void swap(int * x, int * y);
void delay();

int main(void)
{
    volatile int * pixel_ctrl_ptr = (int *)0xFF203020;
    short int colourlist[] = {0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF};
    int i;

    //direction data
    int direction_line = 50; //length of line
    const int direction_initial [2] = {5 + direction_line, 220}; //the point of rotation of line
    direction_position [0] = 2*direction_line + 5;
    direction_position [1] = direction_initial[1] - direction_line;
    int position_incx =1;

    //power data
    int power_line = 20;
    const int power_initial[2] = {10,150};
    power_position [0] = power_line + power_initial[0];
    power_position [1] = power_initial[1];
    int power_incy = 1;
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
        //clear_screen(); 

        //animation to move power and direction lines
        //direction
        draw_line(direction_initial[0], direction_initial[1], direction_position[0], direction_position[1], colourlist[0]);
        //power
        draw_line(power_initial[0], power_position [1], power_position[0], power_position[1], colourlist[1]);
        //delay
        wait();
        //delay();
        //erase
        draw_line(direction_initial[0], direction_initial[1], direction_position[0], direction_position[1], 0x0000);
        draw_line(power_initial[0], power_position [1], power_position[0], power_position[1], 0x0000);

        if (direction_position[0] == 5 || direction_position[0] == 105) position_incx = -position_incx;
        direction_position[0] += position_incx;
        
        if (power_position[1] == 150 || power_position[1] == 100) power_incy = -power_incy;
        power_position[1] += power_incy;
        
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

void delay() {
    volatile int * timer_ptr = 0xFFFEC600;
    int delay = 100000000;
    int i=0;
    while (i!=delay) i++;
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














