
#include <stdlib.h>

//global variables
volatile int pixel_buffer_start; 
int direction_position [2] = {0,0};
int power_position[2] = {0,0};
int dart_position[2] = {0,0};
//step tells you what part in the game are you (finding power, direction or shooting)
int step = -1;
/*
-1 = start of game, show the moving power and direction lines + the dart
0 = key 2 or 3 has been pressed and user can start
1 = direction has been chosen
2 = power has been chosen and projecticle starts
3 = dart has reached the dart board aka end of game
*/

//function prototypes to draw on VGA
void clear_screen();
void draw_line(int x0, int y0, int x1, int y1, short int color);
void plot_pixel(int x, int y, short int line_color);
void plot_arc(double velocity);
void wait();
void swap(int * x, int * y);
void delay();
void reset_vector();
void write_char(int x, int y, char c);


//function prototypes to set up button interrupts
void set_A9_IRQ_stack(void);
void config_GIC(void);
void config_KEYs(void);
void enable_A9_interrupts(void);
void pushbutton_ISR(void);
void config_interrupt(int, int);

int main(void)
{
    disable_A9_interrupts(); // disable interrupts in the A9 processor
    set_A9_IRQ_stack(); // initialize the stack pointer for IRQ mode
    config_GIC(); // configure the general interrupt controller
    config_KEYs(); // configure pushbutton KEYs to generate interrupts
    enable_A9_interrupts(); // enable interrupts in the A9 processor
    volatile int * pixel_ctrl_ptr = (int *)0xFF203020;
    short int colourlist[] = {0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF};

    //direction data
    const int direction_initial [2] = {210, 220}; //the point of rotation of line
    int position_incx =1;

    //power data
    const int power_initial[2] = {30,150};
    int power_incy = 1;

    //dart data
    int dart_incx = 1;
    int dart_incy = -1;

    reset_vector();

    *(pixel_ctrl_ptr + 1) = 0xC8000000; 
    wait();
    pixel_buffer_start = *pixel_ctrl_ptr;
    clear_screen(); 
    *(pixel_ctrl_ptr + 1) = 0xC0000000;
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); 


    while (1)
    {
        /* Erase any boxes and lines that were drawn in the last iteration */
        clear_screen();
        plot_arc(1.5);
        if (step<=1) {
            draw_line(direction_initial[0], direction_initial[1], direction_position[0], direction_position[1], colourlist[0]);
            draw_line(power_initial[0], power_position [1], power_position[0], power_position[1], colourlist[1]);
        }
        if (step>1) draw_line(dart_position[0], dart_position[1], dart_position[0] + 10, dart_position[1], colourlist[2]);
        //delay
        wait();
        //delay();

        //erase
        //draw_line(direction_initial[0], direction_initial[1], direction_position[0], direction_position[1], 0x0000);
        //draw_line(power_initial[0], power_position [1], power_position[0], power_position[1], 0x0000);

        //check if any should be updated
        if (direction_position[0] == 5 || direction_position[0] == 105) position_incx = -position_incx;
        if (power_position[1] == 150 || power_position[1] == 100) power_incy = -power_incy;
        if (dart_position[1] == 0 || dart_position[1] == 100) dart_incy = -dart_incy;
        if (dart_position[0] == 20 || dart_position[0] == 309) dart_incx = -dart_incx;
        
        if (step == 0) direction_position[0] += position_incx; //changing direction
        else if (step == 1) power_position[1] += power_incy; //changing power
        else if (step == 2) { //sending projectile
            dart_position[0] += dart_incx;
            dart_position[1] +=dart_incy;
        }
        
        pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer
        
    }
}


void swap(int * x, int * y){
    int temp = *x;
    *x = *y;
    *y = temp;  
}

void delay() {
    int delay = 50000;
    int i=0;
    while (i!=delay) i++;
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

void reset_vector(){
    //60 is length of direction line
    direction_position [0] = 270; //2*direction line + direction initial
    direction_position [1] = 160; //direction initial - direction line

    //30 is length of power line
    power_position [0] = 60; //power line + power initial
    power_position [1] = 150; //power initial

    dart_position [0] = 40;
    dart_position [1] = 150;

} 

void plot_pixel(int x, int y, short int line_color){
    *(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = line_color;
}

void write_char(int x, int y, char c) {
  // VGA character buffer
  volatile char * character_buffer = (char *) (0x09000000 + (y<<7) + x);
  *character_buffer = c;
}

void draw_line(int x0, int y0, int x1, int y1, short int color) {
    int is_steep = 0; //initialize to false
    int abs_y = y1 - y0;
    int abs_x = x1 - x0;
    
    if (abs_y < 0 ) abs_y =-abs_y; //change sign if negative
    if (abs_x < 0) abs_x = -abs_x;
    
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
    int deltay = y1-y0;
    if (deltay <0) deltay = -deltay;
    int error = -(deltax / 2);
    int y = y0;
    int y_step;
        
    if (y0 < y1) y_step =1;
    else y_step = -1;
    
    int x;
    for(x=x0; x<=x1; x++) {
        if (is_steep) plot_pixel(y,x, color);
        else plot_pixel(x,y, color);
        
        error += deltay;
        
        if (error>=0) {
            y +=y_step;
            error -= deltax;
        }
    }
}

/* setup the KEY interrupts in the FPGA */
void config_KEYs() {
    volatile int * KEY_ptr = (int *) 0xFF200050; // pushbutton KEY base address
    *(KEY_ptr + 2) = 0xF; // enable interrupts for the two KEYs
}

// Define the IRQ exception handler
void __attribute__((interrupt)) __cs3_isr_irq(void) {
    // Read the ICCIAR from the CPU Interface in the GIC
    int interrupt_ID = *((int *)0xFFFEC10C);
    if (interrupt_ID == 73) // check if interrupt is from the KEYs
    pushbutton_ISR();
    else
    while (1); // if unexpected, then stay here
    // Write to the End of Interrupt Register (ICCEOIR)
    *((int *)0xFFFEC110) = interrupt_ID;
}
// Define the remaining exception handlers
void __attribute__((interrupt)) __cs3_reset(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_undef(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_swi(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_pabort(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_dabort(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_fiq(void) {
    while (1);
}
//Turn off interrupts in the ARM processor
void disable_A9_interrupts(void) {
    int status = 0b11010011;
    asm("msr cpsr, %[ps]" : : [ps] "r"(status));
}
//Initialize the banked stack pointer register for IRQ mode
void set_A9_IRQ_stack(void) {
    int stack, mode;
    stack = 0xFFFFFFFF - 7; // top of A9 onchip memory, aligned to 8 bytes
    /* change processor to IRQ mode with interrupts disabled */
    mode = 0b11010010;
    asm("msr cpsr, %[ps]" : : [ps] "r"(mode));
    /* set banked stack pointer */
    asm("mov sp, %[ps]" : : [ps] "r"(stack));
    /* go back to SVC mode before executing subroutine return! */
    mode = 0b11010011;
    asm("msr cpsr, %[ps]" : : [ps] "r"(mode));
}
//Turn on interrupts in the ARM processor
void enable_A9_interrupts(void) {
    int status = 0b01010011;
    asm("msr cpsr, %[ps]" : : [ps] "r"(status));
}
/*
* Configure the Generic Interrupt Controller (GIC)
*/
void config_GIC(void) {
    config_interrupt (73, 1); // configure the FPGA KEYs interrupt (73)
    // Set Interrupt Priority Mask Register (ICCPMR). Enable interrupts of all
    // priorities
    *((int *) 0xFFFEC104) = 0xFFFF;
    // Set CPU Interface Control Register (ICCICR). Enable signaling of
    // interrupts
    *((int *) 0xFFFEC100) = 1;
    // Configure the Distributor Control Register (ICDDCR) to send pending
    // interrupts to CPUs
    *((int *) 0xFFFED000) = 1;
}
/*
* Configure Set Enable Registers (ICDISERn) and Interrupt Processor Target
* Registers (ICDIPTRn). The default (reset) values are used for other registers
* in the GIC.
*/
void config_interrupt(int N, int CPU_target) {
    int reg_offset, index, value, address;
    /* Configure the Interrupt Set-Enable Registers (ICDISERn).
    * reg_offset = (integer_div(N / 32) * 4
    * value = 1 << (N mod 32) */
    reg_offset = (N >> 3) & 0xFFFFFFFC;
    index = N & 0x1F;
    value = 0x1 << index;
    address = 0xFFFED100 + reg_offset;
    /* Now that we know the register address and value, set the appropriate bit */
    *(int *)address |= value;
    /* Configure the Interrupt Processor Targets Register (ICDIPTRn)
    * reg_offset = integer_div(N / 4) * 4
    * index = N mod 4 */
    reg_offset = (N & 0xFFFFFFFC);
    index = N & 0x3;
    address = 0xFFFED800 + reg_offset + index;
    /* Now that we know the register address and value, write to (only) the
    * appropriate byte */
    *(char *)address = (char)CPU_target;
}
void pushbutton_ISR(void) {
    /* KEY base address */
    volatile int * KEY_ptr = (int *) 0xFF200050;
    /* HEX display base address */
    volatile int * HEX3_HEX0_ptr = (int *) 0xFF200020;
    int press, HEX_bits;
    press = *(KEY_ptr + 3); // read the pushbutton interrupt register
    *(KEY_ptr + 3) = press; // Clear the interrupt
    if (press & 0x1) { //key0
        HEX_bits = 0b00111111;
        step = 1; 
    }
    else if (press & 0x2) { //key1
        HEX_bits = 0b00000110;
        step = 2 ;
    }
    else { //key 2 or 3
        HEX_bits = 0b1111111;
        if (step == -1) {
            step = 0; //to begin game
            reset_vector();
        }
        else step = -1; //to reset
    }

    *HEX3_HEX0_ptr = HEX_bits;
    return;
}

void plot_arc(double velocity){
    int x, y;
    double calcY, calcX;
    double transNum = 0.03483;
    
    for (x = 0; x < 160; x++){
        calcX = (2.44/160)*x;
        calcY = calcX - (((calcX)*(calcX))/(velocity * velocity));
        y = calcY / transNum;
        plot_pixel (x + 80, 120 - y, 0xFFFF);
    }

}











	