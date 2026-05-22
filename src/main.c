#include <reg51.h>
#include <intrins.h>

/* ---------------- PIN CONFIG ---------------- */

#define MOTOR P2

sbit scl    = P2^4;
sbit sda    = P2^5;
sbit BUZZER = P2^6;

/* ---------------- SERVO PINS (P0) ---------------- */

sbit SERVO1 = P0^0;   /* Base     */
sbit SERVO2 = P0^1;   /* Shoulder */
sbit SERVO3 = P0^2;   /* Elbow    */
sbit SERVO4 = P0^3;   /* Gripper  */

/* ---------------- GLOBAL ---------------- */

unsigned char slave1   = 0x4E;
unsigned char slave_add;

#define IDLE      0
#define COUNTDOWN 1

unsigned char state = IDLE;

/* ---------------- SERVO PULSE WIDTHS ---------------- */
/* Units: timer ticks (1 tick = 100us)
   1000us = 10 ticks  (0 deg)
   1500us = 15 ticks  (90 deg / neutral)
   2000us = 20 ticks  (180 deg)            */

unsigned char servo1_ticks = 15;
unsigned char servo2_ticks = 15;
unsigned char servo3_ticks = 15;
unsigned char servo4_ticks = 15;

unsigned char servo_count  = 0;   /* ISR cycle counter — global, not static */

/* ---------------- TIMER0 ISR — SERVO PWM ---------------- */
/*
   Each tick = 100us
   Cycle = 200 ticks = 20ms = 50Hz

   count=1  : all servos HIGH
   count=N  : servo LOW when count reaches its tick value
   count=200: reset to 0
*/

void timer0_ISR(void) interrupt 1
{
    /* Reload FIRST — before anything else for consistent timing */
    TH0 = 0xFF;
    TL0 = 0x9C;   /* 65536 - 100 = 65436 = 0xFF9C  → exactly 100us @ 12MHz */

    
    servo_count++;

    if(servo_count == 1)
    {
        SERVO1 = 1;
        SERVO2 = 1;
        SERVO3 = 1;
        SERVO4 = 1;
    }

    if(servo_count >= servo1_ticks) SERVO1 = 0;
    if(servo_count >= servo2_ticks) SERVO2 = 0;
    if(servo_count >= servo3_ticks) SERVO3 = 0;
    if(servo_count >= servo4_ticks) SERVO4 = 0;

    if(servo_count >= 200) servo_count = 0;
}

/* ---------------- TIMER0 INIT ---------------- */

void Timer0_Init()
{
    /* Set Timer0 mode 1 (16-bit) WITHOUT touching Timer1 bits */
    TMOD = (TMOD & 0xF0) | 0x01;
    TH0  = 0xFF;
    TL0  = 0x9C;
    ET0  = 1;
    TR0  = 1;
}

/* ---------------- UART INIT ---------------- */

void UART_Init()
{
    /* Set Timer1 mode 2 WITHOUT touching Timer0 bits */
    TMOD = (TMOD & 0x0F) | 0x20;
    TH1  = 0xFD;   /* 9600 baud @ 11.0592MHz */
    SCON = 0x50;
    TR1  = 1;
}

/* ---------------- DELAY ---------------- */

void delay_ms(unsigned int n)
{
    unsigned int m;
    for(; n > 0; n--)
    {
        for(m = 121; m > 0; m--);
        _nop_();
    }
}

/* ---------------- I2C ---------------- */

void i2c_start(void)
{
    sda=1; _nop_();
    scl=1; _nop_();
    sda=0; _nop_();
}

void i2c_stop(void)
{
    scl=0;
    sda=0;
    scl=1;
    sda=1;
}

void i2c_ACK(void)
{
    scl=0;
    sda=1;
    scl=1;
    while(sda);
}

void i2c_write(unsigned char val)
{
    unsigned char i;
    for(i = 0; i < 8; i++)
    {
        scl=0;
        sda=(val & (0x80 >> i)) ? 1 : 0;
        scl=1;
    }
}

/* ---------------- LCD ---------------- */

void lcd_slave(unsigned char sl)
{
    slave_add = sl;
}

void lcd_send_cmd(unsigned char cmd)
{
    unsigned char l, u;
    l = (cmd << 4) & 0xF0;
    u = (cmd & 0xF0);

    i2c_start();
    i2c_write(slave_add); i2c_ACK();

    i2c_write(u|0x0C); i2c_ACK(); delay_ms(1);
    i2c_write(u|0x08); i2c_ACK(); delay_ms(5);
    i2c_write(l|0x0C); i2c_ACK(); delay_ms(1);
    i2c_write(l|0x08); i2c_ACK(); delay_ms(5);

    i2c_stop();
}

void lcd_send_data(unsigned char val)
{
    unsigned char l, u;
    l = (val << 4) & 0xF0;
    u = (val & 0xF0);

    i2c_start();
    i2c_write(slave_add); i2c_ACK();

    i2c_write(u|0x0D); i2c_ACK(); delay_ms(1);
    i2c_write(u|0x09); i2c_ACK(); delay_ms(5);
    i2c_write(l|0x0D); i2c_ACK(); delay_ms(1);
    i2c_write(l|0x09); i2c_ACK(); delay_ms(5);

    i2c_stop();
}

void lcd_send_str(unsigned char *p)
{
    while(*p) lcd_send_data(*p++);
}

void lcd_set_cursor(unsigned char row, unsigned char col)
{
    lcd_send_cmd((row == 0 ? 0x80 : 0xC0) + col);
}

void lcd_init()
{
    delay_ms(50);
    lcd_send_cmd(0x02);
    lcd_send_cmd(0x28);
    lcd_send_cmd(0x0C);
    lcd_send_cmd(0x06);
    lcd_send_cmd(0x01);
    delay_ms(2);
}

/* ---------------- BUZZER (active LOW on P2^6) ---------------- */

#define BUZZ_ON()  { BUZZER = 0; }
#define BUZZ_OFF() { BUZZER = 1; }

void beep_short() { BUZZ_ON(); delay_ms(80);  BUZZ_OFF(); }
void beep_fast()  { BUZZ_ON(); delay_ms(40);  BUZZ_OFF(); }
void beep_long()  { BUZZ_ON(); delay_ms(400); BUZZ_OFF(); }

/* ---------------- DISPLAY ---------------- */

void show(unsigned char *l1, unsigned char *l2)
{
    lcd_send_cmd(0x01);
    delay_ms(2);
    lcd_set_cursor(0,0); lcd_send_str(l1);
    lcd_set_cursor(1,0); lcd_send_str(l2);
}

/* ---------------- BOOT ---------------- */

void boot_screen()
{
    show((unsigned char *)"SENTRY-V v2.1", (unsigned char *)"INITIALIZING...");
    delay_ms(1500);
}

/* ---------------- COUNTDOWN ---------------- */

void countdown()
{
    unsigned char i, t;
    char rx;

    MOTOR = 0x00;
    state = COUNTDOWN;

    show((unsigned char *)"PAYLOAD ARMED", (unsigned char *)"STAND CLEAR");
    delay_ms(1500);

    for(i = 10; i > 0; i--)
    {
        lcd_send_cmd(0x01);
        delay_ms(2);
        lcd_set_cursor(0,0);
        lcd_send_str((unsigned char *)"DEPLOY IN:");
        lcd_set_cursor(1,0);
        lcd_send_data((i / 10) + '0');
        lcd_send_data((i % 10) + '0');
        lcd_send_str((unsigned char *)" SEC  ");

        if(i > 3)
            beep_short();
        else
        {
            beep_fast();
            delay_ms(60);
            beep_fast();
        }

        for(t = 0; t < 10; t++)
        {
            delay_ms(100);

            if(RI)
            {
                rx = SBUF;
                RI = 0;

                if(rx == 'A')
                {
                    show((unsigned char *)"ABORTED", (unsigned char *)"SAFE MODE");
                    beep_long();
                    delay_ms(200);
                    beep_long();
                    state = IDLE;
                    return;
                }
            }
        }
    }

    show((unsigned char *)"DEPLOYED", (unsigned char *)"MISSION DONE");
    beep_long();
    state = IDLE;
}

/* ---------------- MAIN ---------------- */

void main()
{
    char cmd;

    /* Disable ALL interrupts during init to prevent ISR
       running before servo_ticks values are ready           */
    EA = 0;

    MOTOR  = 0x00;
    BUZZ_OFF();

    /* Force servo pins LOW so they don't glitch on boot */
    SERVO1 = 0;
    SERVO2 = 0;
    SERVO3 = 0;
    SERVO4 = 0;

    /* Neutral position — 1500us = 15 ticks */
    servo1_ticks = 15;
    servo2_ticks = 15;
    servo3_ticks = 15;
    servo4_ticks = 15;

    lcd_slave(slave1);
    lcd_init();

    /* Init timers — EA still 0, no ISR fires yet */
    Timer0_Init();
    UART_Init();

    /* NOW enable interrupts — everything is ready */
    EA = 1;

    boot_screen();
    show((unsigned char *)"READY", (unsigned char *)"WAIT CMD");

    while(1)
    {
        if(RI)
        {
            cmd = SBUF;
            RI  = 0;

            /* Ignore CR and LF from serial terminals */
            if(cmd == '\r' || cmd == '\n') continue;

            if(state != COUNTDOWN)
            {
                switch(cmd)
                {
                    /* -------- DRIVE -------- */
                    case 'F':
                        MOTOR = 0x05;
                        show((unsigned char *)"MOVING",   (unsigned char *)"FORWARD");
                        break;

                    case 'B':
                        MOTOR = 0x0A;
                        show((unsigned char *)"MOVING",   (unsigned char *)"BACKWARD");
                        break;

                    case 'L':
                        MOTOR = 0x06;
                        show((unsigned char *)"TURNING",  (unsigned char *)"LEFT");
                        break;

                    case 'R':
                        MOTOR = 0x09;
                        show((unsigned char *)"TURNING",  (unsigned char *)"RIGHT");
                        break;

                    case 'S':
                        MOTOR = 0x00;
                        show((unsigned char *)"STOPPED",  (unsigned char *)"SAFE");
                        break;

                    case 'D':
                        countdown();
                        show((unsigned char *)"READY",    (unsigned char *)"WAIT CMD");
                        break;

                    /* -------- BASE (SERVO1) : 1=left  3=right  2=center -------- */
                    case '1':
                        if(servo1_ticks > 10) servo1_ticks--;
                        show((unsigned char *)"BASE",     (unsigned char *)"<< LEFT");
                        break;

                    case '3':
                        if(servo1_ticks < 20) servo1_ticks++;
                        show((unsigned char *)"BASE",     (unsigned char *)"RIGHT >>");
                        break;

                    case '2':
                        servo1_ticks = 15;
                        show((unsigned char *)"BASE",     (unsigned char *)"CENTER");
                        break;

                    /* -------- SHOULDER (SERVO2) : 4=down  6=up  5=mid -------- */
                    case '4':
                        if(servo2_ticks > 10) servo2_ticks--;
                        show((unsigned char *)"SHOULDER", (unsigned char *)"v DOWN");
                        break;

                    case '6':
                        if(servo2_ticks < 20) servo2_ticks++;
                        show((unsigned char *)"SHOULDER", (unsigned char *)"^ UP");
                        break;

                    case '5':
                        servo2_ticks = 15;
                        show((unsigned char *)"SHOULDER", (unsigned char *)"MID");
                        break;

                    /* -------- ELBOW (SERVO3) : 7=fold  9=extend  8=mid -------- */
                    case '7':
                        if(servo3_ticks > 10) servo3_ticks--;
                        show((unsigned char *)"ELBOW",    (unsigned char *)"<< FOLD");
                        break;

                    case '9':
                        if(servo3_ticks < 20) servo3_ticks++;
                        show((unsigned char *)"ELBOW",    (unsigned char *)"EXTEND >>");
                        break;

                    case '8':
                        servo3_ticks = 15;
                        show((unsigned char *)"ELBOW",    (unsigned char *)"MID");
                        break;

                    /* -------- GRIPPER (SERVO4) : G=open  H=close -------- */
                    case 'G':
                        if(servo4_ticks > 10) servo4_ticks--;
                        show((unsigned char *)"GRIPPER",  (unsigned char *)"OPENING...");
                        break;

                    case 'H':
                        if(servo4_ticks < 20) servo4_ticks++;
                        show((unsigned char *)"GRIPPER",  (unsigned char *)"CLOSING...");
                        break;

                    /* -------- ARM FULL CENTER -------- */
                    case '0':
                        servo1_ticks = 15;
                        servo2_ticks = 15;
                        servo3_ticks = 15;
                        servo4_ticks = 15;
                        show((unsigned char *)"ARM",      (unsigned char *)"ALL CENTER");
                        beep_short();
                        break;
                }
            }
        }
    }
