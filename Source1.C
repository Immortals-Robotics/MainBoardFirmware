#include "serial.h"

char robotNum=0;

unsigned char des[4];
unsigned char dir_des[4];

unsigned char push[4];
unsigned char dip[8];

unsigned char BatteryLevel;

unsigned char Direct_Power;
unsigned char Chip_Power;

unsigned char Servo_State;
unsigned char Force;
unsigned char Ready;
unsigned char Dribbler_Power ;
unsigned char SOS;
unsigned char Debug_Mode;
unsigned char beep_Command ;


#define Motor1    1
#define Motor2    2
#define Motor3    3
#define Motor4    4
#define Dribbler  5
#define Servo     6
#define DKick     7
#define CKick     8

#define Led1    P1_0
#define Led2    P1_1
#define Led3    P1_2
#define Buzzer  P1_3

#define MUX1  P1_4
#define MUX2  P1_5
#define WE0   P1_6
#define WE1   P1_7

#define Input1    P2_0
#define Input2    P2_1
#define Input3    P2_2
#define Input4    P2_3

#define Push1    P2_4
#define Push2    P2_5
#define Push3    P2_6
#define Push4    P2_7

#define InputSel0    P1_4
#define InputSel1    P1_5

#define Force_Kick  P3_0
#define Kick_Ready  P3_1
#define ChargePulse P3_2

#define dir1 P3_4
#define dir2 P3_5
#define dir3 P3_6
#define dir4 P3_7



void delay(unsigned int ms)
{
    int t = ms /100;
    t*=100;
    ms=ms%100;
    for (int i = 0; i < t ; i++ )
    {
        for (int i = 0; i < 1135 ; i++ )  //1135
        {
            __asm( "nop" );
        }
    }
    ms*=1135;  // recalculate    1135
    for (int i = 0; i < ms ; i++ )
    {
        __asm( "nop" );
    }
}

void beep(int count,int d)
{
   for(int i=0;i<count;i++)
   {
      Buzzer=1;
      delay(d);
      Buzzer=0;
      delay(d);
   }
}

void getDipState()
{
      dip[0]=Input1;
      dip[1]=Input2;
      dip[2]=Input3;
      dip[3]=Input4;
}

void SetData ( unsigned char add , char data )
{
    switch ( add )
    {
        case 1:
             MUX1 = 0;
             MUX2 = 0;
             P0 = data;
             WE1 = 1;
             __asm( "nop" );
             __asm( "nop" );
             WE1 = 0;
             break;
        case 2:
             MUX1 = 1;
             MUX2 = 0;
             P0 = data;
             WE1 = 1;
             __asm( "nop" );
             __asm( "nop" );
             WE1 = 0;
             break;
        case 3:
             MUX1 = 0;
             MUX2 = 1;
             P0 = data;
             WE1 = 1;
             __asm( "nop" );
             __asm( "nop" );
             WE1 = 0;
             break;
        case 4:
             MUX1 = 1;
             MUX2 = 1;
             P0 = data;
             WE1 = 1;
             __asm( "nop" );
             __asm( "nop" );
             WE1 = 0;
             break;

        case 5:
             MUX1 = 0;
             MUX2 = 0;
             P0 = data;
             WE0 = 1;
             __asm( "nop" );
             __asm( "nop" );
             WE0 = 0;
             break;

        case 6:
             MUX1 = 1;
             MUX2 = 0;
             P0 = data;
             WE0 = 1;
             __asm( "nop" );
             __asm( "nop" );
             WE0 = 0;
             break;

        case 7:
             MUX1 = 0;
             MUX2 = 1;
             P0 = data;
             WE0 = 1;
             __asm( "nop" );
             __asm( "nop" );
             WE0 = 0;
             break;

        case 8:
             MUX1 = 1;
             MUX2 = 1;
             P0 = data;
             WE0 = 1;
             __asm( "nop" );
             __asm( "nop" );
             WE0 = 0;
             break;
    }
}

void setServo(unsigned char teta)
{
     // conver the teta to data
     SetData(Servo,teta);
}

void init()
{
   Kick_Ready =0;
   Force_Kick =0;
   Buzzer=0;
   Led1=0;
   Led2=0;
   Led3=0;
   ChargePulse = 0;
   serial_init(5);  // Baud Rate : 38400 ;
   getDipState();
   robotNum= dip[3] + dip[2]*2 + dip[1]*4 + dip[0]*8;
   SetData ( 1 , 0 );
   SetData ( 2 , 0 );
   SetData ( 3 , 0 );
   SetData ( 4 , 0 );
   SetData ( 5 , 0 );
   Direct_Power = 0;
   Chip_Power = 0;
   Force=0;
   Ready =0;
   SetData(DKick,Direct_Power);
   SetData(CKick,Chip_Power);
   Servo_State = 1;
   //setServo(Servo_State);
   beep_Command =0;
   //beep(3,50);
}

void recivePacket()
{
     int counter=0;
     unsigned char c;
     while(counter<8)
     {
          c=serial_getch();
          if(c==0)
          {
             counter=1;
          }
          else if(counter==1)
          {
            if((c%16)==robotNum)
            {
             Led1 = 1;
             c=c>>4;
             Direct_Power = c%16;
             counter++;
            }
            else
            {
                counter = 0;
            }
          }
          else if(counter==2)
          {
            if(c>127)
            {
                c=c-127;
                c=c*2;
                des[0] = c;
                dir_des[0]=0;
            }else
            {
                c=127-c;
                c=c*2;
                des[0] = c;
                dir_des[0]=1;

            }
             counter++;
          }
          else if(counter==3)
          {
             if(c>127)
            {
                c=c-127;
                c=c*2;
                des[1] = c;
                dir_des[1]=0;
            }else
            {
                c=127-c;
                c=c*2;
                des[1] = c;
                dir_des[1]=1;

            }
             counter++;
          }
          else if(counter==4)
          {
             if(c>127)
            {
                c=c-127;
                c=c*2;
                des[2] = c;
                dir_des[2]=0;
            }else
            {
                c=127-c;
                c=c*2;
                des[2] = c;
                dir_des[2]=1;

            }
             counter++;
          }
          else if(counter==5)
          {
             if(c>127)
            {
                c=c-127;
                c=c*2;
                des[3] = c;
                dir_des[3]=0;
            }else
            {
                c=127-c;
                c=c*2;
                des[3] = c;
                dir_des[3]=1;

            }
             counter++;
          }
          else if(counter==6)
          {
            Servo_State = c%4;
            c=c>>2;
            Ready = c%2;
            c=c>>1;
            Force = c%2;
            c=c>>1;
            Chip_Power = c%16;
            counter++;
          }

          else if(counter==7)
          {
            beep_Command = c%2;
            c=c>>2;
            Debug_Mode = c%2;
            c=c>>1;
            SOS = c%2;
            c=c>>1;
            Dribbler_Power = c%16;
            counter++;
          }

     }

     /*if(SOS==1)
     {

          SetData( Motor1,0);
          SetData( Motor2,0);
          SetData( Motor3,0);
          SetData( Motor4,0);
          SetData(Dribbler,0);
          setServo(0);
          Force = 0;
          Ready = 0;
          ChargePulse = 0;
          Buzzer =0;
          beep(20,50);

     }*/
     //else
     {

          SetData( Motor1,des[0]);
          dir1 = dir_des[0];

          SetData( Motor2,des[1]);
          dir2 = dir_des[1];

          SetData( Motor3,des[2]);
          dir3 = dir_des[2];

          SetData( Motor4,des[3]);
          dir4 = dir_des[3];

          Dribbler_Power = Dribbler_Power << 4 ;

          SetData(Dribbler,Dribbler_Power);

          setServo(Servo_State);

          SetData(DKick,Direct_Power);
          SetData(CKick,Chip_Power);

          Force_Kick = 0;
          Kick_Ready = Ready;
          Buzzer = beep_Command % 2;

     }

     Led1 = 0;
}

void putch(char t)
{
       SBUF = t;
       while (TI == 0)    // wait for TI flag to come true
       {

       }
       TI = 0;
}

void sendData(unsigned char d1, unsigned char d2, unsigned char d3 , unsigned char d4)
{
    serial_putch(0);
    serial_putch(robotNum + (Direct_Power%16)*16);
    serial_putch(d1);
    serial_putch(d2);
    serial_putch(d3);
    serial_putch(d4);
}

#define max(a,b) (a>b?a:b)
#define min(a,b) (a<b?a:b)
#define sss 30

void main( void )
{
    init();
    while (1)
    {
        recivePacket();
    }
}
