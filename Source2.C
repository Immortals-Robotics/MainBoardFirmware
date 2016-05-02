#define ADD1 P3_0
#define ADD2 P3_1
#define ADD3 P3_2
#define WR P3_2

#define DirM1 P3_4
#define DirM2 P3_5
#define DirM3 P3_6
#define DirM4 P3_7

#define max(a,b) (a>b?a:b)
#define min(a,b) (a<b?a:b)

unsigned char des[4];
unsigned char current[4][2];
int curr[4];

void setMotor(char id,char dir,unsigned char speed);
int getMotorSpeed(char id);
void getData (void);

inline void p ( char id );


void main ( void )
{
    current[0][0] = current[0][1] = current[1][0] = current[1][1] = current[2][0] = current[2][1] = current[3][0] = current[3][1] = 0;
    curr[0] = curr[1] = curr[2] = curr[3] = 0;
    WR = 0;
    setMotor( 3 , 0 , 200 );
        setMotor( 4 , 1 , 30 );
    for ( ;; )
    {
        //P2 = P1;
        //getData();
        des[0] = 50;
        des[1] = 50;
        des[2] = 50;
        des[3] = 50;
        //p(1);
        //p(2);
        //p(3);
        //getMotorSpeed( 3 );

        //p(4);
        /*getMotorSpeed( 4 );
        getMotorSpeed( 4 );
        getMotorSpeed( 3 );
        getMotorSpeed( 2 );
        getMotorSpeed( 1 );*/
        P2 = curr[3];

    }
}

void setMotor(char id,char dir,unsigned char speed)
{
    switch(id)
    {
        case 1 :
             P0=speed;
             ADD1 = 0;
             ADD2 = 0;
             __asm( "nop" );
             __asm( "nop" );
             WR = 1;
             DirM1=dir;
             __asm( "nop" );
             __asm( "nop" );
             WR = 0;
             break;

        case 2 :
             P0=speed;
             ADD1 = 1;
             ADD2 = 0;
             __asm( "nop" );
             __asm( "nop" );
             WR = 1;
             DirM2=dir;
             __asm( "nop" );
             __asm( "nop" );
             WR = 0;
             break;


        case 3 :
             P0=speed;
             ADD1 = 0;
             ADD2 = 1;
             __asm( "nop" );
             __asm( "nop" );
             WR = 1;
             DirM3=dir;
             __asm( "nop" );
             __asm( "nop" );
             WR = 0;
             break;


        case 4 :
             P0=speed;
             ADD1 = 1;
             ADD2 = 1;
             __asm( "nop" );
             __asm( "nop" );
             WR = 1;
             DirM4=dir;
             __asm( "nop" );
             __asm( "nop" );
             WR = 0;
             break;

    }

}

int getMotorSpeed(char id)
{
    char changed = 0;
    int ans = 0;
    switch(id)
    {
        case 1:
             ADD1 = 0;
             ADD2 = 0;
             ADD3 = 0;
             ans += P1;
             if ( P1 != current[0][0] )
             {
                current[0][0] = P1;
                changed = 1;

             }
             ADD1 = 1;
             ADD2 = 0;
             ADD3 = 0;
             if ( ( P1 != current[0][1] ) || ( changed ) )
             {
                ans += P1*256;
                current[0][1] = P1;
                changed = 1;

             }

             break;
        case 2:
             ADD1 = 0;
             ADD2 = 1;
             ADD3 = 0;
             ans += P1;
             if ( P1 != current[1][0] )
             {
                current[1][0] = P1;
                changed = 1;

             }
             ADD1 = 1;
             ADD2 = 1;
             ADD3 = 0;
             if ( ( P1 != current[1][1] ) || ( changed ) )
             {
                ans += P1*256;
                current[1][1] = P1;
                changed = 1;
             }

             break;
        case 3:
             ADD1 = 0;
             ADD2 = 0;
             ADD3 = 1;
             ans += P1;
             if ( P1 != current[2][0] )
             {
                current[2][0] = P1;
                changed = 1;

             }
             ADD1 = 1;
             ADD2 = 0;
             ADD3 = 1;
             if ( ( P1 != current[2][1] ) || ( changed ) )
             {
                ans += P1*256;
                current[2][1] = P1;
                changed = 1;
             }

             break;
        case 4:
             ADD1 = 0;
             ADD2 = 1;
             ADD3 = 1;
             ans += P1;
             if ( P1 != current[3][0] )
             {
                current[3][0] = P1;
                changed = 1;

             }
             ADD1 = 1;
             ADD2 = 1;
             ADD3 = 1;
             if ( ( P1 != current[3][1] ) || ( changed ) )
             {
                ans += P1*256;
                current[3][1] = P1;
                changed = 1;
             }

             break;
        default:
                ans = 1;
    }
    if ( changed == 1 )
    {
    ans = 12754 / ans;
    if ( ans < 0 )
       ans = 0;
    if ( ans > 255 )
       ans = 255;
       curr[id-1] = ans;
    }
    else
    {
        ans = curr[id-1];
    }
    return ans;
}

void getData (void)
{
    ADD1 = 0;
    ADD2 = 0;
    ADD3 = 0;
    des[0] = P2 - 127 ;
    ADD1 = 1;
    ADD2 = 0;
    ADD3 = 0;
    des[1] = P2 - 127;
    ADD1 = 0;
    ADD2 = 1;
    ADD3 = 0;
    des[2] = P2 - 127;
    ADD1 = 1;
    ADD2 = 1;
    ADD3 = 0;
    des[3] = P2 - 127;
}
 #define hehe 127
inline void p ( char id )
{
    int tmp = curr[id-1];
    if ( tmp == getMotorSpeed( id ) )
       return;
    int khor = 20*(des[id-1] - curr[id-1]+des[id-1]);

    khor = max ( -hehe , min ( hehe , khor ) );
    khor += des[id-1];

    if ( khor >= 0 )
    {
        setMotor( id , 1 , khor );
        //P2=100;
    }
    else
    {
        setMotor( id , 0 , -khor );
        //P2=0;
    }
}
