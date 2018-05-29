
/*
 * this is the program for ekf and the implement of the sensor AHB100
 * @author:NWPU_FC
 * DATE:2017/6/8
*/

#include <iostream>
#include"matrix_operation.h"
#include<vector>
#include<math.h>
#include <string.h> /* String function definitions */
#include <unistd.h> /* UNIX standard function definitions */
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
using namespace std;
/***********global variables*********/
#define pi 3.145159265359
int fd; /* File descriptor for the port */


#pragma pack(1)
typedef struct
{
unsigned char  Header1; //'T'
unsigned char  Header2;
unsigned char  Class;
unsigned char  ID;
unsigned short Length;
unsigned char  Payload[49];
unsigned short  CheckSum;//CheckSum[2];
}sDataLink;
#pragma pack()
/****************************************************************************************/
double x=0.04;
double beta=0.000003;
double dT = 0.01;
vector<vector<double> > alpha = {{0.0001,0,0},{0,0.0001,0},{0,0,0.0001}};//noise of accelerometer
vector<vector<double> > gyro_null={{0.0183},{0.0101},{0.0063}};//zero offset of gyroscope
vector<vector<double> > gyro_gain={{1/1.3096,0,0},{0,1/1.41875,0},{0,0,1/1.54607}};
vector<vector<double> > qq={{1},{0},{0},{0}};
vector<vector<double> > P={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
vector<vector<double> > Q={{x/dT,0,0,0},{0,x/dT,0,0},{0,0,x/dT,0},{0,0,0,x/dT}};
vector<vector<double> > Gyro={{0},{0},{0}};//the measurments of gyroscope
vector<vector<double> > Acc={{0},{0},{0}};
vector<vector<double> > H;
vector<vector<double> > F;
vector<vector<double> > E;
vector<vector<double> > K;
vector<vector<double> > Ouler;
/****** Function Declaration *********/
vector<vector<double> > Q_Convert_Ou(vector<vector<double> > quaternion);
vector<vector<double > > Normalize(vector<vector<double> > quater);
unsigned short CRC16(unsigned char *p, unsigned int length);
int open_port(char *port_device);


int main()
{
/****  initialization  ****/
H.resize(3);
for (int i=0;i<4;i++)
    H[i].resize(4);

F.resize(4);
for(int i=0;i<4;i++)
    F[i].resize(4);

E.resize(4);
for(int i=0;i<4;i++)
    E[i].resize(4);

K.resize(4);
for(int i=0;i<4;i++)
    K[i].resize(3);

Ouler.resize(3);
for(int i=0;i<3;i++)
    Ouler[i].resize(1);
/****  initialization  ****/

/*** read the data ***/
/***
 * read the data and return the struct
 * give the data to the A


/*** read the data ***/
    struct termios options,oldtio,op2;

    int fd=open_port("/dev/ttyUSB0");
    if(fd==-1){
        return -1;
    }

    tcgetattr(fd, &options);
if  ( tcgetattr( fd,&oldtio)  !=  0) {
            perror("SetupSerial 1");
            return -1;
        }
        bzero( &options, sizeof( options ) );
    //Set the baud rates to 115200...
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);


    //Enable the receiver and set local mode...
    options.c_cflag |= CLOCAL | CREAD;//(CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE; /* Mask the character size bits */
    options.c_cflag |= CS8; /* Select 8 data bits */

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
   // options.c_cflag &= ~CSIZE;
    //options.c_cflag |= CS8;
 options.c_cc[VTIME]  = 0;//重要
        options.c_cc[VMIN] = 100;//100;//返回的最小值  重要
        tcflush(fd,TCIFLUSH);

    //Set the new options for the port...
    tcsetattr(fd, TCSANOW, &options);

 /*******read step one*********************/
        float a, b, c,gx, gy, gz,ax, ay, az, mx, my, mz ;

        //unsigned char tt, ttt,lx,chx,chy;//8wei
    unsigned char t,m,q,q1,q2,q3;
        unsigned short mbd,mbd2;char buf[38];char *pos=buf;
        sDataLink gDataLinkPackage; /****struct**/
        unsigned short CheckSumTemp;//temp 16
        unsigned char ChecksumTem[2];//tep 8
   // ssize_t br;
       // FILE *dat; FILE *dat1; FILE *dat2;
    //dat=fopen("aa.txt","w");dat1=fopen("bb.txt","w");dat2=fopen("cc.txt","w");

      /**************begin while*************/
    while(1)
{
        ssize_t n=read(fd, &t, 1);
if(n>0){
               if(t=='T')
           {  read(fd, &m, 1);
               if(m=='M')
                {
                      read(fd, &q1, 1);
                      read(fd, &q2, 1);
                      read(fd, &mbd, 2);
                      read(fd, &q3, 1);
                                read(fd, &a, 4);
                                read(fd, &b, 4);
                read(fd, &c, 4);
                                read(fd, &gx, 4);
                read(fd, &gy, 4);
                read(fd, &gz, 4);
                read(fd, &ax, 4);
                read(fd, &ay, 4);
                read(fd, &az, 4);
                read(fd, &mx, 4);
                read(fd, &my, 4);
                read(fd, &mz, 4);
                read(fd, &mbd2, 2);

/******************define sturct********************************/
//sDataLink gDataLinkPackage;
//unsigned short CheckSumTemp;
//unsigned char ChecksumTem[2];
gDataLinkPackage.Header1=t;
gDataLinkPackage.Header2=m;
gDataLinkPackage.Class=q1;
gDataLinkPackage.ID=q2;
gDataLinkPackage.Length=mbd;
gDataLinkPackage.Payload[0]=q3;
 /******read payload**************************************************/
unsigned char *p1 = (unsigned char *)&a;
unsigned char *p2 = (unsigned char *)&b;
unsigned char *p3 = (unsigned char *)&c;
unsigned char *p4= (unsigned char *)&gx;
unsigned char *p5= (unsigned char *)&gy;
unsigned char *p6= (unsigned char *)&gz;
unsigned char *p7= (unsigned char *)&ax;
unsigned char *p8= (unsigned char *)&ay;
unsigned char *p9= (unsigned char *)&az;
unsigned char *p10= (unsigned char *)&mx;
unsigned char *p11= (unsigned char *)&my;
unsigned char *p12= (unsigned char *)&mz;
for(int i=1;i<5;i++)
{
    gDataLinkPackage.Payload[i] = *p1++;//把相应地址中的数据保存到unsigned char数组中
}
 for(int i=5;i<9;i++)
{
    gDataLinkPackage.Payload[i] = *p2++;//把相应地址中的数据保存到unsigned char数组中
}
for(int i=9;i<13;i++)
{
    gDataLinkPackage.Payload[i] = *p3++;//把相应地址中的数据保存到unsigned char数组中
}
for(int i=13;i<17;i++)
{
    gDataLinkPackage.Payload[i] = *p4++;//把相应地址中的数据保存到unsigned char数组中
}
for(int i=17;i<21;i++)
{
    gDataLinkPackage.Payload[i] = *p5++;//把相应地址中的数据保存到unsigned char数组中
}
for(int i=21;i<25;i++)
{
    gDataLinkPackage.Payload[i] = *p6++;//把相应地址中的数据保存到unsigned char数组中
}
for(int i=25;i<29;i++)
{
    gDataLinkPackage.Payload[i] = *p7++;//把相应地址中的数据保存到unsigned char数组中
}
for(int i=29;i<33;i++)
{
    gDataLinkPackage.Payload[i] = *p8++;//把相应地址中的数据保存到unsigned char数组中
}
for(int i=33;i<37;i++)
{
    gDataLinkPackage.Payload[i] = *p9++;//把相应地址中的数据保存到unsigned char数组中
}
for(int i=37;i<41;i++)
{
    gDataLinkPackage.Payload[i] = *p10++;//把相应地址中的数据保存到unsigned char数组中
}
for(int i=41;i<45;i++)
{
    gDataLinkPackage.Payload[i] = *p11++;//把相应地址中的数据保存到unsigned char数组中
}
for(int i=45;i<49;i++)
{
    gDataLinkPackage.Payload[i] = *p12++;//把相应地址中的数据保存到unsigned char数组中
}
/************************read cheksum************************************************/
gDataLinkPackage.CheckSum=mbd2;
CheckSumTemp=CRC16(&gDataLinkPackage.Class,53);
if (CheckSumTemp==gDataLinkPackage.CheckSum)


{

/************************Kalman************************************************/

     Gyro[0][0]=gx,Gyro[1][0]=gy,Gyro[2][0]=gz;
     Gyro=matrix_multiply(gyro_gain,matrix_sub(Gyro,gyro_null));
     Gyro[0][0] *=dT;
     Gyro[1][0] *=dT;
     Gyro[2][0] *=dT;



     double sum1=ax*ax+ay*ay+az*az;
     Acc[0][0]=ax/sqrt(sum1);
    Acc[1][0]=-ay/sqrt(sum1);
    Acc[2][0]=-az/sqrt(sum1);


    H[0][0]=2*qq[2][0],H[0][1]=2*qq[3][0],H[0][2]=2*qq[0][0],H[0][3]=2*qq[1][0];
    H[1][0]=-2*qq[1][0],H[1][1]=-2*qq[0][0],H[1][2]=2*qq[3][0],H[1][3]=2*qq[2][0];
    H[2][0]=2*qq[0][0],H[2][1]=-2*qq[1][0],H[2][2]=-2*qq[2][0],H[2][3]=2*qq[3][0];


    F[0][0]=1,F[0][1]=-Gyro[0][0]/2,F[0][2]=-Gyro[1][0]/2,F[0][3]=-Gyro[2][0]/2;
    F[1][0]=Gyro[0][0],F[1][1]=1,F[1][2]=Gyro[2][0]/2,F[1][3]=-Gyro[1][0]/2;
    F[2][0]=Gyro[1][0]/2,F[2][1]=-Gyro[2][0]/2,F[2][2]=1,F[2][3]=Gyro[0][0]/2;
    F[3][0]=Gyro[2][0]/2,F[3][1]=Gyro[1][0]/2,F[3][2]=-Gyro[0][0]/2,F[3][3]=1;


    E[0][0]=-qq[1][0]/2,E[0][1]=-qq[2][0]/2,E[0][2]=-qq[3][0]/2;
    E[1][0]=qq[0][0]/2,E[1][1]=-qq[3][0]/2,E[1][2]=-qq[2][0]/2;
    E[2][0]=qq[3][0]/2,E[2][1]=qq[0][0]/2,E[2][2]=-qq[1][0]/2;
    E[3][0]=qq[2][0]/2,E[3][1]=qq[1][0]/2,E[3][2]=qq[0][0]/2;

    qq=matrix_multiply(F,qq);

    //normalize
    double sum2=qq[0][0]*qq[0][0]+qq[1][0]*qq[1][0]+qq[2][0]*qq[2][0]+qq[3][0]*qq[3][0];
    qq[0][0]=qq[0][0]/sqrt(sum2),qq[1][0]=qq[1][0]/sqrt(sum2),qq[2][0]=qq[2][0]/sqrt(sum2),qq[3][0]=qq[3][0]/sqrt(sum2);


    vector<vector<double> > E_new=matrix_multiply(E,matrix_Trans(E));
    E_new[0][0]=beta*E_new[0][0],E_new[0][1]=beta*E_new[0][1],E_new[0][2]=beta*E_new[0][2],E_new[0][3]=beta*E_new[0][3];
    E_new[1][0]=beta*E_new[1][0],E_new[1][1]=beta*E_new[1][1],E_new[1][2]=beta*E_new[1][2],E_new[1][3]=beta*E_new[1][3];
    E_new[2][0]=beta*E_new[2][0],E_new[2][1]=beta*E_new[2][1],E_new[2][2]=beta*E_new[2][2],E_new[2][3]=beta*E_new[2][3];
    E_new[3][0]=beta*E_new[3][0],E_new[3][1]=beta*E_new[3][1],E_new[3][2]=beta*E_new[3][2],E_new[3][3]=beta*E_new[3][3];

    P=matrix_add(matrix_multiply(matrix_multiply(F,P),matrix_Trans(F)),E_new);
    P=matrix_add(P,Q);

    double p1=2*(qq[1][0]*qq[3][0]+qq[0][0]*qq[3][0]);
    double p2=2*(qq[2][0]*qq[3][0]-qq[0][0]*qq[1][0]);
    double p3=qq[0][0]*qq[0][0]-qq[1][0]*qq[1][0]-qq[2][0]*qq[2][0]+qq[3][0]*qq[3][0];
    vector<vector<double> > res={{p1},{p2},{p3}};
    vector<vector<double> > y=matrix_sub(Acc,res);

    vector<vector<double> > S=matrix_add(matrix_multiply(matrix_multiply(H,P),matrix_Trans(H)),alpha);

    K=matrix_multiply(matrix_multiply(P,matrix_Trans(H)),matrix_inv(S));
    qq=matrix_add(qq,matrix_multiply(K,y));

    P=matrix_sub(P,matrix_multiply(matrix_multiply(K,H),P));

    /**** quaternion to ouler angles ***/
    qq=Normalize(qq);
    Ouler=Q_Convert_Ou(qq);
    /**** quaternion to ouler angles ***/


    cout<<"Roll:"<<a<<"  "<<"Pitch:"<<b<<"  "<<"Yaw:"<<c<<endl;
    cout<<"MyRoll:"<<Ouler[0][0]<<" "<<"MyPitch:"<<Ouler[1][0]<<" "<<"MyYaw:"<<Ouler[2][0]<<endl;
    cout<<endl;







 /************************Kalman************************************************/

}


                }
           }

            }

}


        cout<<"Finished The Attitude Estimation!\n";
        close(fd);

    }




vector<vector<double> > Q_Convert_Ou(vector<vector<double> > quaternion)
{
    vector<vector <double> > res;
    res.resize(3);
    for(int i=0;i<3;i++)
        res[i].resize(1);
     res[0][0] = atan2(2*(quaternion[0][0]*quaternion[1][0]+quaternion[2][0]*quaternion[3][0]),1-2*quaternion[1][0]*quaternion[1][0]-2*quaternion[2][0]*quaternion[2][0])*180/pi ;//roll angle
   // res[0][0]=asin(2*quaternion[0][0]*quaternion[1][0]+2*quaternion[2][0]*quaternion[3][0])*180/pi;

    res[1][0] = asin(2*(quaternion[1][0]*quaternion[3][0]-quaternion[0][0]*quaternion[2][0]))*180/pi ;//pitch angle
    res[2][0] = atan2(2*(quaternion[0][0]*quaternion[3][0]+quaternion[1][0]*quaternion[2][0]),1-2*quaternion[2][0]*quaternion[2][0]-2*quaternion[3][0]*quaternion[3][0])*180/pi ;//yaw angle
    return res;
}


vector<vector<double > > Normalize(vector<vector<double> > quaternion)
{
    vector<vector<double> > res;
    res.resize(4);
    for (int i=0;i<4;i++)
        res[i].resize(1);
    double k=quaternion[0][0]*quaternion[0][0]+quaternion[1][0]*quaternion[1][0]+quaternion[2][0]*quaternion[2][0]+quaternion[3][0]*quaternion[3][0];
    res[0][0] = quaternion[0][0]/sqrt(k);
    res[1][0] = quaternion[1][0]/sqrt(k);
    res[2][0] = quaternion[2][0]/sqrt(k);
    res[3][0] = quaternion[3][0]/sqrt(k);
    return res;
}


unsigned short CRC16(unsigned char *p, unsigned int length)
{
    unsigned short checksum;
    static unsigned short CRC16Table[256] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,

        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };
    //unsigned short checksum = 0;
checksum = 0;
    for( ; length > 0; length-- )
    {
        checksum = ( checksum >> 8 ) ^ CRC16Table[ (checksum & 0xFF) ^ *p ];
        p++;

    }

/*for(js=0 ; js < 20000; js++ )
    {
        printf("%d\n",js);
    }*/

    return checksum;
     //    printf("return\t\t");
}
/***************************************************************************************************/

int open_port(char *port_device)
{

    fd = open(port_device, O_RDWR | O_NONBLOCK);// | O_NDELAY);//O_NONBLOCK   O_NOCTTY
    if (fd == -1)
    {
        perror("open_port: Unable to open /dev/tty0 - ");
    }
    else
        fcntl(fd, F_SETFL, 0);

    return (fd);
}
