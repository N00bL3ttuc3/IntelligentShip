#include "rostostm32/rostostm32.h"

#define DEBUG 0

using namespace std;
using namespace boost::asio;
//Serial
boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyUSB0");
boost::system::error_code err;

//header and ender according to protocol
const unsigned char ender[2] = {0x0d, 0x0a};
const unsigned char header[2] = {0x55, 0xaa};

//Date to send
union sendData
{
	short d;
	unsigned char data[2];
}VelSet,YawSet;

//Data Received
union receiveData
{
	short d;
	unsigned char data[2];
}leftVelNow,rightVelNow,angleNow;


/**
 * @brief Init the configuration of serial port
 */
void serialInit()
{
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));    
}

/**
 * @brief write data to STM32
 * @param RobotV velocity
 * @param YawRate yaw
 * @param ctrlFlag
 */
void writeSpeed(double RobotV, double YawRate,unsigned char ctrlFlag)
{
    unsigned char buf[11] = {0};
    int i, length = 0;
    VelSet.d = (short)RobotV;
    YawSet.d = (short)YawRate;
    


    // protocol header
    for(i = 0; i < 2; i++)
        buf[i] = header[i];             //buf[0]  buf[1]
    
    // length of data
    length = 5;
    buf[2] = length;                    //buf[2]
    for(i = 0; i < 2; i++)
    {
        buf[i + 3] = VelSet.data[i];  //buf[3] buf[4]
        buf[i + 5] = YawSet.data[i]; //buf[5] buf[6]
    }
    // ctrlFlag
    buf[3 + length - 1] = ctrlFlag;       //buf[7]

    // crc check
    buf[3 + length] = getCrc8(buf, 3 + length);//buf[8]
    buf[3 + length + 1] = ender[0];     //buf[9]
    buf[3 + length + 2] = ender[1];     //buf[10]

    // send data throught serial port
    boost::asio::write(sp, boost::asio::buffer(buf));
    printf("Send data is : %x %x %x %x %x %x %x %x %x %x %x \n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10]);
    //std::cout<<buf[0]<<buf[1]<<buf[2]<<buf[3]<<buf[4]<<buf[5]<<buf[6]<<buf[7]<<buf[8]<<buf[9]<<buf[10]<<std::endl;
}
/**
 * @brief read data from STM32
 * @param vx
 * @param vth
 * @param th
 * @param ctrlFlag
 * @return
 */
bool readSpeed(double &vx,double &vth,double &th,unsigned char &ctrlFlag)
{
    char i, length = 0;
    unsigned char checkSum;
    unsigned char buf[150]={0};
    //try read data from ttyUSB
    try
    {
        boost::asio::streambuf response;
        boost::asio::read_until(sp, response, "\r\n",err);   
        copy(istream_iterator<unsigned char>(istream(&response)>>noskipws),
        istream_iterator<unsigned char>(),
        buf); 
    }  
    catch(boost::system::system_error &err)
    {
        ROS_INFO("read_until error");
    } 
    //=========================================================        

    // check header
    if (buf[0]!= header[0] || buf[1] != header[1])   //buf[0] buf[1]
    {
        ROS_ERROR("Received message header error!");
        return false;
    }
    // data length
    length = buf[2];                                 //buf[2]

    // check crc
    checkSum = getCrc8(buf, 3 + length);             //buf[10]
    if (checkSum != buf[3 + length])                 //buf[10]
    {
        ROS_ERROR("Received data check sum error!");
        return false;
    }    

    //read data
    for(i = 0; i < 2; i++)
    {
        leftVelNow.data[i]  = buf[i + 3]; //buf[3] buf[4]
        rightVelNow.data[i] = buf[i + 5]; //buf[5] buf[6]
        angleNow.data[i]    = buf[i + 7]; //buf[7] buf[8]
#if DEBUG
        printf("buf[5] = %x,buf[6] = %x\r\n",buf[5],buf[6]);
#endif
    }

    // control flag
    ctrlFlag = buf[9];


   //post-process
    vx = leftVelNow.d;
    vth =rightVelNow.d;
    th  = angleNow.d;

    return true;
}
/**
 * @brief crc check
 * @param ptr
 * @param len
 * @return
 */
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else 
                crc >>= 1;
        }
    }
    return crc;
}
