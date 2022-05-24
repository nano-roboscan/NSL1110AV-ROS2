#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <sys/socket.h> // for socket(), bind(), and connect()
#include <arpa/inet.h>  // for sockaddr_in and inet_ntoa()
#include <string.h>     // for memset()
#include <netdb.h>      // for hosten

#ifndef _MSC_VER
#include <sys/poll.h>
#include <poll.h>
#endif

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



// Lidar Parameter
#define DIST6MM 0
#define MAX_DISTANCE_MM_12Mhz     12500
#define MAX_DISTANCE_MM_24Mhz     6250
#define MAX_PASE            30000
#define IMAGE_WIDTH         320
#define IMAGE_HEIGHT        240
#define IMAGE_SIZE          76800   // 320x240
#define DISTANCE_OFFSET       0     // 거리정보의 OFFSET 만큼 보정한다.
#define AMPLITUDE_THRESHOLD     50    // 경계값이하의 약한 신호는 사용하지 않음.
#define AMPLITUDE_OVERFLOW      60000   // 6XXXX이상은 overflow 인 픽셀을 나타냄

#define DISTANCE_MIN_THRESHOLD    300   // 가까운 거리는 사용하지 않음.
#define DISTANCE_MAX_THRESHOLD    12500   // 먼거리 데이터를 제한함.
#define BUFF_SIZE          153600




static rclcpp::Clock s_rclcpp_clock; // replacement for ros::Time::now()

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pointcloud_publisher;

  static uint8_t *distanceBuff;
  static uint8_t *amplutudeBuff;
  static uint8_t *response;
  static int *dist;
class RoboScanPublisher : public rclcpp::Node { 

  // TCP
  //Tcp m_tcp;
  std::string m_ipAddress;
  uint16_t m_portNumber;
  
  //static int *dist;


#ifndef _MSC_VER  
  int16_t m_connectionSocket; // Socket, wenn wir der Client sind (z.B. Verbindung zum Scanner)
#else
  SOCKET m_connectionSocket;  // Socket, wenn wir der Client sind (z.B. Verbindung zum Scanner)
#endif




//Lidar Tcp서버 연결
bool opentcpConnection(std::string ipAddress, uint16_t port)
{
     //bool status = false;

     m_connectionSocket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);


     if (m_connectionSocket  < 0)
     {
        //printError("Tcp::open: socket() failed, aborting.");
        return false;
     }


     struct sockaddr_in addr;
     struct hostent *server;
     server = gethostbyname(ipAddress.c_str());
     memset(&addr, 0, sizeof(addr));         // Zero out structure
     addr.sin_family = AF_INET;

  #ifdef _MSC_VER
     memcpy((char *)&addr.sin_addr.s_addr, (char *)server->h_addr,  server->h_length);
  #else
     bcopy((char *)server->h_addr, (char *)&addr.sin_addr.s_addr, server->h_length);
  #endif
     addr.sin_port = htons(port);        // Host-2-Network byte order



     int result = connect(m_connectionSocket, (sockaddr*)(&addr), sizeof(addr));
  
     if (result < 0)
     {
       // Verbindungsversuch ist fehlgeschlagen
      // std::string text = "Tcp::open: Failed to open TCP connection to " + ipAddress + ":" + toString(port) + ", aborting.";
       //printError(text);
       return false;
     }



     return true;
}


//disconnection
bool DisconnectTcp()
{
  return true;
}


//cmd 전송
void SendCommand( int16_t m_connectionSocket, std::string cmd, uint8_t len)
{
  //UINT8 sendBuffer[1024];
  // bool result;
  uint8_t bytesSent;

const char * byte_cmd = cmd.c_str();

//RCLCPP_INFO(this->get_logger(), "Tcp::write: command %s, %d, %d", byte_cmd, strlen(byte_cmd),len);
#ifdef _MSC_VER
  SOCKET* socketPtr = &m_connectionSocket;
  bytesSent = ::send(*socketPtr, (const char*)&byte_cmd[0], len, 0);
#else
  int16_t* socketPtr = &m_connectionSocket;
  bytesSent = ::send(*socketPtr, (const char*)&byte_cmd[0], len, 0);
#endif  
  // Sende Daten an das Socket
  if (bytesSent != len)
  {
    //printWarning("Tcp::write: Failed to send data to socket.");
  }
  else
  {
   // RCLCPP_INFO(this->get_logger(), "Tcp::write: Sent %d", bytesSent);
    //printInfoMessage("Tcp::write: Sent " + toString(len) + " bytes to client.", m_beVerbose);
  }


}



typedef struct _RGB888Pixel
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RGB888Pixel;



int Convert_To_RGB24_2( float fValue, RGB888Pixel *nRGBData, float fMinValue, float fMaxValue)
{
    if(fValue == 0) //Invalide Pixel
    {
        nRGBData->r = 0;//R
        nRGBData->g = 0;//G
        nRGBData->b = 0;//B
    }
    else if(fValue < fMinValue)
    {
        nRGBData->r = 255;//R
        nRGBData->g = 0;//G
        nRGBData->b = 0;//B
    }
    else if(fValue > fMaxValue)
    {
        nRGBData->r = 255;//R
        nRGBData->g = 0;//G
        nRGBData->b = 255;//B
    }
    else
    {
        float fColorWeight;
        fColorWeight = (fValue-fMinValue) / (fMaxValue-fMinValue);

        if( (fColorWeight <= 1.0f) && (fColorWeight > 0.8f) )
        {
            nRGBData->r = (unsigned char)(255 * ((fColorWeight - 0.8f) / 0.2f));//값에 따라 증가
            nRGBData->g = 0;
            nRGBData->b = 255;
        }
        else if( (fColorWeight <= 0.8f) && (fColorWeight > 0.6f) )
        {
            nRGBData->r = 0;
            nRGBData->g = (unsigned char)(255 * (1.0f - (fColorWeight - 0.6f) / 0.2f));//값에 따라 감소
            nRGBData->b = 255;
        }
        else if( (fColorWeight <= 0.6f) && (fColorWeight > 0.4f) )
        {
            nRGBData->r = 0;
            nRGBData->g = 255;
            nRGBData->b = (unsigned char)(255 * ((fColorWeight - 0.4f) / 0.2f));//값에 따라 증가
        }
        else if( (fColorWeight <= 0.4f) && (fColorWeight > 0.2f) )
        {
            nRGBData->r = (unsigned char)(255 * (1.0f - (fColorWeight - 0.2f) / 0.2f));//값에 따라 감소
            nRGBData->g = 255;
            nRGBData->b = 0;
        }
        else if( (fColorWeight <= 0.2f) && (fColorWeight >= 0.0f) )
        {
            nRGBData->r = 255;
            nRGBData->g = (unsigned char)(255 * ((fColorWeight - 0.0f) / 0.2f));//값에 따라 증가
            nRGBData->b = 0;
        }
        else
        {
            nRGBData->r = 0;
            nRGBData->g = 0;
            nRGBData->b = 0;
        }
    }

    return true;
}



inline int lidar_calcDistance(int phase)
{
  //RCLCPP_INFO(this->get_logger(), " lidar_calcDistance()");

#if 0 // Modulation 24Mhz
  int data = MAX_DISTANCE_MM_24Mhz * phase/MAX_PASE;
#else 
  int data = MAX_DISTANCE_MM_12Mhz * phase/MAX_PASE;
#endif

  if(phase < 0)
    return -1;

  if(data > DISTANCE_OFFSET) data -= DISTANCE_OFFSET;
  else data = 0;


  
  return data;
}



//frame data 수신을 위한 TCP 소켓 통신
int* roboscan_getDistanceAndAmplitudeSorted()
{

  uint16_t *pMemDistance = 0;
  uint16_t *pMemAmplitude = 0;
 // int *dist;
  bool isConn = false;

  isConn = opentcpConnection("192.168.7.2",50660);


  if(!isConn)
  {
    //RCLCPP_INFO(this->get_logger(), "Connection error!!....");
    return NULL;
  }
  else
  {
   // RCLCPP_INFO(this->get_logger(), "Connection !!....");
  }


  int nSize, recvMsgSize = 256 ;
  int sum = 0;
  uint8_t  buff[1024];


    //nSize = strlen("startVideo\n");
    //SendCommand(m_connectionSocket, "startVideo\n", nSize);
 
   // int count = 0;


    // rclcpp::WallRate loop_rate(500);
    // while (rclcpp::ok()) {
    //    count++;
    //    loop_rate.sleep();
    //    //RCLCPP_INFO(this->get_logger(), "loop() ");

    //    if(count > 100)break;
    // }




  nSize = strlen("getDistanceAndAmplitudeSorted\n");

  SendCommand(m_connectionSocket, "getDistanceAndAmplitudeSorted\n", nSize);


  while(recvMsgSize != 0)
  {
     // RCLCPP_INFO(this->get_logger(), "Tcp: recv");
      recvMsgSize = recv(m_connectionSocket, buff, 1024, 0);


      //if(recvMsgSize == 0) break;
      
      memcpy(&response[sum] , &buff[0] , sizeof(buff));
      sum += recvMsgSize;
  }

    if(sum < 10) return NULL;


    memcpy(&distanceBuff[0], &response[0], BUFF_SIZE);
    memcpy(&amplutudeBuff[0], &response[BUFF_SIZE], BUFF_SIZE);


    close(m_connectionSocket);


    pMemDistance = (uint16_t *)&distanceBuff[0];
    pMemAmplitude = (uint16_t *)&amplutudeBuff[0];
  

   // for(int j = 0; j < IMAGE_HEIGHT; j++)
   //  {       
   //     for(int k = 0; k < IMAGE_WIDTH; k++)
   //      {
   //        RCLCPP_INFO(this->get_logger(), " ++test--pMemDistance %d, %d", j,k);
   //       // dist[j*IMAGE_WIDTH + k] = 0;
   //        pMemDistance[j*IMAGE_WIDTH + k] = 0;
   //      }
  
   //  }


     rclcpp::WallRate loop_rate(200);
  #if 0//DIST6MM
      for(int y = 0; y < IMAGE_HEIGHT; y++)
      {
        for(int x = 0; x < IMAGE_WIDTH; x++)
        {
          // if(x == 160 && y == 120)
          //    printf("(160, 120) Distance : %%d\n", dist[320*y + x])


          if(pMemDistance[y*IMAGE_WIDTH + x] < AMPLITUDE_THRESHOLD)
          {
            // RCLCPP_INFO(this->get_logger(), " pMemDistance[y*IMAGE_WIDTH + x] < AMPLITUDE_THRESHOLD");
            dist[y*IMAGE_WIDTH + x] = 0;
          }
          // else if(pMemDistance[y*IMAGE_WIDTH + x] < DISTANCE_THRESHOLD)
          //  dist[y*IMAGE_WIDTH + x] = 0;
          else if(pMemAmplitude[y*IMAGE_WIDTH + x] > 28800) // max 6M
          {
            //RCLCPP_INFO(this->get_logger(), " pMemAmplitude[y*IMAGE_WIDTH + x] > 28800");
            dist[y*IMAGE_WIDTH + x] = 0;
          }
          else  
          {
            dist[y*IMAGE_WIDTH + x] = lidar_calcDistance(pMemDistance[y*IMAGE_WIDTH + x]);
          }
            
          //RCLCPP_INFO(this->get_logger(), " pMemDistance %d, %d", x,y);
          //loop_rate.sleep();
    //      if(x == 160 && y == 120)
    //        printf("(160, 120) Distance : %.2f cm\n", dist[320*y + x]*0.1);
        }
      }
#else //12Mhz
      for(int y = 0; y < IMAGE_HEIGHT; y++)
      {
        for(int x = 0; x < IMAGE_WIDTH; x++)
        {
          // bin error pixel
          if(pMemAmplitude[y*IMAGE_WIDTH + x] < AMPLITUDE_THRESHOLD)
            dist[y*IMAGE_WIDTH + x] = 0;
          else if(pMemAmplitude[y*IMAGE_WIDTH + x] > AMPLITUDE_OVERFLOW)
          {
            dist[y*IMAGE_WIDTH + x] = 0;
          
          }

          // calculate distance
          if(pMemDistance[y*IMAGE_WIDTH + x] != 0)        
            dist[y*IMAGE_WIDTH + x] = (MAX_DISTANCE_MM_12Mhz * pMemDistance[y*IMAGE_WIDTH + x])/MAX_PASE;
            // dist[y*IMAGE_WIDTH + x] = lidar_calcDistance(pMemDistance[y*IMAGE_WIDTH + x]);


          if(dist[y*IMAGE_WIDTH + x] < DISTANCE_MIN_THRESHOLD)
          {
            dist[y*IMAGE_WIDTH + x] = 0;
          }
          else if(dist[y*IMAGE_WIDTH + x] > DISTANCE_MAX_THRESHOLD)
            dist[y*IMAGE_WIDTH + x] = 0;
            

          // if(x == 160 && y == 120)
          //   printf("(160, 120) Distance : %.2f mm--***--\n", dist[320*y + x]);
        }
      }
#endif
    //RCLCPP_INFO(this->get_logger(), "return dist..");
    
    return dist;
}



public: 
  RoboScanPublisher() : Node("roboscan_node")
{ 

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  _pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("RoboScan_Point", qos_profile); 


  auto data_stamp = s_rclcpp_clock.now();

  RGB888Pixel* pTex = new RGB888Pixel[1];

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  
  cloud->width  = IMAGE_WIDTH;
  cloud->height = IMAGE_HEIGHT;
  cloud->points.resize (cloud->width * cloud->height);

  RCLCPP_INFO(this->get_logger(), "++Start RoboScanPublisher()....");


  while(true)
  {
       int *arr = roboscan_getDistanceAndAmplitudeSorted();

      
        if(arr == NULL)
        {
           RCLCPP_INFO(this->get_logger(), "arr == NULL....");
          continue;
        }


       //PCL변환
        int i = 0;
        for(int y = 0; y < IMAGE_HEIGHT; y++)
        {
          for(int x = 0; x < IMAGE_WIDTH; x++)
          {       
            // i = 320*y + (320-x);
            i = 320*y + x;
            Convert_To_RGB24_2((double)arr[i], pTex, 0.0f, 15000.0f);
 
            
            //if(x == 160 && y == 120)
              //RCLCPP_INFO(this->get_logger(), "(160, 120) Distance : %.2f cm\n", arr[320*y + (320-x)]*0.1);
            // Generate the data
            if(arr[i] == 0)
            {
              cloud->points[i].x = 0;
              cloud->points[i].z = 0;
              cloud->points[i].y = 0;
              cloud->points[i].b = 0;
              cloud->points[i].g = 0;
              cloud->points[i].r = 0;
            }
            else
            {
              cloud->points[i].x = (x-160)/50.0;
              cloud->points[i].z = (240-y)/50.0;
              cloud->points[i].y = arr[i]/500.0;
              cloud->points[i].b = pTex->b;
              cloud->points[i].g = pTex->g;
              cloud->points[i].r = pTex->r;
            }
          }
        }

       

       //RCLCPP_INFO(this->get_logger(), "PointCloud2 ....");
       
       sensor_msgs::msg::PointCloud2 msg;
       pcl::toROSMsg(*cloud, msg);
       msg.header.stamp = data_stamp;
       msg.header.frame_id = "/map";
       _pointcloud_publisher->publish (msg);  

         

  }


} 


};


void init_mem(void)
{
  // response = (uint8_t *)malloc(sizeof(char)*BUFF_SIZE*2);
  // distanceBuff = (uint8_t *)malloc(sizeof(char)*BUFF_SIZE);
  // amplutudeBuff = (uint8_t *)malloc(sizeof(char)*BUFF_SIZE);  


  response = new uint8_t[sizeof(char)*BUFF_SIZE*2];
  distanceBuff = new uint8_t[sizeof(char)*BUFF_SIZE];
  amplutudeBuff = new uint8_t[sizeof(char)*BUFF_SIZE];
  dist = new int[sizeof(char)*BUFF_SIZE*2];

  if(response == NULL || distanceBuff == NULL || amplutudeBuff == NULL || dist == NULL )
  {
    printf("response == NULL");
  }

}

void buffer_free(void)
{
  free(response);
  free(distanceBuff);
  free(amplutudeBuff);
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  init_mem();
  rclcpp::init (argc, argv);

  //auto node = rclcpp::Node::make_shared("roboscan_node");

  auto node = std::make_shared<RoboScanPublisher>();


  // Spin
  rclcpp::spin (node);
  rclcpp::shutdown(); 

  buffer_free();
  return 0;

}