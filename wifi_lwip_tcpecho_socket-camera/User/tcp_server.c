#include "tcp_server.h"
#include "stm32f7xx.h"
#include "wifi_base_config.h"
#include "debug.h"
#include "camera_data_queue.h"


#include "tcpecho.h"
#include "lwip/opt.h"
#include "wifi_base_config.h"
#include "string.h"

#include "lwip/sys.h"
#include "lwip/api.h"
#include "./delay/core_delay.h"  

#define PRINTF printf

#define ssize_t int
	
#define kNotWritableErr             -6739 
#define kParamErr                   -6705
#define kUnderrunErr                -6750
#define kNoResourcesErr             -6729 

extern CircularBuffer cam_circular_buff;

#define tcp_server_log(M, ...)  custom_log("TCP", M, ##__VA_ARGS__)

#define SERVER_PORT 5001 /*set up a tcp server,port at 20000*/

#define NO_USED_BUFF_LEN    (512)
#define TCP_MAX_SEND_SIZE   (1024*50)

#define IsValidSocket( X )                  ( ( X ) >= 0 )


int32_t jpeg_send( int fd, const uint8_t *inBuf, size_t inBufLen )
{
    int32_t err = kParamErr;
    ssize_t writeResult;
    int selectResult;
    size_t numWritten;
    fd_set writeSet;
		struct timeval t;

    require( fd>=0, exit );
    require( inBuf, exit );
    require( inBufLen, exit );

    err = kNotWritableErr;

    t.tv_sec = 5;
    t.tv_usec = 0;
    numWritten = 0;

	
	int time1=0,time2=0;	
		
    while( numWritten < inBufLen )
    {

time1=HAL_GetTick();			
        FD_ZERO( &writeSet );
        FD_SET( fd, &writeSet );

				selectResult = select( fd + 1, NULL, &writeSet, NULL,&t );
			
        require( selectResult >= 1, exit );//log��ʾ���������

time2=HAL_GetTick();
printf("---------------------------->>>use time is %d <<<-\r\n",(time2-time1));
time1=0;
time2=0;	
			
        if(FD_ISSET( selectResult, &writeSet ))
        {
            writeResult = write( fd, (void *)( inBuf + numWritten ), ( inBufLen - numWritten ) );
				
            require( writeResult > 0, exit );
				
            numWritten += writeResult;
        }
				
    }

    require_action( numWritten == inBufLen, exit, tcp_server_log("ERROR: Did not write all the bytes in the buffer. BufLen: %zu, Bytes Written: %zu", inBufLen, numWritten ); err = kUnderrunErr );

    err = kNoErr;

exit:
    return err;
}
int32_t jpeg_tcp_send( int fd, const uint8_t *inBuf, size_t inBufLen )
{
    uint32_t i = 0, count = 0, index = 0;
    int32_t err = kGeneralErr;


	
		count =  inBufLen / TCP_MAX_SEND_SIZE;//������Ҫ���͵İ���
	
    for(i = 0; i < count; i ++)//������
    {
         err = jpeg_send(fd, inBuf + index, TCP_MAX_SEND_SIZE);

         require( err == kNoErr, exit);
         index = index + TCP_MAX_SEND_SIZE;
    }
		

    if((inBufLen % TCP_MAX_SEND_SIZE) != 0)//����һ��
    {
        err = jpeg_send(fd, inBuf + index, inBufLen % TCP_MAX_SEND_SIZE);

        require( err == kNoErr, exit);
    }
		


 exit:
    return err;
}


void jpeg_socket_close(int* fd)
{
    int tempFd = *fd;
    if ( tempFd < 0 )
        return;
    *fd = -1;
    close(tempFd);
}
extern __align(4) uint8_t queue_buff[CAMERA_QUEUE_NUM][CAMERA_QUEUE_DATA_LEN];

int send_fream=0;
/* TCP server listener thread */
int cbReadFinish_num=0;
void tcp_server_thread( void *arg )
{
		static int  num=0;
    int32_t err = kNoErr;
    struct sockaddr_in server_addr,client_addr;
    socklen_t sockaddr_t_size;
    char  client_ip_str[16];
    int sock = -1, client_fd = -1;
//    IPStatusTypedef para;
    int32_t camera_data_len = 0, i = 0;
    uint8_t *in_camera_data = NULL;
    uint8_t packet_index = 0;
    uint8_t *no_used_buff = NULL;
		int start_time=0, end_time=0, use_time=0;

//    micoWlanGetIPStatus(&para, Station);
//    tcp_server_log("TCP server ip:%s, port:%d", para.ip, SERVER_PORT);     //��ӡ����IP�Ͷ˿�
		PRINTF("���ض˿ں���%d\n\n",SERVER_PORT);
	
	  no_used_buff = pvPortMalloc(NO_USED_BUFF_LEN);
		memset(no_used_buff, 0, NO_USED_BUFF_LEN);
		if (no_used_buff == NULL)
		{
				PRINTF("No memory\n");
//				goto __exit;
		}
    sock = socket( AF_INET, SOCK_STREAM, IPPROTO_TCP );
		if (sock < 0)
		{
				PRINTF("Socket error\n");
				//goto __exit;
		}
	
		server_addr.sin_family = AF_INET;
		server_addr.sin_addr.s_addr = INADDR_ANY;
		server_addr.sin_port = htons(SERVER_PORT);
		memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));
	
		if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
		{
				PRINTF("Unable to bind\n");
		}


		if (listen(sock, 5) == -1)
		{
				PRINTF("Listen error\n");
		}
		
		printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\r\n");
		cbPrint(&cam_circular_buff) ;//���
    while(1)
    {
				sockaddr_t_size = sizeof(struct sockaddr_in);
        client_fd = accept( sock, (struct sockaddr *)&client_addr, &sockaddr_t_size );
			  PRINTF("new client connected from (%s, %d)\n",
			            inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
				{
					int flag = 1;
					
					setsockopt(client_fd,
										 IPPROTO_TCP,     /* set option at TCP level */
										 TCP_NODELAY,     /* name of option */
										 (void *) &flag,  /* the cast is historical cruft */
										 sizeof(int));    /* length of option value */
				}

        if( IsValidSocket( client_fd ) )
        {			
            tcp_server_log( "TCP Client %s:%d connected, fd: %d", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port), client_fd );

						cbPrint(&cam_circular_buff) ;//���
						printf("-------------------------\r\n");
					
            while(1)
            {
                //2.���ݳ����� 
								
                err = pull_data_from_queue(&in_camera_data, &camera_data_len);
												
                if(err != kNoErr)
                {
										//���¶�ָ��		
										cbReadFinish(&cam_circular_buff);  

										continue;
                }
									cbPrint(&cam_circular_buff) ;//���
												
                //3.��������

                if((err = jpeg_tcp_send(client_fd, (const uint8_t *)in_camera_data, camera_data_len)) != kNoErr)
								{
										//���¶�ָ��		
										cbReadFinish(&cam_circular_buff);

										printf("error-->[%d]%d KB\r\n", packet_index, camera_data_len/1024);
										break;
                }					
							
								cbReadFinish(&cam_circular_buff);

								
								send_fream++;
								

								//4.���ͼ������
//								if((err = jpeg_send(client_fd, (const uint8_t *)no_used_buff, NO_USED_BUFF_LEN)) != kNoErr)
//								{
//																				//���¶�ָ��		
//										cbReadFinish(&cam_circular_buff);
//										printf("error-->[%d]\r\n", packet_index);
//										break;
//								}

                //���¶�ָ��		

            }

            jpeg_socket_close( &client_fd );

        }
    }

 exit:
    if( err != kNoErr )
    {
        printf( "Server listerner thread exit with err: %d\r\n", err );
    }

    jpeg_socket_close( &sock );	
		
}






