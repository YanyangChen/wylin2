
//compile with: gcc client.c sleep_time.c -o client  -lpthread
//run with: ./client
//note: run the server first before running this!!
//note: if connection is lost, restart both the ./server and ./client

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include "sleep_time.h"


















































































//client

#define MY_PORT	8765//1234

//void *fast_send(void *ptr); // fast loop... intended to send the force data
//void *slow_receive(void *ptr); // slow loop... intended to receive position measurements

// needed for socket creation and usage
int connectionFd, _true = 1;
struct sockaddr_in servaddr;
char in_buffer[1024], out_buffer[1024];
double var[3] = {2.2};



int clientfunc(void)
{
	int rc1; // For the returned values
	pthread_t thread1; // Threads pointer's
	
	// create socket
	if ( (connectionFd = socket( AF_INET, SOCK_STREAM, 0 )) == -1 ) {
		printf("Error creating socket\n");
		exit(1);	
	}
	
		// set socket port options
	if ( setsockopt(connectionFd, SOL_SOCKET, SO_REUSEADDR, &_true, sizeof(int)) == -1 ) {
		printf("Error setting socket port options\n");		
		exit(1);
	}
	
	// initialise address
	memset( &servaddr, 0, sizeof(servaddr) ); // fill with 0's
	servaddr.sin_family = PF_INET; // IPv4
	servaddr.sin_port = htons( MY_PORT ); // port
	servaddr.sin_addr.s_addr = inet_addr( "192.168.1.106" ); // host to network long------------------------------

	// new connected socket... WAITS HERE UNTIL CONNECTED
	if ( connect( connectionFd, (struct sockaddr *)&servaddr, sizeof(servaddr) ) == -1) {
		printf("Error while connecting\n");
		exit(1);
	}
	else
		printf("Connection established\n");
	
	// create fast thread 
	//if( (rc1=pthread_create( &thread1, NULL, &slow_receive, NULL )) )
	//{
	//	printf( "Thread creation failed: %d\n", rc1 );
		//exit(1);
	//}

	/* Wait till threads are complete before main continues. Unless we  */
	/* wait we run the risk of executing an exit which will terminate   */
	/* the process and all threads before the threads have completed.   */
	pthread_join( thread1, NULL ); 

	close( connectionFd	); // close connection with server
	printf("\n closed connection\n");

	return 0;
}

//~ void *slow_receive(void *ptr)
//~ {
	//~ while (1) {
		//~ // first send
		//~ memset( &out_buffer, 0, sizeof(out_buffer) ); // reset memory to 0's
		//~ sprintf( out_buffer, "%f %d\n", var[0], 0 ); // fill buffer
		//~ send( connectionFd, out_buffer, strlen(out_buffer), 0 ); // send the message
		//~ sleep_time(0.1); // wait 0.1 sec
//~ 
		//~ // then receive	
		//~ memset( &in_buffer, 0, sizeof(in_buffer) ); // reset memory to 0's
		//~ recv( connectionFd, in_buffer, sizeof(in_buffer), 0 ); // receive message
		//~ printf("Received: %s\n", in_buffer);
		//~ sleep_time(0.1);
	//~ }
//~ }



//~ void slow_receive(void *ptr)
//~ {
	//~ while (1) {
		// first send
		//~ memset( &out_buffer, 0, sizeof(out_buffer) ); // reset memory to 0's
		//~ sprintf( out_buffer, "%f %d\n", var[0], 0 ); // fill buffer
		//~ send( connectionFd, out_buffer, strlen(out_buffer), 0 ); // send the message
		//~ sleep_time(0.1); // wait 0.1 sec

		// then receive	
		//~ memset( &in_buffer, 0, sizeof(in_buffer) ); // reset memory to 0's
		//~ recv( connectionFd, in_buffer, sizeof(in_buffer), 0 ); // receive message
		//~ printf("Received: %s\n", in_buffer);
		//~ sleep_time(0.1);
	//~ 
//~ }
