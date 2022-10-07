/***************************************************************************//**
*  \file       chat.c
*
*  \details    nrf24l01p user space application
*
*  \author     Jagath
*
* *******************************************************************************/
#include <assert.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

int main(void)
{
	char txdata[32], ch;
	char rxdata[32];
	int fd, ret, n;
	struct pollfd pfd;
	struct pollfd stdin_poll = { STDIN_FILENO, POLLIN|POLLPRI };

	fd = open("/dev/nrf24l0.0", O_RDWR | O_NONBLOCK);

	if( fd == -1 )
	{
		perror("open");
		exit(EXIT_FAILURE);
	}

	pfd.fd = fd;
	pfd.events = ( POLLIN | POLLRDNORM | POLLOUT | POLLWRNORM );

		printf("Usage:\n1.By default application will be in receive\n"
			       "2.Enter data to transmit\n");

		puts("Starting poll...");


	while( 1 )
	{
		memset(txdata, 0, sizeof(txdata));
		memset(rxdata, 0, sizeof(rxdata));

		ret = poll(&pfd, (unsigned long)1, 10);   //wait for 5secs
		if( ret < 0 )
		{
			perror("pollfd");
			assert(0);
		}

		ret = poll(&stdin_poll, (unsigned long)1, 10);   //wait for 5secs
		if( ret < 0 )
		{
			perror("pollfd");
			assert(0);
		}

		if (stdin_poll.revents & POLLIN){
			{
				scanf("%s",txdata);
				printf("Transmitting data: %s\n",txdata);
				write(pfd.fd, &txdata, strlen(txdata));
			}
		}

		if( ( pfd.revents & POLLIN )  == POLLIN ) {
			read(pfd.fd, &rxdata, sizeof(rxdata));
			printf("Received data = %s\n", rxdata);
			}
	}
	return 0;
}
