
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
// trying to make a class out of the example so I can use this upd server 

class SimpleServer {
 public:
	SimpleServer(const int PORT, const int MAXLINE);
	~SimpleServer();
	char* buffer;
	const int buffersize;
	socklen_t sockfd, len, n;
	struct sockaddr_in servaddr, cliaddr;	

	int receive();
};

