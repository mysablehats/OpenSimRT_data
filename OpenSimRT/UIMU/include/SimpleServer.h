#include <vector>
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
	SimpleServer(int, int );
	~SimpleServer();
	char* buffer;
	char* hello;
	int buffersize;
	socklen_t sockfd, len, n;
	struct sockaddr_in servaddr, cliaddr;	
	//std::vector<double> receive();
	bool receive();
	std::vector<double> output;

};

