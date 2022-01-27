// trying to make a class out of the example so I can use this upd server 

namespace SimpleServer {
class SimpleServer {
 public:
	SimpleServer(const int PORT, const int MAXLINE);
	~SimpleServer();
	char* buffer;
	const int buffersize;
	socklen_t sockfd, len, n;
	struct sockaddr_in servaddr, cliaddr;	

	int receive();
}

}
