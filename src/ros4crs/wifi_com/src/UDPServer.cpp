/**
 * @file    UDPServer.c
 * @author  Lukas Vogel (vogellu@ethz.ch)
 * @brief   Class running a UDP server on the Berkeley Sockets API.
 */

#include "wifi_com/UDPServer.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define UDP_SERVER_DEFAULT_PORT_NUM 20211

UDPServer::UDPServer() : udp_port_(UDP_SERVER_DEFAULT_PORT_NUM)
{
  return;
}

UDPServer::UDPServer(int port = UDP_SERVER_DEFAULT_PORT_NUM) : udp_port_(port)
{
  return;
}

UDPServer::~UDPServer()
{
  stopListening();
}

bool UDPServer::startListening()
{
  // Make sure to close the socket that was previously opened in case this function is called twice
  if (sock_ != -1)
  {
    close(sock_);
    sock_ = -1;
  }

  // Open UDP socket (use SOCK_STREAM for TCP experimentation)
  sock_ = socket(PF_INET, SOCK_DGRAM, 0);

  if (sock_ < 0)
  {
    std::cerr << "Could not open UDP socket! Errno: " << errno << " (" << strerror(errno) << ")\n";
    return false;
  }

  struct sockaddr_in si;
  memset(&si, 0, sizeof(si));
  si.sin_family = AF_INET;                 // address family
  si.sin_addr.s_addr = htonl(INADDR_ANY);  // as server, accept from anyone
  si.sin_port = htons(udp_port_);          // port we're opening

  // make socket reusable so that it won't fail if we restart ROS
  int reuse = 1;
  if (setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, (char*)&reuse, sizeof(int)) < 0)
  {
    std::cerr << "Could not set socket re-use option! Errno: " << errno << " (" << strerror(errno) << ")\n";
    return false;
  }

  // Marking the socket as non-blocking, so that it can be polled
  int flags = fcntl(sock_, F_GETFL);
  if (fcntl(sock_, F_SETFL, flags | O_NONBLOCK) == -1)
  {
    std::cerr << "Unable to set socket non blocking! Errno: " << errno << " (" << strerror(errno) << ")\n";
    return false;
  }

  if (bind(sock_, (struct sockaddr*)&si, sizeof(si)) < 0)
  {
    std::cerr << "Failed to bind to socket! Errno: " << errno << " (" << strerror(errno) << ")\n";
    return false;
  }

  return true;
}

void UDPServer::stopListening()
{
  if (sock_ != -1)
  {
    close(sock_);
    sock_ = -1;
  }
}

void UDPServer::setPortNum(int port)
{
  udp_port_ = port;

  if (sock_ != -1)
  {
    stopListening();
    startListening();
  }
}

bool UDPServer::send(struct sockaddr_storage* dest, unsigned char* bytes, uint8_t num_bytes)
{
  if (!dest || !bytes || sock_ == -1)
  {
    return false;
  }

  if (dest->ss_family == AF_INET || dest->ss_family == AF_INET6)
  {
    size_t client_len = sizeof(struct sockaddr);
    ssize_t sent = sendto(sock_, bytes, num_bytes, 0, (struct sockaddr*)dest, client_len);

    if (sent < 0)
    {
      std::cerr << "Error occurred during UDP sending: " << errno << " (" << strerror(errno) << ")\n";
      return false;
    }
    if (sent < num_bytes)
    {
      // Because this only supports sending up to 255 bytes, it SHOULD probably
      // be able to send all bytes every time. However, in case this becomes
      // an issue we could also implement sending data in multiple datagrams.
      std::cerr << "Not all bytes sent!";
      return false;
    }

    return true;
  }

  // don't support other address families than IPv4 and IPv6 currently
  return false;
}

ssize_t UDPServer::pollReceive(struct sockaddr_storage* from, unsigned char* dest, size_t max_bytes)
{
  if (!from || !dest)
  {
    return -1;
  }

  socklen_t client_len = sizeof(struct sockaddr_storage);
  int received = recvfrom(sock_, dest, max_bytes - 1, 0, (struct sockaddr*)from, &client_len);

  if (received < 0)
  {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINPROGRESS)
    {
      // this is fine, the receive would block and instead
      // returned
      return 0;
    }
    // connection broke, socket closed or other error
    std::cerr << "Receival failed. Errno: " << errno << " (" << strerror(errno) << ")\n";
    return -2;
  }
  else if (received == 0)
  {
    // socket may have been closed
    std::cerr << "recv_len == 0, socket closed? errno " << errno << "\n";
    return 0;
  }
  else
  {
    // We got data! Nul-terminate so it can be printed and hand number of received bytes to caller
    dest[received] = '\0';
    return received;
  }
}
