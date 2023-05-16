#ifndef WIFI_COM_UDPSERVER_H
#define WIFI_COM_UDPSERVER_H

/**
 * @file    UDPServer.h
 * @author  Lukas Vogel (vogellu@ethz.ch)
 * @brief   Class running a UDP server on the Berkeley Sockets API.
 */

#pragma once

#include <cstdint>
#include <sys/types.h>

/**
 * @brief Class running a UDP server on the Berkeley Sockets API.
 *
 * This server is used to communicate with the robotic systems that are controlled
 * through CRS. The setup of the communication works as follows:
 * - instantiate a UDPServer object and pass the port number that devices can connect to
 * - call setPortNum() if you need to change the port number
 * - call startListening() to open the server
 * - call send() to send a datagram to a host
 * - call pollReceive() to check if a new datagram is available, and if there is, to receive the data
 * - call stopListening() when shutting down the server
 * - destroy the UDPServer object
 */
class UDPServer
{
public:
  /**
   * @brief Construct a new UDPServer object
   */
  UDPServer();

  /**
   * @brief Construct a new UDPServer object.
   *
   * @param port The port where the server should listen on for connections.
   */
  UDPServer(int port);

  /**
   * @brief Start listening on the port specified in the constructor.
   * @returns true if the listening socket could be opened, false otherwise
   */
  bool startListening();

  /**
   * @brief Stop listening on the port and close the socket.
   */
  void stopListening();

  /**
   * @brief Set or change the port number that the server is listening on.
   * @note If the server has already started listening, this will close the
   *  opened socket and reopen a new one. Otherwise, call startListening() after
   *  this to open the socket.
   */
  void setPortNum(int port);

  /**
   * @brief Send a payload to another host.
   *
   * @param dest The destination host address to send the data to
   * @param bytes Pointer to the payload
   * @param num_bytes Number of bytes in payload
   * @returns true if the datagram was sent, false otherwise
   */
  bool send(struct sockaddr_storage* dest, unsigned char* bytes, uint8_t num_bytes);

  /**
   * @brief Check the server if new data arrived from another host.
   *
   * @param[out] from Pointer to location where the sender address can be stored.
   * @param[out] dest Address where the received data can be written
   * @param[in] max_bytes Maximum number of bytes to receive, >= strlen(dest)!
   * @returns ssize_t Number of bytes received from host or zero if none.
   *  Returns <0 if an error occurred.
   */
  ssize_t pollReceive(struct sockaddr_storage* from, unsigned char* dest, size_t max_bytes);

  /**
   * @brief Destroy the UDPServer object.
   *
   * This will close the socket and stop processing of all messages.
   */
  ~UDPServer();

private:
  /** Port number that the server listens on. */
  int udp_port_;

  /** UDP socket that the node opened. */
  int sock_ = -1;
};
#endif
