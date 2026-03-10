/*
 * Copyright 2023 Shanghai CPJRobot Co., Ltd. All rights reserved.
 * http://www.cpjrobot.com
 * http://www.poelidar.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "net.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <string>
#include <iostream>

using namespace cpj::net;

SocketClient::SocketClient(){}

SocketClient::~SocketClient()
{
    ::close(sockfd_);
}
    
SocketClient::SocketClient(SocketType type)
{
    if(type == SocketType::UDP){
        sockfd_ = createSocket(SocketType::UDP);
        type_ = SocketType::UDP;
    }else{
        sockfd_ = createSocket(SocketType::TCP);
        type_ = SocketType::TCP;
    }

    if (sockfd_ == -1){
        std::cout << "Create socket faild!" <<std::endl;
    }

}

int SocketClient::createSocket(SocketType type)
{

    int socket_fd = -1;
    if(type == SocketType::UDP){
        socket_fd = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    }else{//default TCP
        socket_fd = ::socket(AF_INET, SOCK_STREAM, 0);

        int reuse = 1;
        ::setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR,(const void*)&reuse, (socklen_t)sizeof(int));

        int nodelay = 1;
        ::setsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY,(const void*)&nodelay, (socklen_t)sizeof(int));

        set_socket_timeout_(socket_fd, 2000);
    }

    if (socket_fd == -1){
        is_socket_created_ = false;
        std::cout << "Create ::socket faild!" <<std::endl;
    }else{
        is_socket_created_ = true;
    }

    return socket_fd;
}


bool SocketClient::connect(const std::string ip, const uint16_t port)
{

    if(!is_socket_created_){
        sockfd_ = createSocket(type_);
        if(-1 == sockfd_){
            return false;
        }
    }

    ip_ = ip;
    port_ = port;

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_port = htons(port);

    if(type_ == SocketType::UDP){
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        int result = ::bind(sockfd_, (struct sockaddr*)&address, sizeof(address));
        if (result == -1) {
            std::cout << "bind failed with error" <<std::endl;
            return false;
        }
        return true;
    }

    if (inet_pton(AF_INET, ip.c_str(), &address.sin_addr) <= 0) {
        printf("\nInvalid address/ Address not supported \n");
        return false;
    }

    int result = ::connect(sockfd_, (struct sockaddr*)&address, sizeof(address));
    if (result == -1) {
        std::cout << "connect socket faild!" <<std::endl;
        return false;
    }

    return true;
}

void SocketClient::close()
{
    ::close(sockfd_);
    is_socket_created_ = false;
}

bool SocketClient::waitfordata(uint32_t timeout_ms)
{
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sockfd_, &readfds);

    timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = ::select(sockfd_ + 1, &readfds, NULL, NULL, &tv);
    if(ret == -1) {
        std::cout << "waitfordata, select result: error!" << ret <<std::endl;
        return false;
    } else if(ret == 0) {//超时（timeout）
        std::cout << "waitfordata, select result: timeout!" << ret <<std::endl;
        return false;
    }

    return true;
}


int SocketClient::send(const uint8_t *data, int size)
{
    int real_send_size = ::send(sockfd_, (const void *)data, size, 0);
    if(real_send_size <= 0 ) {
        return 0;
    }
    return real_send_size;
}



int SocketClient::recv(unsigned char *data, int size)
{
    int real_recv_size = ::recv(sockfd_, (void *)data, size, 0);
    if (real_recv_size <= 0) {
        return 0;
    } else {
        return real_recv_size;
    }
}

int SocketClient::sendto(const uint8_t *data, int size)
{
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_port = htons(port_);
    if (inet_pton(AF_INET, ip_.c_str(), &address.sin_addr) <= 0) {
        printf("\nInvalid address/ Address not supported \n");
        return false;
    }

    int real_send_size = ::sendto(sockfd_, (const void *)data, size, 0, (struct sockaddr*)&address, sizeof(address));
    if (real_send_size <= 0 ) {
        return 0;
    } else {
        return real_send_size;
    }
}

int SocketClient::recvfrom(unsigned char *data, int size)
{
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_port = htons(port_);
#if 0
    if (inet_pton(AF_INET, ip_.c_str(), &address.sin_addr) <= 0) {
        printf("\nInvalid address/ Address not supported \n");
        return false;
    }
#else
    address.sin_addr.s_addr = htonl(INADDR_ANY);
#endif


    int SenderAddrSize = sizeof(address);
    int real_recv_size = ::recvfrom(sockfd_, (void *)data, size, 0, (struct sockaddr*)&address, (socklen_t*)&SenderAddrSize);
    if (real_recv_size <= 0) {
        std::cout <<"SOCKET_ERROR recvfrom"<<std::endl;
        return 0;
    } else if (real_recv_size == 0){
        std::cout <<"0 recvfrom, socket type error:connect?" <<std::endl;
        return 0;
    } else {
        return real_recv_size;
    }
}


bool SocketClient::set_socket_timeout_(int sockfd, int timeout_ms)
{
    timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    //recv timeout
    if(0 != ::setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const void *)&tv, (socklen_t)sizeof(tv) ) ) {
        return false;
    }
    //send timeout
    if(0 != ::setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (const void *)&tv, (socklen_t)sizeof(tv) ) ) {
        return false;
    }

    return true;
}

