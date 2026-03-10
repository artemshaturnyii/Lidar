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

#include <string>


namespace cpj { namespace net {

enum SocketType{
    TCP = 0,
    UDP = 1
};


class SocketClient
{
public:
    SocketClient();
    ~SocketClient();

    SocketClient(SocketType type=SocketType::TCP);

    int createSocket(SocketType type=SocketType::TCP);

    bool connect(const std::string ip, const uint16_t port);

    void close();

    bool waitfordata(uint32_t timeout_ms = 1000);

    int send(const uint8_t *data, int size);
    int recv(unsigned char *data, int size);

    int sendto(const uint8_t *data, int size);
    int recvfrom(unsigned char *data, int size);

private:
    bool set_socket_timeout_(int sockfd, int timeout_ms);

private:
    int             sockfd_;
    SocketType      type_;
    bool            is_socket_created_;
    std::string     ip_;
    uint16_t        port_;

};

}}
