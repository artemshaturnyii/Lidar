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

#ifndef POELIDAR_IMPL_H
#define POELIDAR_IMPL_H

#include "../include/poelidar.h"

#if defined(_WIN32)
#include "net/win32/net.h"

#elif defined(__GNUC__)
#include "net/linux/net.h"
#else
#error "unsupported target"
#endif

#include "poelidar_protocol.h"
#include <mutex>
#include <thread>
#include <condition_variable>


namespace cpj { namespace poelidar {

class POELidarDriverImpl : public POELidarDriver
{
public:
    POELidarDriverImpl(int model=poelidar_model::M1);
    ~POELidarDriverImpl();

public:
    bool connect(const std::string ip, const uint16_t port = 2105, uint32_t flag = 0);
    bool disconnect();
    
    bool start();
    bool stop();
    
    bool getScanData(poelidar_point_t *buffer, int &count, int timeout_ms = 1000);
    bool getScanData(std::vector<poelidar_point_t> &laser_scan,  int timeout_ms = 1000);

    bool getScanSpeed(int & speed_Hz);
    bool setScanSpeed(int speed_Hz);

    bool setIP(std::string ip);

    bool getUdpPort(int & udp_port);
    bool setUdpPort(int udp_port);

private:
    void data_thread_F1_(void);
    void data_thread_P2_(void);
    void data_thread_M1_(void);


    bool tcp_read_data_(uint8_t * read_buff, int read_size, int & real_read_size, int timeout_ms = 1000);

    uint8_t m1_cal_crc8_(const uint8_t *data, uint16_t data_len);

    uint32_t get_ms_(void);

private:
    size_t               cached_data_count_;
    poelidar_point_t     cached_data_buf_[2048];

    std::vector<poelidar_point_t>  cached_laser_scan_;

    cpj::net::SocketClient *socket_;
    cpj::net::SocketClient *socket_udp_;

    bool  data_thread_enbale_;

    std::thread data_thread_;

    std::mutex mtx_;
    std::condition_variable cv_;
    std::condition_variable cv_end_;

    std::string ip_;
    int udp_port_;
    int model_;
    bool is_start_;
    bool is_connect_;

    uint8_t cmd_id_;
    uint8_t cmd_buff_[256];
    int cmd_buff_len_;
};

}}

#endif
