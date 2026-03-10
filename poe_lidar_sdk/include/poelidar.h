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
 

#ifndef POELIDAR_H
#define POELIDAR_H

#ifndef __cplusplus
#error "The POELIDAR SDK requires a C++ compiler to be built"
#endif


#include <stdint.h>
#include <string>
#include <vector>


namespace cpj { namespace poelidar {


struct poelidar_point_t {
    float      intensity;
    float      range;//unit: mm
    float      angle;//unit: degree
};

enum poelidar_model{
    T1 = 0,
    F1 = 1,
    M1 = 2,
    P2 = 3
};

class POELidarDriver {
public:
    static POELidarDriver * CreateDriver(int model=poelidar_model::M1);

    static void DisposeDriver(POELidarDriver * drv);
    
public:
    virtual bool connect(const std::string ip, const uint16_t port, uint32_t flag = 0) = 0;
    virtual bool disconnect() = 0;
        
    virtual bool start() = 0;
    virtual bool stop() = 0;
    
    virtual bool getScanData(poelidar_point_t * buffer, int & count, int timeout_ms = 1000) = 0;
    virtual bool getScanData(std::vector<poelidar_point_t> & laser_scan,  int timeout_ms = 1000) = 0;

    virtual bool getScanSpeed(int & speed_Hz) = 0;
    virtual bool setScanSpeed(int speed_Hz) = 0;

    virtual bool setIP(std::string ip) = 0;

    virtual bool getUdpPort(int & udp_port) = 0;
    virtual bool setUdpPort(int udp_port) = 0;

    virtual ~POELidarDriver() {}
protected:
    POELidarDriver() {}

};

}}

#endif
