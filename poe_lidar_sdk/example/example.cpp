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


#include "poelidar.h"

#include <vector>
#include <string>


using namespace cpj::poelidar;


int main(int argc, char **argv)
{
    //ip and port of lidar
    std::string ip = "192.168.0.7";
    uint16_t port = 25168;

    //create instance
    printf("Create driver instance.\n");
    POELidarDriver *drv = POELidarDriver::CreateDriver(poelidar_model::M1);

    //connect to lidar
    printf("Connect to lidar %s : %d\n", ip.c_str(), port);
    if(!drv->connect(ip, port)){
        printf("Error, when connect to %s : %d\n", ip.c_str(), port);
        delete drv;
        return -1;
    }

    //start scan
    printf("Start scan...\n");
    if(!drv->start()) {
        printf("Error, when start lidar");
        delete drv;
        return -1;
    }


    while (true) {
        //fetch lidar data
        std::vector<poelidar_point_t>  laser_scan;
        if(!drv->getScanData(laser_scan)) {
            continue;
        }

        //print data info
        printf("Got valid laser scan, point count: %d \n", laser_scan.size());
        for (size_t i = 0; i < laser_scan.size(); i++) {
            float angle = laser_scan[i].angle;//unit: degree
            float range = laser_scan[i].range;//unit: mm
            float intensity = laser_scan[i].intensity;

            printf("point %d, angle:%f, range: %f, intensity:%f \n",i, angle, range, intensity);
        }
    }

    //stop
    drv->stop();
    drv->disconnect();

    delete drv;
    return 0;
}
