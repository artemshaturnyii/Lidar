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


#include "poelidar_impl.h"

#include <string.h>
#include <iostream>
#include <chrono>

namespace cpj { namespace poelidar {


POELidarDriver*  POELidarDriver::CreateDriver(int model) {
    return new POELidarDriverImpl(model);
}

void POELidarDriver::DisposeDriver(POELidarDriver *drv) {
    delete drv;
}

POELidarDriverImpl::POELidarDriverImpl(int model)
{
    model_ = model;

    if (model_ == poelidar_model::F1) {
        socket_ = new cpj::net::SocketClient(cpj::net::SocketType::TCP);
        socket_udp_ = new cpj::net::SocketClient(cpj::net::SocketType::UDP);
    }

    if (model_ == poelidar_model::P2) {
        socket_ = new cpj::net::SocketClient(cpj::net::SocketType::TCP);
    }

    if (model_ == poelidar_model::M1) {
        socket_ = new cpj::net::SocketClient(cpj::net::SocketType::TCP);
    }

    is_start_ = false;
    is_connect_ = false;
    cmd_id_ = 0;
    cmd_buff_len_ = 0;
}

POELidarDriverImpl::~POELidarDriverImpl()
{
    disconnect();

    if(socket_){
        delete socket_;
        socket_ = NULL;
    }

    if(socket_udp_){
        delete socket_udp_;
        socket_udp_ = NULL;
    }

}

bool POELidarDriverImpl::connect(const std::string ip, const uint16_t port, uint32_t flag)
{
    if(is_connect_){return true;}

    switch (model_) {

      case poelidar_model::P2  : {
          if(!socket_->connect(ip, port)) {
              std::cout <<"[POELidarDriverImpl]connect failed!" << std::endl;
              return false;
          }
      } break;

      case poelidar_model::M1  : {
          if(!socket_->connect(ip, port)) {
              std::cout <<"[POELidarDriverImpl]connect failed!" << std::endl;
              return false;
          }
    } break;

      default :
          break;
    }

    ip_ = ip;

    is_connect_ = true;
    return true;
}

bool POELidarDriverImpl::disconnect()
{
    if(!is_connect_){return true;}

    stop();

    if (socket_) { socket_->close(); }

    if (socket_udp_) { socket_udp_->close(); }

    is_connect_ = false;

    return true;
}

bool POELidarDriverImpl::start()
{
    if(is_start_){return true;}

    switch (model_) {

    case poelidar_model::M1  : {
        //power on first
        int send_size = socket_->send(M1_CMD_TYPE_PW_ON, sizeof(M1_CMD_TYPE_PW_ON));
        if (send_size == 0) {
            std::cout <<"[POELidarDriverImpl]start(): send M1_CMD_TYPE_PW_ON error"<<std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        send_size = socket_->send(M1_CMD_TYPE_START, sizeof(M1_CMD_TYPE_START));
        if (send_size == 0) {
            std::cout <<"[POELidarDriverImpl]start(): send Head_M1_SCAN error"<<std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    }
    break;

    default :
        break;

    }

    data_thread_enbale_ = true;//enbale data thread

    switch (model_) {
      case poelidar_model::M1  : {
          data_thread_ = std::thread(&POELidarDriverImpl::data_thread_M1_, this);
      } break;

      default :
          break;
    }

    data_thread_.detach();

    is_start_ = true;

    return true;
}

bool POELidarDriverImpl::stop()
{
    if(!is_start_){return true;}

    data_thread_enbale_ = false;
    std::mutex m;
    std::unique_lock<std::mutex> lk(m);

    if (std::cv_status::timeout == cv_end_.wait_for(lk, std::chrono::milliseconds(1000))) {
        std::cout <<"[POELidarDriverImpl]stop data thread timeout!"<<std::endl;
        return false;
    }

    switch (model_) {

      case poelidar_model::M1  : {
          std::lock_guard<std::mutex> lk(mtx_);

          if(true) {
              int real_send_size = socket_->send(M1_CMD_TYPE_STOP, sizeof(M1_CMD_TYPE_STOP));
              if (real_send_size == 0) {
                  std::cout <<"[POELidarDriverImpl]stop(): send Head_M1_STOP error"<<std::endl;
                  return false;
              }
              std::cout <<"[POELidarDriverImpl]stop() OK!"<<std::endl;
          }
      } break;


      default :
          break;
    }

    is_start_ = false;

    return true;
}

bool POELidarDriverImpl::getScanData(poelidar_point_t *buffer, int &count, int timeout_ms)
{
    //TO DO
    return false;
}

bool POELidarDriverImpl::getScanData(std::vector<poelidar_point_t> &laser_scan,  int timeout_ms)
{
    {
    std::unique_lock<std::mutex> lk(mtx_);

    if(std::cv_status::timeout == cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms))) {
        std::cout <<"[POELidarDriverImpl]getScanData timeout!"<<std::endl;
        return false;
    }

    if (cached_laser_scan_.empty()) {return false;}

    std::cout <<"[POELidarDriverImpl::getScanData]Got valid laser_scan: "<<cached_laser_scan_.size()<<std::endl;

    laser_scan.clear();
    laser_scan = cached_laser_scan_;
    cached_laser_scan_.clear();

    }

    return true;
}

bool POELidarDriverImpl::getScanSpeed(int &speed_Hz)
{
    //TO DO
    return false;
}

bool POELidarDriverImpl::getUdpPort(int &udp_port)
{
    //TO DO
    return false;
}

bool POELidarDriverImpl::setIP(std::string ip)
{
    //TO DO
    return false;
}

bool POELidarDriverImpl::setScanSpeed(int speed_Hz)
{
    //TO DO
    return false;
}

bool POELidarDriverImpl::setUdpPort(int udp_port)
{
    //TO DO
    return false;
}

void POELidarDriverImpl::data_thread_F1_(void)
{
    //TO DO
    return;
}

void POELidarDriverImpl::data_thread_P2_(void)
{
    //TO DO
    return;
}

void POELidarDriverImpl::data_thread_M1_(void)
{
    printf("data_thread_M1_ begin\n");

    std::vector<poelidar_point_t>  laser_scan;
    size_t    cnt = 0;
    const int kSkipFrameCnt = 35;//skip unstable data at the beginning


    DataPackM1 m1_pack;
    unsigned char* recvBuffPtr = (unsigned char*)&m1_pack;
    float pre_frame_last_pt_angle = 0;

    while (data_thread_enbale_)
    {
        //read sync head
        bool is_read_head_OK = false;
        int real_recv_size = 0;
        if(tcp_read_data_(recvBuffPtr, 1, real_recv_size, 1000)){
            if(*recvBuffPtr == M1_RES_SYNC_HEADER){
                if(tcp_read_data_(recvBuffPtr+1, 1, real_recv_size, 1000)){
                    uint8_t value = *(recvBuffPtr + 1);
                    if ( (value & 0x1F) == M1_NODE_NUM_PER_PACK) {
                        is_read_head_OK = true;
                    } else {
                        continue;
                    }
                }
            }else{
                continue;
            }
        }else{
            std::cout<<"[POELidarDriverImpl::data_thread_M1_]Error:real_recv_size < pack_size!"<< std::endl;
            return;
        }


        if(!is_read_head_OK) return;


        //read remain data
        const size_t res_size = sizeof(m1_pack) - 2;
        if(!tcp_read_data_(recvBuffPtr+2, res_size, real_recv_size, 1000)){
            std::cout<<"[POELidarDriverImpl::data_thread_M1_]Error:real_recv_size < pack_size!"<< std::endl;
            return;
        }

        //crc check
        bool is_crc_ok = false;
        uint8_t crc_result = m1_cal_crc8_((const uint8_t *)(&m1_pack), sizeof(m1_pack)-1 );
        if (crc_result == m1_pack.CRC8) { is_crc_ok = true; }

        if (!is_crc_ok) {
            std::cout<<"[POELidarDriverImpl::data_thread_M1_]Error:CRC check failed!"<< std::endl;
            continue;
        }

        //parse data
        float angle_increment;
        if(m1_pack.endAngle < m1_pack.startAngle) {
            angle_increment = (float)(m1_pack.endAngle + 36000 - m1_pack.startAngle) / (float)(M1_NODE_NUM_PER_PACK - 1);
        } else {
            angle_increment = (float)(m1_pack.endAngle - m1_pack.startAngle) / (float)(M1_NODE_NUM_PER_PACK - 1);
        }

        //printf("m1 start angle: %f, end angle:%f\n", m1_pack.startAngle*0.01f, m1_pack.endAngle*0.01f);
        //printf("m1 angle increment: %f\n", angle_increment*0.01f);
        //printf("m1 speed: %f\n", m1_pack.speed);

        float angle_pre = pre_frame_last_pt_angle;
        bool is_got_one_frame = false;
        int cross_frame_pt_idx = 0;
        for (int pos = 0; pos < M1_NODE_NUM_PER_PACK; pos++) {
            float angle_now = ((float)m1_pack.startAngle + angle_increment*pos) * 0.01f;
            angle_now = (angle_now >= 360.0f) ? (angle_now - 360.0f) : angle_now;
            if (angle_now < angle_pre) {

                cnt++;
                //std::cout<<"[POELidarDriverImpl::data_thread_M1_]cnt:"<< cnt<<std::endl;

                if(cnt > kSkipFrameCnt) {
                    mtx_.lock();
                    cached_laser_scan_.clear();
                    cached_laser_scan_.resize(laser_scan.size());
                    cached_laser_scan_.assign(laser_scan.begin(), laser_scan.end());

                    cv_.notify_one();
                    mtx_.unlock();
                }

                laser_scan.clear();

                is_got_one_frame = true;
                cross_frame_pt_idx = pos;
                break;
            }

            poelidar_point_t pt;
            pt.intensity = m1_pack.nodes[pos].confidence;
            pt.range = m1_pack.nodes[pos].distance;
            pt.angle = angle_now;

            laser_scan.push_back(pt);

            angle_pre = angle_now;
       }

       if (is_got_one_frame) {
            for (int pos = cross_frame_pt_idx; pos < M1_NODE_NUM_PER_PACK; pos++) {
                float angle_now = ((float)m1_pack.startAngle + angle_increment*pos) * 0.01f;
                angle_now = (angle_now >= 360.0f) ? (angle_now - 360.0f) : angle_now;

                poelidar_point_t pt;
                pt.intensity = m1_pack.nodes[pos].confidence;
                pt.range = m1_pack.nodes[pos].distance;
                pt.angle = angle_now;

                laser_scan.push_back(pt);
            }
        }

        pre_frame_last_pt_angle = m1_pack.endAngle * 0.01f;
    }

    data_thread_enbale_ = false;
    return;
}

bool POELidarDriverImpl::tcp_read_data_(uint8_t * read_buff, int read_size, int & real_read_size, int timeout_ms)
{
    uint32_t startTime = get_ms_();

    int recv_size = 0;
    int remain_size = read_size;
    int data_pos = 0;
    while ((get_ms_() - startTime) <= timeout_ms) {
        if(!socket_->waitfordata(timeout_ms)) {
            return false;
        }
        recv_size = socket_->recv((unsigned char*)(read_buff+data_pos), remain_size);
        data_pos = data_pos + recv_size;
        remain_size = remain_size - recv_size;

        real_read_size = read_size - remain_size;
        if(remain_size <= 0){
            return true;
        }
    }

    return false;
}

uint8_t POELidarDriverImpl::m1_cal_crc8_(const uint8_t *data, uint16_t data_len)
{
    uint8_t crc = 0;
    while (data_len--) {
        crc = kM1CrcTable[(crc ^ *data) & 0xff];
        data++;
    }
    return crc;
}

uint32_t POELidarDriverImpl::get_ms_()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return ms.count();
}


}} //namespace cpj::poelidar
