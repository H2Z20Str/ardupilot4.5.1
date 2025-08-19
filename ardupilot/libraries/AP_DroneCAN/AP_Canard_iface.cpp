#include "AP_Canard_iface.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>
#if HAL_ENABLE_DRONECAN_DRIVERS
#include <canard/handler_list.h>
#include <canard/transfer_object.h>
#include <AP_Math/AP_Math.h>
#include <dronecan_msgs.h>
extern const AP_HAL::HAL& hal;
#define LOG_TAG "DroneCANIface"
#include <canard.h>
#include <AP_CANManager/AP_CANSensor.h>
#include <GCS_MAVLink/GCS.h>

#define DEBUG_PKTS 0

#define CANARD_MSG_TYPE_FROM_ID(x)                         ((uint16_t)(((x) >> 8U)  & 0xFFFFU))

DEFINE_HANDLER_LIST_HEADS();
DEFINE_HANDLER_LIST_SEMAPHORES();

DEFINE_TRANSFER_OBJECT_HEADS();
DEFINE_TRANSFER_OBJECT_SEMAPHORES();

#if AP_TEST_DRONECAN_DRIVERS
CanardInterface* CanardInterface::canard_ifaces[] = {nullptr, nullptr, nullptr};
CanardInterface CanardInterface::test_iface{2};
uint8_t test_node_mem_area[1024];
HAL_Semaphore test_iface_sem;
#endif

void canard_allocate_sem_take(CanardPoolAllocator *allocator) {
    if (allocator->semaphore == nullptr) {
        allocator->semaphore = new HAL_Semaphore;
        if (allocator->semaphore == nullptr) {
            // out of memory
            CANARD_ASSERT(0);
            return;
        }
    }
    ((HAL_Semaphore*)allocator->semaphore)->take_blocking();
}

void canard_allocate_sem_give(CanardPoolAllocator *allocator) {
    if (allocator->semaphore == nullptr) {
        // it should have been allocated by canard_allocate_sem_take
        CANARD_ASSERT(0);
        return;
    }
    ((HAL_Semaphore*)allocator->semaphore)->give();
}

CanardInterface::CanardInterface(uint8_t iface_index) :
Interface(iface_index) {
#if AP_TEST_DRONECAN_DRIVERS
    if (iface_index < 3) {
        canard_ifaces[iface_index] = this;
    }
    if (iface_index == 0) {
        test_iface.init(test_node_mem_area, sizeof(test_node_mem_area), 125);
    }
    canardInitTxTransfer(&tx_transfer);
#endif
}

void CanardInterface::init(void* mem_arena, size_t mem_arena_size, uint8_t node_id) {
    canardInit(&canard, mem_arena, mem_arena_size, onTransferReception, shouldAcceptTransfer, this);
    canardSetLocalNodeID(&canard, node_id);
    initialized = true;
}

bool CanardInterface::broadcast(const Canard::Transfer &bcast_transfer) {
    if (!initialized) {
        return false;
    }
    WITH_SEMAPHORE(_sem_tx);

#if AP_TEST_DRONECAN_DRIVERS
    if (this == &test_iface) {
        test_iface_sem.take_blocking();
    }
#endif

    tx_transfer = {
        .transfer_type = bcast_transfer.transfer_type,
        .data_type_signature = bcast_transfer.data_type_signature,
        .data_type_id = bcast_transfer.data_type_id,
        .inout_transfer_id = bcast_transfer.inout_transfer_id,
        .priority = bcast_transfer.priority,
        .payload = (const uint8_t*)bcast_transfer.payload,
        .payload_len = uint16_t(bcast_transfer.payload_len),
#if CANARD_ENABLE_CANFD
        .canfd = bcast_transfer.canfd,
#endif
        .deadline_usec = AP_HAL::micros64() + (bcast_transfer.timeout_ms * 1000),
#if CANARD_MULTI_IFACE
        .iface_mask = uint8_t((1<<num_ifaces) - 1),
#endif
    };
    // do canard broadcast
    int16_t ret = canardBroadcastObj(&canard, &tx_transfer);
#if AP_TEST_DRONECAN_DRIVERS
    if (this == &test_iface) {
        test_iface_sem.give();
    }
#endif
    if (ret <= 0) {
        protocol_stats.tx_errors++;
    } else {
        protocol_stats.tx_frames += ret;
    }
    return ret > 0;
}

bool CanardInterface::request(uint8_t destination_node_id, const Canard::Transfer &req_transfer) {
    if (!initialized) {
        return false;
    }
    WITH_SEMAPHORE(_sem_tx);

    tx_transfer = {
        .transfer_type = req_transfer.transfer_type,
        .data_type_signature = req_transfer.data_type_signature,
        .data_type_id = req_transfer.data_type_id,
        .inout_transfer_id = req_transfer.inout_transfer_id,
        .priority = req_transfer.priority,
        .payload = (const uint8_t*)req_transfer.payload,
        .payload_len = uint16_t(req_transfer.payload_len),
#if CANARD_ENABLE_CANFD
        .canfd = req_transfer.canfd,
#endif
        .deadline_usec = AP_HAL::micros64() + (req_transfer.timeout_ms * 1000),
#if CANARD_MULTI_IFACE
        .iface_mask = uint8_t((1<<num_ifaces) - 1),
#endif
    };
    // do canard request
    int16_t ret = canardRequestOrRespondObj(&canard, destination_node_id, &tx_transfer);
    if (ret <= 0) {
        protocol_stats.tx_errors++;
    } else {
        protocol_stats.tx_frames += ret;
    }
    return ret > 0;
}

bool CanardInterface::respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer) {
    if (!initialized) {
        return false;
    }
    WITH_SEMAPHORE(_sem_tx);

    tx_transfer = {
        .transfer_type = res_transfer.transfer_type,
        .data_type_signature = res_transfer.data_type_signature,
        .data_type_id = res_transfer.data_type_id,
        .inout_transfer_id = res_transfer.inout_transfer_id,
        .priority = res_transfer.priority,
        .payload = (const uint8_t*)res_transfer.payload,
        .payload_len = uint16_t(res_transfer.payload_len),
#if CANARD_ENABLE_CANFD
        .canfd = res_transfer.canfd,
#endif
        .deadline_usec = AP_HAL::micros64() + (res_transfer.timeout_ms * 1000),
#if CANARD_MULTI_IFACE
        .iface_mask = uint8_t((1<<num_ifaces) - 1),
#endif
    };
    // do canard respond
    int16_t ret = canardRequestOrRespondObj(&canard, destination_node_id, &tx_transfer);
    if (ret <= 0) {
        protocol_stats.tx_errors++;
    } else {
        protocol_stats.tx_frames += ret;
    }
    return ret > 0;
}

void CanardInterface::onTransferReception(CanardInstance* ins, CanardRxTransfer* transfer) {
    CanardInterface* iface = (CanardInterface*) ins->user_reference;
    iface->handle_message(*transfer);
}

bool CanardInterface::shouldAcceptTransfer(const CanardInstance* ins,
                                           uint64_t* out_data_type_signature,
                                           uint16_t data_type_id,
                                           CanardTransferType transfer_type,
                                           uint8_t source_node_id) {
    CanardInterface* iface = (CanardInterface*) ins->user_reference;
    return iface->accept_message(data_type_id, *out_data_type_signature);
}

#if AP_TEST_DRONECAN_DRIVERS
void CanardInterface::processTestRx() {
    if (!test_iface.initialized) {
        return;
    }
    WITH_SEMAPHORE(test_iface_sem);
    for (const CanardCANFrame* txf = canardPeekTxQueue(&test_iface.canard); txf != NULL; txf = canardPeekTxQueue(&test_iface.canard)) {
        if (canard_ifaces[0]) {
            canardHandleRxFrame(&canard_ifaces[0]->canard, txf, AP_HAL::micros64());   
        }
        canardPopTxQueue(&test_iface.canard);
    }
}
#endif

void CanardInterface::processTx(bool raw_commands_only = false) {
    WITH_SEMAPHORE(_sem_tx);

    for (uint8_t iface = 0; iface < num_ifaces; iface++) {
        if (ifaces[iface] == NULL) {
            continue;
        }
        auto txq = canard.tx_queue;
        if (txq == nullptr) {
            return;
        }
        // volatile as the value can change at any time during can interrupt
        // we need to ensure that this is not optimized
        volatile const auto *stats = ifaces[iface]->get_statistics();
        uint64_t last_transmit_us = stats==nullptr?0:stats->last_transmit_us;
        bool iface_down = true;
        if (stats == nullptr || (AP_HAL::micros64() - last_transmit_us) < 200000UL) {
            /*
            We were not able to queue the frame for
            sending. Only mark the send as failing if the
            interface is active. We consider an interface as
            active if it has had successful transmits for some time.
            */
            iface_down = false;
        } 
        // scan through list of pending transfers
        while (true) {
            auto txf = &txq->frame;
            if (raw_commands_only &&
                CANARD_MSG_TYPE_FROM_ID(txf->id) != UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID &&
                CANARD_MSG_TYPE_FROM_ID(txf->id) != COM_HOBBYWING_ESC_RAWCOMMAND_ID) {
                // look at next transfer
                txq = txq->next;
                if (txq == nullptr) {
                    break;
                }
                continue;
            }
            AP_HAL::CANFrame txmsg {};
            txmsg.dlc = AP_HAL::CANFrame::dataLengthToDlc(txf->data_len);
            memcpy(txmsg.data, txf->data, txf->data_len);
            txmsg.id = (txf->id | AP_HAL::CANFrame::FlagEFF);
#if HAL_CANFD_SUPPORTED
            txmsg.canfd = txf->canfd;
#endif
            bool write = true;
            bool read = false;
            ifaces[iface]->select(read, write, &txmsg, 0);
            if (!write) {
                // if there is no space then we need to start from the
                // top of the queue, so wait for the next loop
                if (!iface_down) {
                    break;
                } else {
                    txf->iface_mask &= ~(1U<<iface);
                }
            } else if ((txf->iface_mask & (1U<<iface)) && (AP_HAL::micros64() < txf->deadline_usec)) {
                // try sending to interfaces, clearing the mask if we succeed
                if (ifaces[iface]->send(txmsg, txf->deadline_usec, 0) > 0) {
                    txf->iface_mask &= ~(1U<<iface);
                } else {
                    // if we fail to send then we try sending on next interface
                    if (!iface_down) {
                        break;
                    } else {
                        txf->iface_mask &= ~(1U<<iface);
                    }
                }
            }
            // look at next transfer
            txq = txq->next;
            if (txq == nullptr) {
                break;
            }
        }
    }

}

void CanardInterface::update_rx_protocol_stats(int16_t res)
{
    switch (res) {
    case CANARD_OK:
        protocol_stats.rx_frames++;
        break;
    case -CANARD_ERROR_OUT_OF_MEMORY:
        protocol_stats.rx_error_oom++;
        break;
    case -CANARD_ERROR_INTERNAL:
        protocol_stats.rx_error_internal++;
        break;
    case -CANARD_ERROR_RX_INCOMPATIBLE_PACKET:
        protocol_stats.rx_ignored_not_wanted++;
        break;
    case -CANARD_ERROR_RX_WRONG_ADDRESS:
        protocol_stats.rx_ignored_wrong_address++;
        break;
    case -CANARD_ERROR_RX_NOT_WANTED:
        protocol_stats.rx_ignored_not_wanted++;
        break;
    case -CANARD_ERROR_RX_MISSED_START:
        protocol_stats.rx_error_missed_start++;
        break;
    case -CANARD_ERROR_RX_WRONG_TOGGLE:
        protocol_stats.rx_error_wrong_toggle++;
        break;
    case -CANARD_ERROR_RX_UNEXPECTED_TID:
        protocol_stats.rx_ignored_unexpected_tid++;
        break;
    case -CANARD_ERROR_RX_SHORT_FRAME:
        protocol_stats.rx_error_short_frame++;
        break;
    case -CANARD_ERROR_RX_BAD_CRC:
        protocol_stats.rx_error_bad_crc++;
        break;
    default:
        // mark all other errors as internal
        protocol_stats.rx_error_internal++;
        break;
    }
}


int gMR72code=0;
int gMR72Dist[10];
int MZBDist[8];

typedef struct { // byte description
uint8_t Objects_ID:8; //目标ID
float Objects_DistLong; //目标纵向距离
float Objects_DistLat; //目标横向距离
float Objects_VrelLong; // 目标纵向速度
float Objects_VrelLat; //目标横向速度
uint8_t Objects_DynProp:8; // 目标运动属性
uint8_t Object_Class:8; //目标分类或扇区编号 有的版本无此值
uint8_t Objects_RCS:8; //目标RCS默认是0
}object_dataBytes;

object_dataBytes mzb;

void CanardInterface::processRx() {
    AP_HAL::CANFrame rxmsg;
    for (uint8_t i=0; i<num_ifaces; i++) {
        while(true) {
            if (ifaces[i] == NULL) {
                break;
            }
            bool read_select = true;
            bool write_select = false;
            ifaces[i]->select(read_select, write_select, nullptr, 0);
            if (!read_select) { // No data pending 无待处理数据
                break;
            }
            CanardCANFrame rx_frame {};

            //palToggleLine(HAL_GPIO_PIN_LED);
            uint64_t timestamp;
            AP_HAL::CANIface::CanIOFlags flags;
            if (ifaces[i]->receive(rxmsg, timestamp, flags) <= 0) {
                break;
            }
            if (!rxmsg.isExtended()) {
                // 11 bit frame, see if we have a handler
                if (aux_11bit_driver != nullptr) {
                    aux_11bit_driver->handle_frame(rxmsg);
                }
               // continue;
            }

            rx_frame.data_len = AP_HAL::CANFrame::dlcToDataLength(rxmsg.dlc);
            memcpy(rx_frame.data, rxmsg.data, rx_frame.data_len);
#if HAL_CANFD_SUPPORTED
            rx_frame.canfd = rxmsg.canfd;
#endif
            rx_frame.id = rxmsg.id;
#if CANARD_MULTI_IFACE
            rx_frame.iface_id = i;
#endif
            // for(int8_t ij=0;ij<8;ij++)
            //     gcs().send_text(MAV_SEVERITY_CRITICAL, "data[%d]=%x",ij,rx_frame.data[ij]);

    
    const uint8_t source_node_id =(((rx_frame.id) >> 0U)  & 0x7FU);

    //CAN解析 2024.08.28 hzz
    {
        //MR72
        static uint8_t MR72PI = 0;
        if(source_node_id==0x20)
        {
            if(rx_frame.data[0]==0x01&&MR72PI==0)
                MR72PI=1;
        }

        if(MR72PI!=0)
        {
            if(source_node_id==0x20)
            {
                switch(MR72PI)
                {
                    case 1:
                        gMR72code = ((int)rx_frame.data[3]<<8) + rx_frame.data[2];
                        gMR72Dist[0] = ((int)rx_frame.data[5]<<8) + rx_frame.data[4];
                        gMR72Dist[1] = rx_frame.data[6];
                        break;
                    case 2:
                        gMR72Dist[1] += ((int)rx_frame.data[0]<<8);
                        gMR72Dist[2] = ((int)rx_frame.data[2]<<8) + rx_frame.data[1];
                        gMR72Dist[3] = ((int)rx_frame.data[4]<<8) + rx_frame.data[3];
                        gMR72Dist[4] = ((int)rx_frame.data[6]<<8) + rx_frame.data[5];
                        break;
                    case 3:
                        gMR72Dist[5] = ((int)rx_frame.data[1]<<8) + rx_frame.data[0];
                        gMR72Dist[6] = ((int)rx_frame.data[3]<<8) + rx_frame.data[2];
                        gMR72Dist[7] = ((int)rx_frame.data[5]<<8) + rx_frame.data[4];
                        break;
                    default:
                        break;
                }
                MR72PI++;
            }
            if(MR72PI>=4)
            {
                MR72PI=0;
                hal.util->radar_type=1;
               if(hal.util->hzz_test[0]==1)  //输出实验
               {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "gMR72code %d",gMR72code);
                gcs().send_text(MAV_SEVERITY_CRITICAL, "gMR72Dist[0] %d",gMR72Dist[0]);
                gcs().send_text(MAV_SEVERITY_CRITICAL, "gMR72Dist[1] %d",gMR72Dist[1]);
                gcs().send_text(MAV_SEVERITY_CRITICAL, "gMR72Dist[2] %d",gMR72Dist[2]);
                gcs().send_text(MAV_SEVERITY_CRITICAL, "gMR72Dist[3] %d",gMR72Dist[3]);
                gcs().send_text(MAV_SEVERITY_CRITICAL, "gMR72Dist[4] %d",gMR72Dist[4]);
                gcs().send_text(MAV_SEVERITY_CRITICAL, "gMR72Dist[5] %d",gMR72Dist[5]);
                gcs().send_text(MAV_SEVERITY_CRITICAL, "gMR72Dist[6] %d",gMR72Dist[6]);
                gcs().send_text(MAV_SEVERITY_CRITICAL, "gMR72Dist[7] %d",gMR72Dist[7]);
                }
                //数据传输
                for(int b=0;b<8;b++)
                {
                    hal.util->MR72_can[b]=(gMR72Dist[i] >= 8000)?0:gMR72Dist[b]*10;//最远距离为80m，这里的单位为cm

                }
                

            }
        }

        //mo zhi bi 
        if(rx_frame.id==0x60B)
        {
            hal.util->radar_type=2;
        //      目标 ID：
          mzb.Objects_ID=rx_frame.data[0];
       
        //      目标纵向距离：
            mzb.Objects_DistLong=(rx_frame.data[1]*32 + (rx_frame.data[2]>>3))*0.2-500;
        
        //      目标横向距离：
            mzb.Objects_DistLat=((rx_frame.data[2] &0x07)*256 + rx_frame.data[3]) *0.2-204.6;
        
        //      目标纵向速度：
          mzb.Objects_VrelLong=(rx_frame.data[4]*4+(rx_frame.data[5]>>6))*0.25-128;
        
        //      目标横向速度：
            mzb.Objects_VrelLat=((rx_frame.data[5]&0x3F)*8+(rx_frame.data[6]>>5))*0.25-64;
       
        //      目标动态属性：
            mzb.Objects_DynProp=rx_frame.data[6]&0x07;
        
        //      RCS：
            mzb.Objects_RCS=rx_frame.data[7]*0.5-64;
        

         if(abs(mzb.Objects_DistLat)<hal.util->mzb_width) //仅取宽度以内的障碍物
            {
                hal.util->mzb_DistLong=mzb.Objects_DistLong;
                if(hal.util->hzz_test[0]==4)  //输出实验
                   gcs().send_text(MAV_SEVERITY_CRITICAL, "i%d,y%.2fm,x%.2f m",mzb.Objects_ID,mzb.Objects_DistLong,mzb.Objects_DistLat);
            } 

         int s=(int)(mzb.Objects_DistLat/hal.util->mzb_width);

         switch(s) //根据设定的宽度分配8个方向的避障数据
         {
             case 0:MZBDist[0]=mzb.Objects_DistLong;break; //-0.6-0.6
             case 1:
             case 2:
             case 3:MZBDist[1]=mzb.Objects_DistLong;break; //0.6--1.8
             case 4:
             case 5:MZBDist[2]=mzb.Objects_DistLong;break; //1.8--3.0
             case 6:
             case 7:MZBDist[3]=mzb.Objects_DistLong;break; //3.0-4.2
             case -1:
             case -2:
             case -3:MZBDist[7]=mzb.Objects_DistLong;break;
             case -4:
             case -5:MZBDist[6]=mzb.Objects_DistLong;break;
             case -6:
             case -7:MZBDist[5]=mzb.Objects_DistLong;break;

             default:MZBDist[4]=mzb.Objects_DistLong;break;
         }

         if(mzb.Objects_DistLat>hal.util->mzb_width)
             MZBDist[1]=mzb.Objects_DistLong;
         else if(mzb.Objects_DistLat<(0-hal.util->mzb_width))
             MZBDist[7]=mzb.Objects_DistLong;
         else MZBDist[0]=mzb.Objects_DistLong;

        // MZBDist[1]=(mzb.Objects_DistLat>hal.util->mzb_width)?mzb.Objects_DistLong:0;

          if(hal.util->hzz_test[0]==2)  //输出实验
          {
               gcs().send_text(MAV_SEVERITY_CRITICAL, "目标 ID:%d    ",mzb.Objects_ID);
               gcs().send_text(MAV_SEVERITY_CRITICAL, "Y:%.2f m   ",mzb.Objects_DistLong);
               gcs().send_text(MAV_SEVERITY_CRITICAL, "X:%.2f m   ",mzb.Objects_DistLat);
               gcs().send_text(MAV_SEVERITY_CRITICAL, "YV:%.2f m/s     ",mzb.Objects_VrelLong);
               gcs().send_text(MAV_SEVERITY_CRITICAL, "XV:%.2f m/s ",mzb.Objects_VrelLat);
               gcs().send_text(MAV_SEVERITY_CRITICAL, "D:%d   ",mzb.Objects_DynProp);
               gcs().send_text(MAV_SEVERITY_CRITICAL, "RCS:%d\r\n",mzb.Objects_RCS);
          }
          else if(hal.util->hzz_test[0]==3)  //输出实验
          {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "i%d,y%.2fm,x%.2f m",mzb.Objects_ID,mzb.Objects_DistLong,mzb.Objects_DistLat);
          }
          
        }
    }
   

            {
                WITH_SEMAPHORE(_sem_rx);

                const int16_t res = canardHandleRxFrame(&canard, &rx_frame, timestamp);
                if (res == -CANARD_ERROR_RX_MISSED_START) {
                    // this might remaining frames from a message that we don't accept, so check 这可能是我们不接受的消息中的剩余帧，因此请检查
                    uint64_t dummy_signature;
                    if (shouldAcceptTransfer(&canard,
                                        &dummy_signature,
                                        extractDataType(rx_frame.id),
                                        extractTransferType(rx_frame.id),
                                        1)) { // doesn't matter what we pass here 我们在这里经过什么并不重要
                        update_rx_protocol_stats(res);
                    } else {
                        protocol_stats.rx_ignored_not_wanted++;
                    }
                } else {
                    update_rx_protocol_stats(res);
                }
            }
        }
    }
}

void CanardInterface::process(uint32_t duration_ms) {
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "22222");//tong
#if AP_TEST_DRONECAN_DRIVERS
    const uint64_t deadline = AP_HAL::micros64() + duration_ms*1000;
    while (AP_HAL::micros64() < deadline) {
        processTestRx();
        hal.scheduler->delay_microseconds(1000);
    }
#else
    const uint64_t deadline = AP_HAL::micros64() + duration_ms*1000;
    while (true) {
        processRx();//接收函数
        processTx(); //发送函数
        {
            WITH_SEMAPHORE(_sem_rx);
            WITH_SEMAPHORE(_sem_tx);
            canardCleanupStaleTransfers(&canard, AP_HAL::micros64());
        }
        const uint64_t now = AP_HAL::micros64();
        if (now < deadline) {
            IGNORE_RETURN(sem_handle.wait(deadline - now));
        } else {
            break;
        }
    }
#endif
}

bool CanardInterface::add_interface(AP_HAL::CANIface *can_iface)
{
    if (num_ifaces > HAL_NUM_CAN_IFACES) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "DroneCANIfaceMgr: Num Ifaces Exceeded\n");
        return false;
    }
    if (can_iface == nullptr) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "DroneCANIfaceMgr: Iface Null\n");
        return false;
    }
    if (ifaces[num_ifaces] != nullptr) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "DroneCANIfaceMgr: Iface already added\n");
        return false;
    }
    ifaces[num_ifaces] = can_iface;
    if (ifaces[num_ifaces] == nullptr) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "DroneCANIfaceMgr: Can't alloc uavcan::iface\n");
        return false;
    }
    if (!can_iface->set_event_handle(&sem_handle)) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "DroneCANIfaceMgr: Setting event handle failed\n");
        return false;
    }
    AP::can().log_text(AP_CANManager::LOG_INFO, LOG_TAG, "DroneCANIfaceMgr: Successfully added interface %d\n", int(num_ifaces));
    num_ifaces++;
    return true;
}

// add an 11 bit auxillary driver
bool CanardInterface::add_11bit_driver(CANSensor *sensor)
{
    if (aux_11bit_driver != nullptr) {
        // only allow one
        return false;
    }
    aux_11bit_driver = sensor;
    return true;
}

// handler for outgoing frames for auxillary drivers
bool CanardInterface::write_aux_frame(AP_HAL::CANFrame &out_frame, const uint64_t timeout_us)
{
    bool ret = false;
    for (uint8_t iface = 0; iface < num_ifaces; iface++) {
        if (ifaces[iface] == NULL) {
            continue;
        }
        ret |= ifaces[iface]->send(out_frame, timeout_us, 0) > 0;
    }
    return ret;
}

#endif // #if HAL_ENABLE_DRONECAN_DRIVERS
