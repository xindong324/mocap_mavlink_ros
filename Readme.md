# mavlink 串口发送信息
serial_mocap ros获取mocap数据， c++自带串口收发  

serial_nrf： ros获取mocap数据，ros串口收发  

## 对mavlink库的修改：
mavlink版本1.0   
常规mavlink库函数用于嵌入式，在进行mavlink电脑端移植时需要修改的部分：
1. mavlink_types.h: 电脑发送端需要有这部分代码（5-6行左右），嵌入式端这部分代码需要注释掉（如果不注释报错的话）
```
#ifdef __GNUC__
  #define MAVPACKED( __Declaration__ ) __Declaration__ __attribute__((packed))
#else
  #define MAVPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif
```
2. mavlink_helpers.h:   
修改两部分，第一部分是修改 mavlink_finalize_message_chan函数中最后关于crc校验赋值部分
```
#if MAVLINK_CRC_EXTRA
MAVLINK_HELPER uint16_t mavlink_finalize_message_chan(mavlink_message_t *msg, uint8_t system_id, uint8_t component_id,
                                                      uint8_t chan, uint8_t length, uint8_t crc_extra)
#else
MAVLINK_HELPER uint16_t mavlink_finalize_message_chan(mavlink_message_t *msg, uint8_t system_id, uint8_t component_id,
                                                      uint8_t chan, uint8_t length)
#endif
{
    // This code part is the same for all messages;
    uint16_t checksum;

    msg->magic  = MAVLINK_STX;
    msg->len    = length;
    msg->sysid  = system_id;
    msg->compid = component_id;
    // One sequence number per component
    msg->seq    = mavlink_get_channel_status(chan)->current_tx_seq;
    mavlink_get_channel_status(chan)->current_tx_seq = mavlink_get_channel_status(chan)->current_tx_seq + 1;
    checksum    = crc_calculate((uint8_t *)&msg->len, length + MAVLINK_CORE_HEADER_LEN);
#if MAVLINK_CRC_EXTRA
    crc_accumulate(crc_extra, &checksum);
#endif
    msg->checksum = checksum;
    //mavlink_ck_a(msg) = (uint8_t)(checksum & 0xFF);
    //mavlink_ck_b(msg) = (uint8_t)(checksum >> 8);

    return length + MAVLINK_NUM_NON_PAYLOAD_BYTES;
}
```

第二部分是重新实现下 MAVLINK_HELPER uint16_t mavlink_msg_to_send_buffer(uint8_t *buf, const mavlink_message_t *msg)函数：  
```
MAVLINK_HELPER uint16_t mavlink_msg_to_send_buffer(uint8_t *buf, const mavlink_message_t *msg)
{
	uint8_t signature_len, header_len;
	uint8_t *ck;
    uint8_t length = msg->len;
        
    signature_len = 0;
    header_len = MAVLINK_CORE_HEADER_LEN;
    buf[0] = msg->magic;
    buf[1] = length;
    buf[2] = msg->seq;
    buf[3] = msg->sysid;
    buf[4] = msg->compid;
    buf[5] = msg->msgid & 0xFF;
    memcpy(&buf[6], _MAV_PAYLOAD(msg), msg->len);
    ck = buf + header_len + 1 + (uint16_t)msg->len;
	
	ck[0] = (uint8_t)(msg->checksum & 0xFF);
	ck[1] = (uint8_t)(msg->checksum >> 8);

	return header_len + 1 + 2 + (uint16_t)length + (uint16_t)signature_len;
}
```

