#include "lc_rm_serial_wlz/include/test/serial_process.hpp"
#include <cstring>

namespace lc_serial
{
    void write_float(uint8_t* buf, float val)
    {
        uint32_t bits;
        std::memcpy(&bits, &val, sizeof(float));
        buf[0] = bits & 0xFF;
        buf[1] = (bits >> 8) & 0xFF;
        buf[2] = (bits >> 16) & 0xFF;
        buf[3] = (bits >> 24) & 0xFF;
    }

    float read_float(const uint8_t* buf)
    {
        uint32_t bits = static_cast<uint32_t>(buf[0])
            | (static_cast<uint32_t>(buf[1]) << 8)
            | (static_cast<uint32_t>(buf[2]) << 16)
            | (static_cast<uint32_t>(buf[3]) << 24);        
        float val;
        std::memcpy(&val, &bits, sizeof(float));
        return val;
    }

    void pack_frame(uint16_t cmd_id, const TxData& data, uint8_t* tx_buf, uint16_t* tx_len)
    {
        //帧头
        tx_buf[0] = PROTOCOL_SOF;
        tx_buf[1] = TX_DATA_LEN & 0xFF;
        tx_buf[2] = (TX_DATA_LEN >> 8) & 0xFF;
        tx_buf[3] = 0x00;//seq
        tx_buf[4] = crc_8(&tx_buf[0], 4);//crc8

        //命令码
        tx_buf[5] = cmd_id & 0xFF;
        tx_buf[6] = (cmd_id >> 8) & 0xFF;

        //数据段
        write_float(&tx_buf[7], data.forward);
        write_float(&tx_buf[11], data.turn);
        write_float(&tx_buf[15], data.rotate);
        write_float(&tx_buf[19], data.yaw);
        write_float(&tx_buf[23], data.pitch);

        //crc16
        uint16_t crc16 = crc_16(&tx_buf[0],7+TX_DATA_LEN);
        tx_buf[27] = crc16 & 0xFF;
        tx_buf[28] = (crc16 >> 8) & 0xFF;

        *tx_len = TX_FRAME_LEN;

    }

    bool unpack_frame(const uint8_t* rx_buf, size_t buf_len, RxFrame& out_frame)
    {
        if(buf_len < RX_FRAME_LEN) return false;
        if(rx_buf[0] != PROTOCOL_SOF) return false;
        if(!crc_8_check(&rx_buf[0], 5)) return false;

        uint16_t data_len = (rx_buf[2] << 8) | rx_buf[1];
        if(data_len != RX_DATA_LEN) return false;
        if(!crc_16_check(&rx_buf[0], RX_FRAME_LEN)) return false;

        out_frame.seq = rx_buf[3];
        out_frame.cmd_id = (rx_buf[6] << 8) | rx_buf[5];
        out_frame.data.yaw = read_float(&rx_buf[7]);
        out_frame.data.pitch = read_float(&rx_buf[11]);
        out_frame.data.free_data = read_float(&rx_buf[15]);
        return true;
        

        
    }
}