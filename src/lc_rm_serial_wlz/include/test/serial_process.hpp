#ifndef SERIAL_PROCESS_HPP
#define SERIAL_PROCESS_HPP

#include <cstdint>
#include <cstddef>

#include "lc_rm_serial_wlz/include/test/protocol.hpp"
#include "lc_rm_serial_wlz/include/test/crc_check.hpp"

namespace lc_serial
{
    //字节读写
    void write_float(uint8_t* buf, float val);
    float read_float(const uint8_t* buf);

    //打包/解包
    void pack_frame(uint16_t cmd_id, const TxData& data, uint8_t* tx_buf, uint16_t* tx_len);

    bool unpack_frame(const uint8_t* rx_buf, size_t buf_len, RxFrame& out_frame);
}

#endif