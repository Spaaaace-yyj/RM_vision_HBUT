#include <cstdio>
#include <cstring>
#include "lc_rm_serial_wlz/include/test/serial_process.hpp"
#include "lc_rm_serial_wlz/include/test/crc_check.hpp"

using namespace lc_serial;

// 打印字节流
static void dump(const char* tag, const uint8_t* buf, size_t len)
{
    printf("%s (%zu bytes): ", tag, len);
    for (size_t i = 0; i < len; ++i) {
        printf("%02X ", buf[i]);
    }
    printf("\n");
}

int main()
{
    // ================== 测试 1：打包 ==================
    TxData tx = {1.5f, -2.0f, 0.3f, 45.0f, 30.0f};
    uint8_t tx_buf[TX_FRAME_LEN] = {0};
    uint16_t tx_len = 0;
    pack_frame(0x0112, tx, tx_buf, &tx_len);
    dump("TX", tx_buf, tx_len);

    // 检查固定字段
    if (tx_buf[0] != PROTOCOL_SOF)  printf("FAIL: SOF\n");
    if (tx_buf[1] != 20)            printf("FAIL: len_lo\n");
    if (tx_buf[2] != 0x00)          printf("FAIL: len_hi\n");
    if (tx_buf[3] != 0x00)          printf("FAIL: seq\n");
    if (tx_buf[4] != crc_8(tx_buf, 4)) printf("FAIL: CRC8\n");
    if (tx_buf[5] != 0x12)          printf("FAIL: cmd_id_lo\n");
    if (tx_buf[6] != 0x01)          printf("FAIL: cmd_id_hi\n");
    printf("TX OK\n\n");

    // ================== 测试 2：解包（手工构造下位机回复帧） ==================
    uint8_t rx_buf[RX_FRAME_LEN] = {0};
    rx_buf[0] = PROTOCOL_SOF;
    rx_buf[1] = RX_DATA_LEN & 0xFF;
    rx_buf[2] = (RX_DATA_LEN >> 8) & 0xFF;
    rx_buf[3] = 0x00;                        // seq
    rx_buf[4] = crc_8(rx_buf, 4);            // CRC8
    rx_buf[5] = 0x00;                        // cmd_id lo
    rx_buf[6] = 0x00;                        // cmd_id hi
    write_float(&rx_buf[7],  10.0f);         // yaw
    write_float(&rx_buf[11], 20.0f);         // pitch
    write_float(&rx_buf[15], 0.0f);          // free_data
    uint16_t c = crc_16(rx_buf, 7 + RX_DATA_LEN);
    rx_buf[7 + RX_DATA_LEN]     = c & 0xFF;
    rx_buf[7 + RX_DATA_LEN + 1] = (c >> 8) & 0xFF;
    dump("RX", rx_buf, RX_FRAME_LEN);

    RxFrame frame;
    if (unpack_frame(rx_buf, RX_FRAME_LEN, frame)) {
        printf("Parse OK: seq=%u cmd_id=0x%04X yaw=%.2f pitch=%.2f free=%.2f\n",
               frame.seq, frame.cmd_id,
               frame.data.yaw, frame.data.pitch, frame.data.free_data);
    } else {
        printf("Parse FAILED!\n");
    }
    printf("\n");

    // ================== 测试 3：异常情况 ==================
    // 3.1 帧头错误
    uint8_t bad1[RX_FRAME_LEN];
    std::memcpy(bad1, rx_buf, RX_FRAME_LEN);
    bad1[0] = 0x22;
    printf("bad SOF    → %s\n", unpack_frame(bad1, RX_FRAME_LEN, frame) ? "SHOULD FAIL" : "OK (rejected)");

    // 3.2 CRC8 错误
    uint8_t bad2[RX_FRAME_LEN];
    std::memcpy(bad2, rx_buf, RX_FRAME_LEN);
    bad2[4] ^= 0xFF;
    printf("bad CRC8   → %s\n", unpack_frame(bad2, RX_FRAME_LEN, frame) ? "SHOULD FAIL" : "OK (rejected)");

    // 3.3 CRC16 错误
    uint8_t bad3[RX_FRAME_LEN];
    std::memcpy(bad3, rx_buf, RX_FRAME_LEN);
    bad3[20] ^= 0xFF;
    printf("bad CRC16  → %s\n", unpack_frame(bad3, RX_FRAME_LEN, frame) ? "SHOULD FAIL" : "OK (rejected)");

    // 3.4 长度不足
    printf("too short  → %s\n", unpack_frame(rx_buf, 10, frame) ? "SHOULD FAIL" : "OK (rejected)");

    // 3.5 数据长度字段错误
    uint8_t bad5[RX_FRAME_LEN];
    std::memcpy(bad5, rx_buf, RX_FRAME_LEN);
    bad5[1] = 0x10;   // 声称 16 字节，不是 12
    bad5[4] = crc_8(bad5, 4);
    printf("bad len    → %s\n", unpack_frame(bad5, RX_FRAME_LEN, frame) ? "SHOULD FAIL" : "OK (rejected)");

    return 0;
}