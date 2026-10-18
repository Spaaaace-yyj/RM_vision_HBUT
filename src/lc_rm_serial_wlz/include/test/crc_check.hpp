#ifndef LC_SERIAL_TEST_CRC_CHECK_HPP
#define LC_SERIAL_TEST_CRC_CHECK_HPP

#include <cstdint>
#include <cstddef>

namespace lc_serial
{
    //crc参数
    // CRC8：多项式 0x1D，初始值 0xFF，用于帧头校验（前 4 字节）
    constexpr uint8_t CRC8_INIT = 0xFF;

   //crc16:多项式初始值0xFFFF(整包校验)
   constexpr uint16_t CRC16_INIT = 0xFFFF; 

    //crc接口
    /**
 * @brief 计算 CRC8 值
 * @param data 输入字节流（非空）
 * @param len  字节数
 * @return CRC8 校验值
 */

    uint8_t crc_8(const uint8_t* data,size_t len);
    /**
 * @brief 增量更新 CRC8（用于流式计算）
 * @param crc 上一次的 CRC8 值
 * @param val 新字节
 * @return 新的 CRC8 值
 */

    uint8_t update_crc_8(uint8_t crc, uint8_t val);
    /**
 * @brief 校验 CRC8
 * @param data 包含 CRC8 的完整数据，最后一字节是 CRC8
 * @param len  数据总长度（含 CRC8 字节）
 * @return true 校验通过
 *
 * 校验范围：data[0 .. len-2]
 * 期望值：  data[len-1]
 */

    bool crc_8_check(const uint8_t* data, size_t len);

    //crc16接口

    /**
 * @brief 计算 CRC16（Modbus 参数，初始值 0xFFFF）
 * @param data 输入字节流（非空）
 * @param len  字节数
 * @return CRC16 校验值
 */

    uint16_t crc_16(const uint8_t* data,size_t len);

    /**
 * @brief 增量更新 CRC16
 * @param crc 上一次的 CRC16 值
 * @param c   新字节
 * @return 新的 CRC16 值
 */

    uint16_t update_crc_16(uint16_t crc,uint8_t c);

    /**
 * @brief 校验 CRC16（小端存放）
 * @param data 包含 CRC16 的完整数据，最后两字节是 CRC16
 * @param len  数据总长度（含 CRC16 两字节）
 * @return true 校验通过
 *
 * 校验范围：data[0 .. len-3]
 * 期望值：  data[len-2] | (data[len-1] << 8)   // 小端
 */

    bool crc_16_check(const uint8_t* data, size_t len);

}



#endif