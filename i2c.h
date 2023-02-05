/**
 * @file      : i2c.h
 * @brief     : Linux平台I2C驱动头文件
 * @author    : huenrong (huenrong1028@outlook.com)
 * @date      : 2023-01-18 13:15:51
 *
 * @copyright : Copyright (c) 2023 huenrong
 *
 * @history   : date       author          description
 *              2023-01-18 huenrong        创建文件
 *
 */

#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  打开I2C设备
 * @param  i2c_dev_name: 输入参数, I2C设备名(如: /dev/i2c-0)
 * @return true : 成功
 * @return false: 失败
 */
bool i2c_open(const char *i2c_dev_name);

/**
 * @brief  关闭I2C设备
 * @param  i2c_dev_name: 输入参数, I2C设备名(如: /dev/i2c-0)
 * @return true : 成功
 * @return false: 失败
 */
bool i2c_close(const char *i2c_dev_name);

/**
 * @brief  向无寄存器地址的I2C从设备发送数据/命令
 * @param  i2c_dev_name : 输入参数, I2C设备名(如: /dev/i2c-0)
 * @param  slave_addr   : 输入参数, I2C从设备地址
 * @param  send_data    : 输入参数, 待发送数据/命令
 * @param  send_data_len: 输入参数, 待发送数据/命令长度
 * @return true : 成功
 * @return false: 失败
 */
bool i2c_write_data(const char *i2c_dev_name, const uint16_t slave_addr,
                    const uint8_t *send_data, const uint32_t send_data_len);

/**
 * @brief  从无寄存器地址的I2C从设备读取数据
 * @param  recv_data    : 输出参数, 读取到的数据
 * @param  i2c_dev_name : 输入参数, I2C设备名(如: /dev/i2c-0)
 * @param  slave_addr   : 输入参数, I2C从设备地址
 * @param  recv_data_len: 输入参数, 待读取数据长度
 * @return true : 成功
 * @return false: 失败
 */
bool i2c_read_data(uint8_t *recv_data, const char *i2c_dev_name,
                   const uint16_t slave_addr, const uint32_t recv_data_len);

/**
 * @brief  向有寄存器的I2C从设备发送数据/命令(即写I2C从设备寄存器)
 * @param  i2c_dev_name  : 输入参数, I2C设备名(如: /dev/i2c-0)
 * @param  slave_addr    : 输入参数, I2C从设备地址
 * @param  reg_addr      : 输入参数, I2C从设备寄存器地址
 * @param  write_data    : 输入参数, 待发送数据/命令
 * @param  write_data_len: 输入参数, 待发送数据/命令长度
 * @return true : 成功
 * @return false: 失败
 */
bool i2c_write_data_sub(const char *i2c_dev_name,
                        const uint16_t slave_addr, const uint8_t reg_addr,
                        const uint8_t *write_data, const uint32_t write_data_len);

/**
 * @brief  从有寄存器地址的I2C从设备读取数据(即读I2C从设备寄存器)
 * @param  recv_data    : 输出参数, 读取到的数据
 * @param  i2c_dev_name : 输入参数, I2C设备名(如: /dev/i2c-0)
 * @param  slave_addr   : 输入参数, I2C从设备地址
 * @param  reg_addr     : 输入参数, I2C从设备寄存器地址
 * @param  recv_data_len: 输入参数, 指定读取长度
 * @return true : 成功
 * @return false: 失败
 */
bool i2c_read_data_sub(uint8_t *recv_data,
                       const char *i2c_dev_name, const uint16_t slave_addr,
                       const uint8_t reg_addr, const uint32_t recv_data_len);

#ifdef __cplusplus
}
#endif

#endif // __I2C_H
