/**
 * @file      : i2c.c
 * @brief     : Linux平台I2C驱动源文件
 * @author    : huenrong (huenrong1028@outlook.com)
 * @date      : 2023-01-18 13:16:50
 *
 * @copyright : Copyright (c) 2023 huenrong
 *
 * @history   : date       author          description
 *              2023-01-18 huenrong        创建文件
 *
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "./i2c.h"

// I2C设备名最大长度
#define I2C_DEV_NAME_MAX_LEN 30

// I2C设备最大数量
#define I2C_DEV_MAX_NUM 10

// I2C设备信息结构体
typedef struct
{
    char i2c_dev_name[I2C_DEV_NAME_MAX_LEN]; // I2C设备名
    int i2c_dev_fd;                          // I2C设备文件描述符
    pthread_mutex_t i2c_dev_mutex;           // I2C设备互斥锁
} i2c_dev_info_t;

// 已打开的I2C设备数量
static uint8_t g_i2c_dev_num = 0;
// I2C设备信息
static i2c_dev_info_t g_i2c_dev_info[I2C_DEV_MAX_NUM];

/**
 * @brief  查找指定I2C设备的信息
 * @param  i2c_dev_info: 输出参数, 查找到的I2C设备信息
 * @param  i2c_dev_name: 输入参数, 待查找的I2C设备名
 * @return true : 成功
 * @return false: 失败
 */
static bool i2c_find_dev_info(i2c_dev_info_t *i2c_dev_info, const char *i2c_dev_name)
{
    int ret = -1;

    if ((!i2c_dev_info) || (!i2c_dev_name))
    {
        return false;
    }

    for (uint8_t i = 0; i < g_i2c_dev_num; i++)
    {
        ret = memcmp(g_i2c_dev_info[i].i2c_dev_name, i2c_dev_name, strlen(i2c_dev_name));
        if (0 == ret)
        {
            memcpy(i2c_dev_info, &g_i2c_dev_info[i], sizeof(i2c_dev_info_t));

            return true;
        }
    }

    return false;
}

/**
 * @brief  打开I2C设备
 * @param  i2c_dev_name: 输入参数, I2C设备名(如: /dev/i2c-0)
 * @return true : 成功
 * @return false: 失败
 */
bool i2c_open(const char *i2c_dev_name)
{
    // I2C设备描述符
    int fd = -1;
    // I2C设备信息
    i2c_dev_info_t i2c_dev_info = {0};

    if (!i2c_dev_name)
    {
        return false;
    }

    // 超过支持的I2C设备数量, 直接返回错误
    if (g_i2c_dev_num > I2C_DEV_MAX_NUM)
    {
        return false;
    }

    // I2C设备已打开, 直接返回成功
    if (i2c_find_dev_info(&i2c_dev_info, i2c_dev_name))
    {
        return true;
    }

    // 打开I2C设备
    fd = open(i2c_dev_name, O_RDWR);
    if (fd < 0)
    {
        return false;
    }

    // 记录I2C设备信息
    memcpy(g_i2c_dev_info[g_i2c_dev_num].i2c_dev_name, i2c_dev_name, strlen(i2c_dev_name));
    g_i2c_dev_info[g_i2c_dev_num].i2c_dev_fd = fd;

    // 初始化互斥锁
    pthread_mutex_init(&g_i2c_dev_info[g_i2c_dev_num].i2c_dev_mutex, NULL);

    g_i2c_dev_num++;

    return true;
}

/**
 * @brief  关闭I2C设备
 * @param  i2c_dev_name: 输入参数, I2C设备名(如: /dev/i2c-0)
 * @return true : 成功
 * @return false: 失败
 */
bool i2c_close(const char *i2c_dev_name)
{
    int ret = -1;

    if (!i2c_dev_name)
    {
        return false;
    }

    for (uint8_t i = 0; i < g_i2c_dev_num; i++)
    {
        ret = memcmp(g_i2c_dev_info[i].i2c_dev_name, i2c_dev_name, strlen(i2c_dev_name));
        // 当前I2C设备已打开
        if (0 == ret)
        {
            pthread_mutex_lock(&g_i2c_dev_info[i].i2c_dev_mutex);

            // 关闭I2C设备
            ret = close(g_i2c_dev_info[i].i2c_dev_fd);
            if (ret < 0)
            {
                pthread_mutex_unlock(&g_i2c_dev_info[i].i2c_dev_mutex);

                return false;
            }

            pthread_mutex_unlock(&g_i2c_dev_info[i].i2c_dev_mutex);

            // 清空I2C设备信息
            g_i2c_dev_info[i].i2c_dev_fd = -1;
            memset(g_i2c_dev_info[i].i2c_dev_name, 0, I2C_DEV_NAME_MAX_LEN);

            // 销毁互斥锁
            pthread_mutex_destroy(&g_i2c_dev_info[i].i2c_dev_mutex);

            // 将I2C设备信息放到数组最前面
            memcpy(&g_i2c_dev_info[i], &g_i2c_dev_info[i + 1], (sizeof(i2c_dev_info_t) * (I2C_DEV_MAX_NUM - i - 1)));

            (g_i2c_dev_num > 0) ? g_i2c_dev_num-- : 0;

            return true;
        }
    }

    return true;
}

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
                    const uint8_t *send_data, const uint32_t send_data_len)
{
    int ret = -1;
    // I2C设备信息
    i2c_dev_info_t i2c_dev_info = {0};

    if ((!send_data) || (!i2c_dev_name))
    {
        return false;
    }

    // I2C设备未打开, 直接返回失败
    if (!i2c_find_dev_info(&i2c_dev_info, i2c_dev_name))
    {
        return false;
    }

    pthread_mutex_lock(&i2c_dev_info.i2c_dev_mutex);

    // 设置从器件地址
    // 不带寄存器地址的从设备操作前需要先设置从器件地址
    ret = ioctl(i2c_dev_info.i2c_dev_fd, I2C_SLAVE_FORCE, slave_addr);
    if (0 != ret)
    {
        pthread_mutex_unlock(&i2c_dev_info.i2c_dev_mutex);

        return false;
    }

    // 向I2C从设备发送数据/命令
    ret = write(i2c_dev_info.i2c_dev_fd, send_data, send_data_len);
    if (send_data_len != (uint32_t)ret)
    {
        pthread_mutex_unlock(&i2c_dev_info.i2c_dev_mutex);

        return false;
    }

    pthread_mutex_unlock(&i2c_dev_info.i2c_dev_mutex);

    return true;
}

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
                   const uint16_t slave_addr, const uint32_t recv_data_len)
{
    int ret = -1;
    // I2C设备信息
    i2c_dev_info_t i2c_dev_info = {0};

    if ((!recv_data) || (!i2c_dev_name))
    {
        return false;
    }

    // I2C设备未打开, 直接返回失败
    if (!i2c_find_dev_info(&i2c_dev_info, i2c_dev_name))
    {
        return false;
    }

    pthread_mutex_lock(&i2c_dev_info.i2c_dev_mutex);

    // 设置从器件地址
    // 不带寄存器地址的从设备操作前需要先设置从器件地址
    ret = ioctl(i2c_dev_info.i2c_dev_fd, I2C_SLAVE_FORCE, slave_addr);
    if (0 != ret)
    {
        pthread_mutex_unlock(&i2c_dev_info.i2c_dev_mutex);

        return false;
    }

    // 从I2C从设备读取数据
    ret = read(i2c_dev_info.i2c_dev_fd, recv_data, recv_data_len);
    if (recv_data_len != (uint32_t)ret)
    {
        pthread_mutex_unlock(&i2c_dev_info.i2c_dev_mutex);

        return false;
    }

    pthread_mutex_unlock(&i2c_dev_info.i2c_dev_mutex);

    return true;
}

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
                        const uint8_t *write_data, const uint32_t write_data_len)
{
    int ret = -1;
    struct i2c_rdwr_ioctl_data ioctl_data = {0};
    // I2C设备信息
    i2c_dev_info_t i2c_dev_info = {0};

    if ((!write_data) || (!i2c_dev_name))
    {
        return false;
    }

    // I2C设备未打开, 直接返回失败
    if (!i2c_find_dev_info(&i2c_dev_info, i2c_dev_name))
    {
        return false;
    }

    pthread_mutex_lock(&i2c_dev_info.i2c_dev_mutex);

    // 消息数
    memset(&ioctl_data, 0, sizeof(struct i2c_rdwr_ioctl_data));
    ioctl_data.nmsgs = 1;

    ioctl_data.msgs = (struct i2c_msg *)malloc(ioctl_data.nmsgs * sizeof(struct i2c_msg));
    if (!ioctl_data.msgs)
    {
        pthread_mutex_unlock(&i2c_dev_info.i2c_dev_mutex);

        return false;
    }

    ioctl_data.msgs[0].buf = (uint8_t *)malloc((write_data_len + 1) * sizeof(uint8_t));
    if (!ioctl_data.msgs[0].buf)
    {
        pthread_mutex_unlock(&i2c_dev_info.i2c_dev_mutex);

        free(ioctl_data.msgs);

        return false;
    }

    // 从器件地址
    ioctl_data.msgs[0].addr = slave_addr;
    // 数据长度(buf的长度)
    ioctl_data.msgs[0].len = (write_data_len + 1);
    // 操作方式(0: write; 1: read)
    ioctl_data.msgs[0].flags = 0;
    // 寄存器地址
    ioctl_data.msgs[0].buf[0] = reg_addr;
    // 发送数据
    memcpy(&(ioctl_data.msgs[0].buf[1]), write_data, write_data_len);

    ret = ioctl(i2c_dev_info.i2c_dev_fd, I2C_RDWR, (unsigned long)&ioctl_data);
    if (-1 == ret)
    {
        pthread_mutex_unlock(&i2c_dev_info.i2c_dev_mutex);

        free(ioctl_data.msgs[0].buf);
        free(ioctl_data.msgs);

        return false;
    }

    free(ioctl_data.msgs[0].buf);
    free(ioctl_data.msgs);

    pthread_mutex_unlock(&i2c_dev_info.i2c_dev_mutex);

    return true;
}

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
                       const uint8_t reg_addr, const uint32_t recv_data_len)
{
    int ret = -1;
    struct i2c_rdwr_ioctl_data ioctl_data = {0};
    // I2C设备信息
    i2c_dev_info_t i2c_dev_info = {0};

    if ((!recv_data) || (!i2c_dev_name))
    {
        return false;
    }

    // I2C设备未打开, 直接返回失败
    if (!i2c_find_dev_info(&i2c_dev_info, i2c_dev_name))
    {
        return false;
    }

    pthread_mutex_lock(&i2c_dev_info.i2c_dev_mutex);

    // 消息数
    ioctl_data.nmsgs = 2;

    ioctl_data.msgs = (struct i2c_msg *)malloc(ioctl_data.nmsgs * sizeof(struct i2c_msg));
    if (!ioctl_data.msgs)
    {
        pthread_mutex_unlock(&i2c_dev_info.i2c_dev_mutex);

        return false;
    }

    ioctl_data.msgs[0].buf = (uint8_t *)malloc(sizeof(uint8_t));
    if (!ioctl_data.msgs[0].buf)
    {
        pthread_mutex_unlock(&i2c_dev_info.i2c_dev_mutex);

        free(ioctl_data.msgs);

        return false;
    }

    ioctl_data.msgs[1].buf = (uint8_t *)malloc(recv_data_len * sizeof(uint8_t));
    if (!ioctl_data.msgs[1].buf)
    {
        pthread_mutex_unlock(&i2c_dev_info.i2c_dev_mutex);

        free(ioctl_data.msgs[0].buf);
        free(ioctl_data.msgs);

        return false;
    }

    // 从器件地址
    ioctl_data.msgs[0].addr = slave_addr;
    // 数据长度
    ioctl_data.msgs[0].len = 1;
    // 操作方式(0: write; 1: read)
    ioctl_data.msgs[0].flags = 0;
    // 寄存器地址
    ioctl_data.msgs[0].buf[0] = reg_addr;

    // 从器件地址
    ioctl_data.msgs[1].addr = slave_addr;
    // 数据长度
    ioctl_data.msgs[1].len = recv_data_len;
    // 操作方式(0: write; 1: read)
    ioctl_data.msgs[1].flags = 1;
    // 初始化
    memset(ioctl_data.msgs[1].buf, 0, recv_data_len);

    ret = ioctl(i2c_dev_info.i2c_dev_fd, I2C_RDWR, (unsigned long)&ioctl_data);
    if (-1 == ret)
    {
        pthread_mutex_unlock(&i2c_dev_info.i2c_dev_mutex);

        free(ioctl_data.msgs[1].buf);
        free(ioctl_data.msgs[0].buf);
        free(ioctl_data.msgs);

        return false;
    }

    memcpy(recv_data, ioctl_data.msgs[1].buf, recv_data_len);

    free(ioctl_data.msgs[1].buf);
    free(ioctl_data.msgs[0].buf);
    free(ioctl_data.msgs);

    pthread_mutex_unlock(&i2c_dev_info.i2c_dev_mutex);

    return true;
}
