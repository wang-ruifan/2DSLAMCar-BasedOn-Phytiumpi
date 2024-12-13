/******************************************************************************
 * 
 *  file: rpmsg.h
 *  author: Ruifan Wang
 *  description: This file is the declaration of the rpmsg functions. It contains
 *              the declaration of the functions and file variables. It uses the
 *              rpmsg driver to communicate with the remote processor. It can
 *              send and receive data from the remote processor.
 * 
*******************************************************************************/
#ifndef RPMSG_H
#define RPMSG_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <poll.h>
#include <linux/rpmsg.h>
#include <errno.h>

//driver file descriptor
int ctrl_fd, rpmsg_fd;
struct rpmsg_endpoint_info eptinfo;
struct pollfd fds;

//write data to the rpmsg driver
static ssize_t write_full(void *buf, size_t count)
{
    ssize_t ret = 0;
    ssize_t total = 0;

    while (count) {
        ret = write(rpmsg_fd, buf, count);
        if (ret < 0) {
            if (errno == EINTR)
                continue;
            perror("write failed\n");
            break;
        }

        count -= ret;
        buf += ret;
        total += ret;
    }

    return total;
}

//read data from the rpmsg driver
static ssize_t read_full(void *buf, size_t count)
{
    ssize_t res;

    do {
        res = read(rpmsg_fd, buf, count);
    } while (res < 0 && errno == EINTR);

    if ((res < 0 && errno == EAGAIN))
        return 0;

    if (res < 0)
        return -1;

    return res;
}

//send car data to the remote processor
int rpmsgSendCarData(float Vx, float Vz)
{
    char buf[32];
    int ret;
    snprintf(buf, sizeof(buf), " %.2f %.2f", Vx, Vz); // 将用户输入放入buf中
    perror(buf);
    ret = write_full(buf, sizeof(buf)); // 发送数据
    return ret;
}

//get car data from the remote processor
int rpmsgGetCarData(int *break_flag, float *Vx, float *Vz)
{
    char buf[32];
    int ret;
    ret = poll(&fds, 1, 0);
    if (ret < 0) {
        return ret;
    }

    memset(buf, 0, 32);
    ret = read_full(buf, 32);
    if (ret < 0) {
        return ret;
    }
    sscanf(buf," %d %f %f",break_flag, Vx, Vz);
    return ret;
}

//initialize the rpmsg driver
int rpmsgInit()
{
    int ret;
    ctrl_fd = open("/dev/rpmsg_ctrl0", O_RDWR);
    if (ctrl_fd < 0) {
        perror("open rpmsg_ctrl0 failed.\n");
        return -1;
    }

    memcpy(eptinfo.name, "xxxx", 32);
    eptinfo.src = 0;
    eptinfo.dst = 0;

    ret = ioctl(ctrl_fd, RPMSG_CREATE_EPT_IOCTL, eptinfo);
    if (ret != 0) {
        perror("ioctl RPMSG_CREATE_EPT_IOCTL failed.\n");
        close(ctrl_fd);
        return 0;
    }

    rpmsg_fd = open("/dev/rpmsg0", O_RDWR);
    if (rpmsg_fd < 0) {
        perror("open rpmsg0 failed.\n");
        close(rpmsg_fd);
        close(ctrl_fd);
        return 1;
    }

    memset(&fds, 0, sizeof(struct pollfd));
    fds.fd = rpmsg_fd;
    fds.events |= POLLIN;

    //Because in the slave core program, the send action is in the receive callback function, 
    //so we need to send a message to the slave core to trigger the receive callback function,
    //then the slave core can receive the message from the master core and send back the data.
    rpmsgSendCarData(0.0,0.0);
    return 2;
}

//close the rpmsg driver
void rpmsgQuit()
{
    close(rpmsg_fd);
    close(ctrl_fd);
}

#endif // RPMSG_H
