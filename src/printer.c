#include "printer.h"

#include <zephyr/drivers/gpio.h>
#include <stdint.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <string.h>

#define CMD_INIT 0x01
#define CMD_PRINT 0x02
#define CMD_DATA 0x04
#define CMD_BREAK 0x08 // break and inquiry no action needed, not implemented in process_byte
#define CMD_INQUIRY 0x0F

static const struct gpio_dt_spec clk = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), clk_gpios); // get GPIO info from device tree
static const struct gpio_dt_spec sin = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), sin_gpios);
static const struct gpio_dt_spec sout = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), sout_gpios);

static struct gpio_callback clk_cb_data;

enum printer_state
{
    SYNC0,
    SYNC1,
    COMMAND,
    COMPRESSION,
    LEN_LO,
    LEN_HI,
    PAYLOAD,
    CHK_LO,
    CHK_HI,
    DEVICE_ID,
    STATUS,
};

static struct
{
    enum printer_state state; // initializes to SYNC0
    uint8_t cmd;
    uint8_t compression;
    uint16_t cur_len;
    uint16_t data_len;
    uint16_t cur_sum;
    uint16_t chk_sum;
} packet;

static uint8_t image_buf[5760]; // array of bytes
static size_t image_len;
static uint8_t status;

static bool corrupt;

static uint8_t process_byte(uint8_t rx) // state machine, (write more here) returns the byte to shift out during the next byte (GB protocol uses one-byte lag)
{
    switch (packet.state)
    {
    case SYNC0:
        if (rx == 0x88)
        {
            packet.state = SYNC1;
        }
        break;
    case SYNC1:
        if (rx == 0x33)
        {
            packet.state = COMMAND;
        }
        else
        {
            packet.state = SYNC0;
        }
        break;
    case COMMAND:
        if (rx == CMD_INIT) // INIT
        {
            image_len = 0;
        }
        packet.cmd = rx;
        packet.cur_sum += rx;
        packet.state = COMPRESSION;
        break;
    case COMPRESSION: // not implemented since not needed for GB Cam
        packet.compression = rx;
        packet.cur_sum += rx;
        packet.state = LEN_LO;
        break;
    case LEN_LO:
        packet.data_len = rx;
        packet.cur_sum += rx;
        packet.state = LEN_HI;
        break;
    case LEN_HI:
        packet.data_len = packet.data_len | (rx << 8);
        packet.cur_sum += rx;
        if (packet.data_len <= 640)
        {
            if (packet.data_len > 0)
            {
                packet.state = PAYLOAD;
            }
            else
            {
                packet.state = CHK_LO;
            }
        }
        else
        {
            corrupt = true; // paused here check notes
        }

        break;
    case PAYLOAD:
        packet.cur_len++;
        packet.cur_sum += rx;
        if (packet.cmd == CMD_DATA && image_len < sizeof(image_buf)) // DATA command (PRINT cmd not implemented)
        {
            image_buf[image_len] = rx;
            image_len++;
        }
        if (packet.cur_len == packet.data_len)
        {
            packet.state = CHK_LO;
        }
        break;
    case CHK_LO:
        packet.chk_sum = rx;
        packet.state = CHK_HI;
        break;
    case CHK_HI:
        packet.chk_sum = packet.chk_sum | (rx << 8);
        packet.state = DEVICE_ID;
        return 0x81; // tell game boy printer is alive
    case DEVICE_ID:
        packet.state = STATUS;
        if (packet.chk_sum != packet.cur_sum)
        {
            corrupt = true;
        }
        return status; // do not return checksum error because uncertain checksum bit is even checked by Gameboy Cam

    case STATUS: // update status at the end of packet (updated status will be sent in the next packet)
        switch (packet.cmd)
        {
        case CMD_INIT:
            status = 0x00;
            corrupt = false;
            break;
        case CMD_PRINT:
            if (!corrupt)
            {
                status = 0x06;
                // write to flash function
            }
            else
            {
                status = 0x40;
            }
            break;
        }
        memset(&packet, 0, sizeof(packet));
        break;
    }
    return 0x00;
}

static void clk_cb(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    static uint8_t rx_byte;
    static uint8_t tx_byte;

    static uint32_t clk_cnt;

    if (gpio_pin_get_dt(&clk) == 0) // falling edge 1->0, need to set SIN so that the GB can read on rising edge
    {
        gpio_pin_set_dt(&sin, tx_byte & BIT(7)); // get the left most bit and set the gpio
        tx_byte <<= 1;                           // set next bit to left most position
    }
    else // rising edge 0->1, read SOUT (data from GB) (GB will read data from SIN too)
    {
        rx_byte = (rx_byte << 1) | gpio_pin_get_dt(&sout); // shift bits left and set the least significant bit to the SOUT reading
        clk_cnt++;
        if (clk_cnt == 8)
        {
            clk_cnt = 0;
            tx_byte = process_byte(rx_byte);
        }
    }
}

int printer_init(void)
{
    if (!gpio_is_ready_dt(&clk) || !gpio_is_ready_dt(&sin) || !gpio_is_ready_dt(&sout))
    {
        return -ENODEV;
    }

    gpio_pin_configure_dt(&clk, GPIO_INPUT);           // initialize pins
    gpio_pin_configure_dt(&sin, GPIO_OUTPUT_INACTIVE); // sin is output pin since it is input into the gameboy not into the peripheral (set to low)
    gpio_pin_configure_dt(&sout, GPIO_INPUT);

    gpio_init_callback(&clk_cb_data, clk_cb, BIT(clk.pin)); // initialize clk_cb_data
    // BIT(clk.pin) bitmask selecting CLK's pin within the GPIO port.
    gpio_add_callback(clk.port, &clk_cb_data);                 // register callback to gpio controller
    gpio_pin_interrupt_configure_dt(&clk, GPIO_INT_EDGE_BOTH); // specify when interrupt should occur

    return 0;
}