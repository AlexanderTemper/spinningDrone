/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_SERIALRX_SBUS

#include "build/debug.h"

#include "common/utils.h"

#include "drivers/time.h"

#include "io/serial.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/sbus.h"
#include "rx/sbus_channels.h"

/*
 * Observations
 *
 * FrSky X8R
 * time between frames: 6ms.
 * time to send frame: 3ms.
*
 * Futaba R6208SB/R6303SB
 * time between frames: 11ms.
 * time to send frame: 3ms.
 */

#define SBUS_TIME_NEEDED_PER_FRAME 3000

#define SBUS_STATE_FAILSAFE (1 << 0)
#define SBUS_STATE_SIGNALLOSS (1 << 1)

#define SBUS_FRAME_SIZE (SBUS_CHANNEL_DATA_LENGTH + 2)

#define SBUS_FRAME_BEGIN_BYTE 0x0F

#define SBUS_BAUDRATE 100000

#if !defined(SBUS_PORT_OPTIONS)
#define SBUS_PORT_OPTIONS (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN)
#endif

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

enum {
    DEBUG_SBUS_FRAME_FLAGS = 0,
    DEBUG_SBUS_STATE_FLAGS,
    DEBUG_SBUS_FRAME_TIME,
};


struct sbusFrame_s {
    uint8_t syncByte;
    sbusChannels_t channels;
    /**
     * The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value indicates the telemetry byte that is sent after every 4th sbus frame.
     *
     * See https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101027349
     * and
     * https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101706023
     */
    uint8_t endByte;
} __attribute__ ((__packed__));

typedef union sbusFrame_u {
    uint8_t bytes[SBUS_FRAME_SIZE];
    struct sbusFrame_s frame;
} sbusFrame_t;

typedef struct sbusFrameData_s {
    sbusFrame_t frame;
    uint32_t startAtUs;
    uint8_t position;
    bool done;
} sbusFrameData_t;

#define DEBUG_SBUS_FRAME_INTERVAL 3

#define SBUS_FLAG_CHANNEL_17        (1 << 0)
#define SBUS_FLAG_CHANNEL_18        (1 << 1)

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812
sbusFrameData_t sbusd;

uint16_t sbusChannelData[SBUS_MAX_CHANNEL];

uint16_t *rxCh = &sbusChannelData;

uint16_t sbustimeout;

static uint8_t mysbusChannelsDecode(const sbusChannels_t *channels)
{
    sbusChannelData[0] = channels->chan0;
    sbusChannelData[1] = channels->chan1;
    sbusChannelData[2] = channels->chan2;
    sbusChannelData[3] = channels->chan3;
    sbusChannelData[4] = channels->chan4;
    sbusChannelData[5] = channels->chan5;
    sbusChannelData[6] = channels->chan6;
    sbusChannelData[7] = channels->chan7;
    sbusChannelData[8] = channels->chan8;
    sbusChannelData[9] = channels->chan9;
    sbusChannelData[10] = channels->chan10;
    sbusChannelData[11] = channels->chan11;
    sbusChannelData[12] = channels->chan12;
    sbusChannelData[13] = channels->chan13;
    sbusChannelData[14] = channels->chan14;
    sbusChannelData[15] = channels->chan15;

    if (channels->flags & SBUS_FLAG_CHANNEL_17) {
        sbusChannelData[16] = SBUS_DIGITAL_CHANNEL_MAX;
    } else {
        sbusChannelData[16] = SBUS_DIGITAL_CHANNEL_MIN;
    }

    if (channels->flags & SBUS_FLAG_CHANNEL_18) {
        sbusChannelData[17] = SBUS_DIGITAL_CHANNEL_MAX;
    } else {
        sbusChannelData[17] = SBUS_DIGITAL_CHANNEL_MIN;
    }

    if (channels->flags & SBUS_FLAG_FAILSAFE_ACTIVE) {
        // internal failsafe enabled and rx failsafe flag set
        // RX *should* still be sending valid channel data (repeated), so use it.
        return RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE;
    }

    if (channels->flags & SBUS_FLAG_SIGNAL_LOSS) {
        // The received data is a repeat of the last valid data so can be considered complete.
        return RX_FRAME_COMPLETE | RX_FRAME_DROPPED;
    }

    return RX_FRAME_COMPLETE;
}

// Receive ISR callback
void sbusDataReceive(uint16_t c)
{
    sbusFrameData_t *sbusFrameData = &sbusd;
    sbustimeout = 0;

    const uint32_t nowUs = micros();

    const int32_t sbusFrameTime = nowUs - sbusFrameData->startAtUs;

    if (sbusFrameTime > (long)(SBUS_TIME_NEEDED_PER_FRAME + 500)) {
        sbusFrameData->position = 0;
    }

    if (sbusFrameData->position == 0) {
        if (c != SBUS_FRAME_BEGIN_BYTE) {
            return;
        }
        sbusFrameData->startAtUs = nowUs;
    }

    if (sbusFrameData->position < SBUS_FRAME_SIZE) {
        sbusFrameData->frame.bytes[sbusFrameData->position++] = (uint8_t)c;
        if (sbusFrameData->position < SBUS_FRAME_SIZE) {
            sbusFrameData->done = false;
        } else {
            sbusFrameData->done = true;
            DEBUG_SET(DEBUG_SBUS, DEBUG_SBUS_FRAME_TIME, sbusFrameTime);
        }
    }
}

uint8_t sbusFrameStatus(void)
{
    sbusFrameData_t *sbusFrameData = &sbusd;
    if (!sbusFrameData->done) {
        return RX_FRAME_PENDING;
    }
    sbusFrameData->done = false;

    DEBUG_SET(DEBUG_SBUS, DEBUG_SBUS_FRAME_FLAGS, sbusFrameData->frame.frame.channels.flags);

    return mysbusChannelsDecode(&sbusFrameData->frame.frame.channels);
}



#endif
