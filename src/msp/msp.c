#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/utils.h"

#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"
#include "msp.h"

static bool mspCommonProcessOutCommand(uint8_t cmdMSP, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn) {
	UNUSED(mspPostProcessFn);

	switch (cmdMSP) {
	case MSP_API_VERSION:
		sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
		sbufWriteU8(dst, API_VERSION_MAJOR);
		sbufWriteU8(dst, API_VERSION_MINOR);
		break;
	default:
		return false;
	}
	return true;
}

static bool mspProcessOutCommand(uint8_t cmdMSP, sbuf_t *dst) {
	bool unsupportedCommand = false;
	switch (cmdMSP) {
	default:
		unsupportedCommand = true;
	}
	return !unsupportedCommand;
}

static mspResult_e mspProcessInCommand(uint8_t cmdMSP, sbuf_t *src) {
	/*uint32_t i;
	uint8_t value;
	const unsigned int dataSize = sbufBytesRemaining(src);*/
	switch (cmdMSP) {
	default:
		// we do not know how to handle the (valid) message, indicate error MSP $M!
		return MSP_RESULT_ERROR;
	}
	return MSP_RESULT_ACK;
}

static mspResult_e mspCommonProcessInCommand(uint8_t cmdMSP, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn)
{
    UNUSED(mspPostProcessFn);
    const unsigned int dataSize = sbufBytesRemaining(src);
    UNUSED(dataSize); // maybe unused due to compiler options

    switch (cmdMSP) {

    default:
        return mspProcessInCommand(cmdMSP, src);
    }
    return MSP_RESULT_ACK;
}

static mspResult_e mspFcProcessOutCommandWithArg(uint8_t cmdMSP, sbuf_t *src, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn)
{
	switch (cmdMSP) {
	default:
		return MSP_RESULT_CMD_UNKNOWN;
	}
	return MSP_RESULT_ACK;
}

/*
 * Returns MSP_RESULT_ACK, MSP_RESULT_ERROR or MSP_RESULT_NO_REPLY
 */
mspResult_e mspFcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn) {
	int ret = MSP_RESULT_ACK;
	sbuf_t *dst = &reply->buf;
	sbuf_t *src = &cmd->buf;
	const uint8_t cmdMSP = cmd->cmd;
	// initialize reply by default
	reply->cmd = cmd->cmd;

	if (mspCommonProcessOutCommand(cmdMSP, dst, mspPostProcessFn)) {
		ret = MSP_RESULT_ACK;
	} else if (mspProcessOutCommand(cmdMSP, dst)) {
		ret = MSP_RESULT_ACK;
	} else if ((ret = mspFcProcessOutCommandWithArg(cmdMSP, src, dst,
			mspPostProcessFn)) != MSP_RESULT_CMD_UNKNOWN) {
		/* ret */;
	} else {
		ret = mspCommonProcessInCommand(cmdMSP, src, mspPostProcessFn);
	}
	reply->result = ret;
	return ret;
}
void mspFcProcessReply(mspPacket_t *reply)
{
    sbuf_t *src = &reply->buf;
    UNUSED(src); // potentially unused depending on compile options.

    switch (reply->cmd) {
    case MSP_ANALOG:
        {
            uint8_t batteryVoltage = sbufReadU8(src);
            uint16_t mAhDrawn = sbufReadU16(src);
            uint16_t rssi = sbufReadU16(src);
            uint16_t amperage = sbufReadU16(src);

            UNUSED(rssi);
            UNUSED(batteryVoltage);
            UNUSED(amperage);
            UNUSED(mAhDrawn);

#ifdef USE_MSP_CURRENT_METER
            currentMeterMSPSet(amperage, mAhDrawn);
#endif
        }
        break;
    }
}

