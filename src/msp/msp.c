#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/utils.h"

#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"
#include "msp.h"
#include "imu.h"

uint16_t test = 0;
// TODO besser machen ----
static const char * const flightControllerIdentifier = BETAFLIGHT_IDENTIFIER; // 4 UPPER
#define FC_VERSION_MAJOR 4 // increment when a major release is made (big new feature, etc)
#define FC_VERSION_MINOR 1 // increment when a minor release is made (small new feature, change etc)
#define FC_VERSION_PATCH_LEVEL 0 // increment when a bug is fixed
#define SIGNATURE_LENGTH 32
const char * const targetName = "UNITTEST";
const char* const buildDate = "Jan 01 2017";
const char * const buildTime = "00:00:00";
const char * const shortGitRevision = "MASTER";
#define GIT_SHORT_REVISION_LENGTH   7 // lower case hexadecimal digits.
#define BUILD_DATE_LENGTH 11
#define BUILD_TIME_LENGTH 8
#define MSP_API_VERSION_STRING STR(API_VERSION_MAJOR) "." STR(API_VERSION_MINOR)
//--------

static bool mspCommonProcessOutCommand(uint8_t cmdMSP, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn) {
	UNUSED(mspPostProcessFn);

	switch (cmdMSP) {
	case MSP_API_VERSION:
		sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
		sbufWriteU8(dst, API_VERSION_MAJOR);
		sbufWriteU8(dst, API_VERSION_MINOR);
		break;
	case MSP_FC_VARIANT:
        sbufWriteData(dst, flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
        break;

    case MSP_FC_VERSION:
        sbufWriteU8(dst, FC_VERSION_MAJOR);
        sbufWriteU8(dst, FC_VERSION_MINOR);
        sbufWriteU8(dst, FC_VERSION_PATCH_LEVEL);
        break;

    case MSP_BOARD_INFO:
    {
        sbufWriteData(dst, "BEBR", BOARD_IDENTIFIER_LENGTH);
#ifdef USE_HARDWARE_REVISION_DETECTION
        sbufWriteU16(dst, hardwareRevision);
#else
        sbufWriteU16(dst, 0); // No other build targets currently have hardware revision detection.
#endif
#if defined(USE_OSD) && defined(USE_MAX7456)
        sbufWriteU8(dst, 2);  // 2 == FC with OSD
#else
        sbufWriteU8(dst, 0);  // 0 == FC
#endif
        // Target capabilities (uint8)
#define TARGET_HAS_VCP_BIT 0
#define TARGET_HAS_SOFTSERIAL_BIT 1
#define TARGET_IS_UNIFIED_BIT 2

        uint8_t targetCapabilities = 0;
#ifdef USE_VCP
        targetCapabilities |= 1 << TARGET_HAS_VCP_BIT;
#endif
#if defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2)
        targetCapabilities |= 1 << TARGET_HAS_SOFTSERIAL_BIT;
#endif
#if defined(USE_UNIFIED_TARGET)
        targetCapabilities |= 1 << TARGET_IS_UNIFIED_BIT;
#endif

        sbufWriteU8(dst, targetCapabilities);

        // Target name with explicit length
        sbufWriteU8(dst, strlen("TEST"));
        sbufWriteData(dst, "TEST", strlen("TEST"));

#if defined(USE_BOARD_INFO)
        // Board name with explicit length
        char *value = getBoardName();
        sbufWriteU8(dst, strlen(value));
        sbufWriteString(dst, value);

        // Manufacturer id with explicit length
        value = getManufacturerId();
        sbufWriteU8(dst, strlen(value));
        sbufWriteString(dst, value);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif

#if defined(USE_SIGNATURE)
        // Signature
        sbufWriteData(dst, getSignature(), SIGNATURE_LENGTH);
#else
        uint8_t emptySignature[SIGNATURE_LENGTH];
        memset(emptySignature, 0, sizeof(emptySignature));
        sbufWriteData(dst, &emptySignature, sizeof(emptySignature));
#endif

        sbufWriteU8(dst, 255);

        break;
    }

    case MSP_BUILD_INFO:
        sbufWriteData(dst, buildDate, BUILD_DATE_LENGTH);
        sbufWriteData(dst, buildTime, BUILD_TIME_LENGTH);
        sbufWriteData(dst, shortGitRevision, GIT_SHORT_REVISION_LENGTH);
        break;

    case MSP_ANALOG:
        sbufWriteU8(dst, 0);
        if(test == 1000){
        	test = 0;
        }
        sbufWriteU16(dst, test++); // milliamp hours drawn from battery
        sbufWriteU16(dst, test++);
        sbufWriteU16(dst, test++); // send current in 0.01 A steps, range is -320A to 320A
        sbufWriteU16(dst, test++);
        break;

    case MSP_UID:
        sbufWriteU32(dst, 0);
        sbufWriteU32(dst, 1);
        sbufWriteU32(dst, 2);
        break;

    case MSP_NAME:
	   {
		   const char pilotname[] = "Alex";
		   const int nameLen = strlen(pilotname);
		   for (int i = 0; i < nameLen; i++) {
			   sbufWriteU8(dst, pilotname[i]);
		   }
	   }
	   break;
    default:
        return false;
    }
    return true;
}

int dummy;
static bool mspProcessOutCommand(uint8_t cmdMSP, sbuf_t *dst) {
	bool unsupportedCommand = false;
	switch (cmdMSP) {
	case MSP_RAW_IMU: {
		for (int i = 0; i < 3; i++) {
			sbufWriteU16(dst, dummy++);
		}
		for (int i = 0; i < 3; i++) {
			sbufWriteU16(dst, dummy++);
		}
		for (int i = 0; i < 3; i++) {
			sbufWriteU16(dst, dummy++);
		}
		break;
	}
	case MSP_ATTITUDE:
	        sbufWriteU16(dst, attitude.values.roll);
	        sbufWriteU16(dst, attitude.values.pitch);
	        sbufWriteU16(dst, attitude.values.yaw);
	break;
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

