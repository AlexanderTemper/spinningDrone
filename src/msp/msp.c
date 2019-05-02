#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "imu.h"

#include <math.h>
#include "common/utils.h"
#include "build/debug.h"
#include "config/feature.h"
#include "common/bitarray.h"

#include "msp/msp_box.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"


#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "msp.h"

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

static bool featureMaskIsCopied = false;
static uint32_t featureMaskCopy;

static uint32_t getFeatureMask(void)
{
    if (featureMaskIsCopied) {
        return featureMaskCopy;
    } else {
        return featureMask();
    }
}


static bool mspCommonProcessOutCommand(uint8_t cmdMSP, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn) {
	UNUSED(mspPostProcessFn);

	switch (cmdMSP) {
	case MSP_BATTERY_CONFIG: //TODO
	        sbufWriteU8(dst, (330 + 5) / 10);
	        sbufWriteU8(dst, (430 + 5) / 10);
	        sbufWriteU8(dst, (350 + 5) / 10);
	        sbufWriteU16(dst, 2500 );
	        sbufWriteU8(dst, 0);
	        sbufWriteU8(dst, 0);
	        sbufWriteU16(dst, 330);
	        sbufWriteU16(dst, 430);
	        sbufWriteU16(dst, 350);
	break;
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
        sbufWriteU16(dst, 0); // No other build targets currently have hardware revision detection
        sbufWriteU8(dst, 0);  // 0 == FC
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, strlen("TEST"));
        sbufWriteData(dst, "TEST", strlen("TEST"));
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        uint8_t emptySignature[SIGNATURE_LENGTH];
        memset(emptySignature, 0, sizeof(emptySignature));
        sbufWriteData(dst, &emptySignature, sizeof(emptySignature));
        sbufWriteU8(dst, 255);
        break;
    }

    case MSP_BUILD_INFO:
        sbufWriteData(dst, buildDate, BUILD_DATE_LENGTH);
        sbufWriteData(dst, buildTime, BUILD_TIME_LENGTH);
        sbufWriteData(dst, shortGitRevision, GIT_SHORT_REVISION_LENGTH);
        break;

    case MSP_ANALOG:
        sbufWriteU8(dst, 3);
        sbufWriteU16(dst, test++); // milliamp hours drawn from battery
        sbufWriteU16(dst, test++);
        sbufWriteU16(dst, test++); // send current in 0.01 A steps, range is -320A to 320A
        sbufWriteU16(dst, test++);
        break;
    case MSP_DEBUG:
            for (int i = 0; i < DEBUG16_VALUE_COUNT; i++) {
                sbufWriteU16(dst, debug[i]);      // 4 variables are here for general monitoring purpose
            }
    break;
    case MSP_UID:
        sbufWriteU32(dst, 0);
        sbufWriteU32(dst, 1);
        sbufWriteU32(dst, 2);
        break;
    case MSP_FEATURE_CONFIG:
            sbufWriteU32(dst, getFeatureMask());
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
	case MSP_STATUS_EX:
	    case MSP_STATUS:
	        {
	        	boxBitmask_t flightModeFlags;
	            const int flagBits = packFlightModeFlags(&flightModeFlags);

	            sbufWriteU16(dst, 10);//getTaskDeltaTime(TASK_GYROPID));
	#ifdef USE_I2C
	            sbufWriteU16(dst, i2cGetErrorCounter());
	#else
	            sbufWriteU16(dst, 0);
	#endif
	            sbufWriteU16(dst, sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_RANGEFINDER) << 4 | sensors(SENSOR_GYRO) << 5);
	            sbufWriteData(dst, &flightModeFlags, 4);        // unconditional part of flags, first 32 bits
	            sbufWriteU8(dst, 0);//getCurrentPidProfileIndex());
	            sbufWriteU16(dst, 10);//constrain(averageSystemLoadPercent, 0, 100));
	            if (cmdMSP == MSP_STATUS_EX) {
	                sbufWriteU8(dst, 0);//PID_PROFILE_COUNT);
	                sbufWriteU8(dst, 0);//getCurrentControlRateProfileIndex());
	            } else {  // MSP_STATUS
	                sbufWriteU16(dst, 0); // gyro cycle time
	            }

	            // write flightModeFlags header. Lowest 4 bits contain number of bytes that follow
	            // header is emited even when all bits fit into 32 bits to allow future extension
	            int byteCount = (flagBits - 32 + 7) / 8;        // 32 already stored, round up
	            byteCount = constrain(byteCount, 0, 15);        // limit to 16 bytes (128 bits)
	            sbufWriteU8(dst, byteCount);
	            sbufWriteData(dst, ((uint8_t*)&flightModeFlags) + 4, byteCount);

	            // Write arming disable flags
	            // 1 byte, flag count
	            sbufWriteU8(dst, 0);//ARMING_DISABLE_FLAGS_COUNT);
	            // 4 bytes, flags
	            //const uint32_t armingDisableFlags = getArmingDisableFlags();
	            sbufWriteU32(dst, 0);//armingDisableFlags);
	        }
	        break;

	case MSP_RAW_IMU: {
		uint8_t scale;
			if (acc.dev.acc_1G > 512 * 4) {
				scale = 8;
			} else if (acc.dev.acc_1G > 512 * 2) {
				scale = 4;
			} else if (acc.dev.acc_1G >= 512) {
				scale = 2;
			} else {
				scale = 1;
		}
		for (int i = 0; i < 3; i++) {
			sbufWriteU16(dst, lrintf(acc.accADC[i] / scale));
		}
		for (int i = 0; i < 3; i++) {
			sbufWriteU16(dst, gyroRateDps(i));
		}
		for (int i = 0; i < 3; i++) {
			sbufWriteU16(dst, 100);
		}
		break;
	}
	case MSP_NAME:
	{
	   const char pilotname[] = "Alex";
	   const int nameLen = strlen(pilotname);
	   for (int i = 0; i < nameLen; i++) {
		   sbufWriteU8(dst, pilotname[i]);
	   }
	}
	break;
	case MSP_ATTITUDE:
		sbufWriteU16(dst, attitude.values.roll);
		sbufWriteU16(dst, attitude.values.pitch);
		sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
	break;
	case MSP_ACC_TRIM: //TODO
		sbufWriteU16(dst, 0);
		sbufWriteU16(dst, 0);
	break;
	case MSP_MIXER_CONFIG: //TODO
		sbufWriteU8(dst, 2);
		sbufWriteU8(dst, 0);
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
		case MSP_SET_ARMING_DISABLED:
		{
			const uint8_t command = sbufReadU8(src);
			uint8_t disableRunawayTakeoff = 0;
#ifndef USE_RUNAWAY_TAKEOFF
			UNUSED(disableRunawayTakeoff);
#endif
			if (sbufBytesRemaining(src)) {
				disableRunawayTakeoff = sbufReadU8(src);
			}
			if (command) {
				setArmingDisabled(ARMING_DISABLED_MSP);
				if (ARMING_FLAG(ARMED)) {
					//disarm(); // TODO
				}
#ifdef USE_RUNAWAY_TAKEOFF
				runawayTakeoffTemporaryDisable(false);
#endif
			} else {
				unsetArmingDisabled(ARMING_DISABLED_MSP);
#ifdef USE_RUNAWAY_TAKEOFF
				runawayTakeoffTemporaryDisable(disableRunawayTakeoff);
#endif
			}
		}
	break;
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
	case MSP_BOXNAMES:
	{
		const int page = sbufBytesRemaining(src) ? sbufReadU8(src) : 0;
		serializeBoxReply(dst, page, &serializeBoxNameFn);
	}
	break;
	case MSP_BOXIDS:
	{
		const int page = sbufBytesRemaining(src) ? sbufReadU8(src) : 0;
		serializeBoxReply(dst, page, &serializeBoxPermanentIdFn);
	}
	break;
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

void mspInit(void)
{
    initActiveBoxIds();
}
