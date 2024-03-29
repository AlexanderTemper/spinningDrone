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
#include <string.h>

#include "platform.h"

#include "rc_modes.h"

#include "common/bitarray.h"
#include "common/maths.h"
#include "drivers/time.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
//#include "pg/rx.h"

//#include "fc/config.h"
//#include "fc/rc_controls.h"

//#include "rx/rx.h"

#define STICKY_MODE_BOOT_DELAY_US 5e3

boxBitmask_t rcModeActivationMask; // one bit per mode defined in boxId_e
static boxBitmask_t stickyModesEverDisabled;

static bool airmodeEnabled;

static int activeMacCount = 0;
static uint8_t activeMacArray[MAX_MODE_ACTIVATION_CONDITION_COUNT];
static int activeLinkedMacCount = 0;
static uint8_t activeLinkedMacArray[MAX_MODE_ACTIVATION_CONDITION_COUNT];

PG_REGISTER_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions, PG_MODE_ACTIVATION_PROFILE, 2);

bool IS_RC_MODE_ACTIVE(boxId_e boxId)
{
    return bitArrayGet(&rcModeActivationMask, boxId);
}

void rcModeUpdate(boxBitmask_t *newState)
{
    rcModeActivationMask = *newState;
}

bool airmodeIsEnabled(void) {
    return airmodeEnabled;
}

bool isRangeActive(uint8_t auxChannelIndex, const channelRange_t *range) {
    if (!IS_RANGE_USABLE(range)) {
        return false;
    }

    const uint16_t channelValue = 1000;//constrain(rcData[auxChannelIndex + NON_AUX_CHANNEL_COUNT], CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX - 1);
    return (channelValue >= 900 + (range->startStep * 25) &&
            channelValue < 900 + (range->endStep * 25));
}

static void updateMasksForMac(const modeActivationCondition_t *mac, boxBitmask_t *andMask, boxBitmask_t *newMask)
{
    bool bAnd = (mac->modeLogic == MODELOGIC_AND) || bitArrayGet(andMask, mac->modeId);
    bool bAct = isRangeActive(mac->auxChannelIndex, &mac->range);
    if (bAnd) {
        bitArraySet(andMask, mac->modeId);
    }
    if (bAnd != bAct) {
        bitArraySet(newMask, mac->modeId);
    }
}

static void updateMasksForStickyModes(const modeActivationCondition_t *mac, boxBitmask_t *andMask, boxBitmask_t *newMask)
{
    if (IS_RC_MODE_ACTIVE(mac->modeId)) {
        bitArrayClr(andMask, mac->modeId);
        bitArraySet(newMask, mac->modeId);
    } else {
        if (bitArrayGet(&stickyModesEverDisabled, mac->modeId)) {
            updateMasksForMac(mac, andMask, newMask);
        } else {
            if (millis() >= STICKY_MODE_BOOT_DELAY_US && !isRangeActive(mac->auxChannelIndex, &mac->range)) {
                bitArraySet(&stickyModesEverDisabled, mac->modeId);
            }
        }
    }
}

static void updateMasksForLinkedMac(const modeActivationCondition_t *mac, boxBitmask_t *andMask, boxBitmask_t *newMask)
{
    bool bAnd = (mac->modeLogic == MODELOGIC_AND) || bitArrayGet(andMask, mac->modeId);
    bool bAct = bitArrayGet(andMask, mac->linkedTo) != bitArrayGet(newMask, mac->linkedTo);
    if (bAnd) {
        bitArraySet(andMask, mac->modeId);
    }
    if (bAnd != bAct) {
        bitArraySet(newMask, mac->modeId);
    }
}

void updateActivatedModes(void)
{
    boxBitmask_t newMask, andMask, stickyModes;
    memset(&andMask, 0, sizeof(andMask));
    memset(&newMask, 0, sizeof(newMask));
    memset(&stickyModes, 0, sizeof(stickyModes));
    bitArraySet(&stickyModes, BOXPARALYZE);

    // determine which conditions set/clear the mode
    for (int i = 0; i < activeMacCount; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(activeMacArray[i]);

        if (bitArrayGet(&stickyModes, mac->modeId)) {
            updateMasksForStickyModes(mac, &andMask, &newMask);
        } else if (mac->modeId < CHECKBOX_ITEM_COUNT) {
            updateMasksForMac(mac, &andMask, &newMask);
        }
    }

    // Update linked modes
    for (int i = 0; i < activeLinkedMacCount; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(activeLinkedMacArray[i]);

        updateMasksForLinkedMac(mac, &andMask, &newMask);
    }

    bitArrayXor(&newMask, sizeof(&newMask), &newMask, &andMask);

    rcModeUpdate(&newMask);

    airmodeEnabled = featureIsEnabled(FEATURE_AIRMODE) || IS_RC_MODE_ACTIVE(BOXAIRMODE);
}

bool isModeActivationConditionPresent(boxId_e modeId)
{
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);

        if (mac->modeId == modeId && (IS_RANGE_USABLE(&mac->range) || mac->linkedTo)) {
            return true;
        }
    }

    return false;
}

bool isModeActivationConditionLinked(boxId_e modeId)
{
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);

        if (mac->modeId == modeId && mac->linkedTo != 0) {
            return true;
        }
    }

    return false;
}

void removeModeActivationCondition(const boxId_e modeId)
{
    unsigned offset = 0;
    for (unsigned i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        modeActivationCondition_t *mac = modeActivationConditionsMutable(i);

        if (mac->modeId == modeId && !offset) {
            offset++;
        }

        if (offset) {
            while (i + offset < MAX_MODE_ACTIVATION_CONDITION_COUNT && modeActivationConditions(i + offset)->modeId == modeId) {
                offset++;
            }

            if (i + offset < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
                memcpy(mac, modeActivationConditions(i + offset), sizeof(modeActivationCondition_t));
            } else {
                memset(mac, 0, sizeof(modeActivationCondition_t));
            }
        }
    }
}

bool isModeActivationConditionConfigured(const modeActivationCondition_t *mac, const modeActivationCondition_t *emptyMac)
{
    if (memcmp(mac, emptyMac, sizeof(*emptyMac))) {
        return true;
    } else {
        return false;
    }
}

// Build the list of used modeActivationConditions indices
// We can then use this to speed up processing by only evaluating used conditions
void analyzeModeActivationConditions(void)
{
    modeActivationCondition_t emptyMac;
    memset(&emptyMac, 0, sizeof(emptyMac));

    activeMacCount = 0;
    activeLinkedMacCount = 0;

    for (uint8_t i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);
        if (mac->linkedTo) {
            activeLinkedMacArray[activeLinkedMacCount++] = i;
        } else if (isModeActivationConditionConfigured(mac, &emptyMac)) {
            activeMacArray[activeMacCount++] = i;
        }
    }
}
