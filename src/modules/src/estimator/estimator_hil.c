/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * estimator_hil.c - a hil estimator
 */

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "stabilizer.h"
#include "estimator_hil.h"
#include "sensfusion6.h"
#include "position_estimator.h"
#include "sensors.h"
#include "stabilizer_types.h"
#include "static_mem.h"
#include "log.h"
#include "param.h"

static tofMeasurement_t tof;

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT (1.0 / ATTITUDE_UPDATE_RATE)

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT (1.0 / POS_UPDATE_RATE)

static baro_t baro;
static tofMeasurement_t tof;

// TODO: Look into the LOG_ADD Macro what is suposed to be the name and where it saves the value
// Maybe change them to a struct later for clearer structure
// Maybe need to sim the baro also?
static volatile float simPosX = 0.0;
static volatile float simPosY = 0.0;
static volatile float simPosZ = 0.0;

//Note: Currently not used
static volatile float simVelX = 0.0;
static volatile float simVelY = 0.0;
static volatile float simVelZ = 0.0;

static volatile float simRotRoll = 0.0;
static volatile float simRotPitch = 0.0;
static volatile float simRotYaw = 0.0;

void estimatorHILInit(void)
{
    sensfusion6Init();
}

bool estimatorHILTest(void)
{
    bool pass = true;

    pass &= sensfusion6Test();

    return pass;
}

void estimatorHIL(state_t *state, const stabilizerStep_t stabilizerStep)
{
    // TODO change to own set values (USE THE state variable and update it according to the simulation values)

    // Pull the latest sensors values of interest; discard the rest
    measurement_t m;
    while (estimatorDequeue(&m))
    {
        switch (m.type)
        {
        case MeasurementTypeBarometer:
            baro = m.data.barometer.baro;
            break;

        case MeasurementTypeTOF:
            tof = m.data.tof;
            break;

        default:
            break;
        }
    }

    // Update filter
    // if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, stabilizerStep)) {
    /* sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z,
                        acc.x, acc.y, acc.z,
                        ATTITUDE_UPDATE_DT); */

    // Save attitude, adjusted for the legacy CF2 body coordinate system
    // sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);
    // state->attitude.roll = simRotRoll;
    // state->attitude.pitch = simRotPitch;
    // state->attitude.yaw = simRotYaw;

    // Save quaternion, hopefully one day this could be used in a better controller.
    // Note that this is not adjusted for the legacy coordinate system
    /* sensfusion6GetQuaternion(
      &state->attitudeQuaternion.x,
      &state->attitudeQuaternion.y,
      &state->attitudeQuaternion.z,
      &state->attitudeQuaternion.w); */

    /* state->acc.z = sensfusion6GetAccZWithoutGravity(acc.x,
                                                    acc.y,
                                                    acc.z); */

    // positionUpdateVelocity(state->acc.z, ATTITUDE_UPDATE_DT);
    //}

    if (RATE_DO_EXECUTE(POS_UPDATE_RATE, stabilizerStep)) 
    {
        state->attitude.roll = simRotRoll;
        state->attitude.pitch = simRotPitch;
        state->attitude.yaw = simRotYaw;

        state->position.x = simPosX;
        state->position.y = simPosY;
        state->position.z = simPosZ;

        positionEstimate(state, &baro, &tof, POS_UPDATE_DT, stabilizerStep);
    }
}

PARAM_GROUP_START(hil)

    /**
     * @brief Simulated position of the global frame: x
     *
     */
    PARAM_ADD_CORE(PARAM_FLOAT, simPosX, &simPosX)

    /**
     * @brief Simulated position of the global frame: y
     *
     */
    PARAM_ADD_CORE(PARAM_FLOAT, simPosY, &simPosY)

    /**
     * @brief Simulated position of the global frame: z
     *
     */
    PARAM_ADD_CORE(PARAM_FLOAT, simPosZ, &simPosZ)

    /**
     * @brief Simulated velocity of the global frame: x
     *
     */
    PARAM_ADD_CORE(PARAM_FLOAT, simVelX, &simVelX)

    /**
     * @brief Simulated velocity of the global frame: y
     *
     */
    PARAM_ADD_CORE(PARAM_FLOAT, simVelY, &simVelY)

    /**
     * @brief Simulated velocity of the global frame: z
     *
     */
    PARAM_ADD_CORE(PARAM_FLOAT, simVelZ, &simVelZ)

    /**
     * @brief Simulated rotation of the local frame: pitch
     *
     */
    PARAM_ADD_CORE(PARAM_FLOAT, simRotPitch, &simRotPitch)

    /**
     * @brief Simulated rotation of the local frame: roll
     *
     */
    PARAM_ADD_CORE(PARAM_FLOAT, simRotRoll, &simRotRoll)

    /**
     * @brief Simulated rotation of the local frame: yaw
     *
     */
    PARAM_ADD_CORE(PARAM_FLOAT, simRotYaw, &simRotYaw)

PARAM_GROUP_STOP(hil)