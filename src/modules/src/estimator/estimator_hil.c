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

#include "debug.h"

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT (1.0 / ATTITUDE_UPDATE_RATE)

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT (1.0 / POS_UPDATE_RATE)


// Maybe change them to a struct later for clearer structure
static volatile float simPosX = 0.0;
static volatile float simPosY = 0.0;
static volatile float simPosZ = 0.0;

static volatile float simRotRoll = 0.0;
static volatile float simRotPitch = 0.0;
static volatile float simRotYaw = 0.0;

static volatile float simOmegaX = 0.0;
static volatile float simOmegaY = 0.0;
static volatile float simOmegaZ = 0.0;

static volatile float simVelocityX = 0.0;
static volatile float simVelocityY = 0.0;
static volatile float simVelocityZ = 0.0;

static volatile float simAccZ = 0.0;

static volatile float testparam = 0.0;

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
    DEBUG_PRINT("Hello From Hil\n");
    testparam = testparam + 1.0f;
    if (testparam > 1000.0f)
        testparam = 0.0;
    // Pull the latest sensors values of interest; discard the rest
    measurement_t m;
    while (estimatorDequeue(&m))
    {
        
    }

    // Update filter
    if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, stabilizerStep)) {
        // Euler angles 
        state->attitude.roll = simRotRoll;
        state->attitude.pitch = simRotPitch;
        state->attitude.yaw = simRotYaw;
        
        // angular velocity
        state->omega.x = simOmegaX;
        state->omega.y = simOmegaY;
        state->omega.z = simOmegaZ;

        // z velocity
        state->velocity.x = simVelocityX;
        state->velocity.y = simVelocityY;
        state->velocity.z = simVelocityZ;


        // z accelleration
        state->acc.z = simAccZ;
    }

    if (RATE_DO_EXECUTE(POS_UPDATE_RATE, stabilizerStep)) 
    {
        state->position.x = simPosX;
        state->position.y = simPosY;
        state->position.z = simPosZ;
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

    /**
     * @brief Simulated angular velocity for x axis
     * 
     */
    PARAM_ADD_CORE(PARAM_FLOAT, simOmegaX, &simOmegaX)

    /**
     * @brief Simulated angular velocity for y axis
     * 
     */
    PARAM_ADD_CORE(PARAM_FLOAT, simOmegaY, &simOmegaY)

    /**
     * @brief Simulated angular velocity for z axis
     * 
     */
    PARAM_ADD_CORE(PARAM_FLOAT, simOmegaZ, &simOmegaZ)

    /**
     * @brief Simulated Z velocity 
     * 
     */
    PARAM_ADD_CORE(PARAM_FLOAT, simVelocityX, &simVelocityX)


    PARAM_ADD_CORE(PARAM_FLOAT, simVelocityY, &simVelocityY)

    PARAM_ADD_CORE(PARAM_FLOAT, simVelocityZ, &simVelocityZ)
    /**
     * @brief Simulated Z accelleration 
     * 
     */
    PARAM_ADD_CORE(PARAM_FLOAT, simAccZ, &simAccZ) 

PARAM_GROUP_STOP(hil)

LOG_GROUP_START(hil)

    /**
     * @brief test parameter to see if the right estimator is used
     * 
     */
    LOG_ADD(LOG_FLOAT, testparam, &testparam)

LOG_GROUP_STOP(hil)