#pragma once

#include "stabilizer_types.h"

void estimatorHILInit(void);
bool estimatorHILTest(void);
void estimatorHIL(state_t *state, const stabilizerStep_t stabilizerStep);

bool estimatorHILEnqueueTOF(const tofMeasurement_t *tof);