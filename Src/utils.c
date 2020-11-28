
#include "foc.h"
#include "utils.h"

extern float Position_Phase_Degree,Position_Degree,Speed,iq;
extern float angle[3];
extern float current[2];
extern float ialpha,ibeta;
Wave_Group wg1={
    0,
    &Position_Degree,
    &Position_Phase_Degree,
    &Speed,
    &iq
};

Wave_Group wg2={
    1,
    current,
    current+1,
    &ialpha,
    &ibeta
};

