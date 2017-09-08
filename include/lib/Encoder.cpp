#include "Encoder.h"
Encoder::Encoder() {

}

Encoder::Encoder(short deviceNum = 0, short enNum = 0, long cpr = 499, short _quadMode = 1) :
DeviceNum(deviceNum), EnNum(enNum), CPR(cpr), QuadMode(_quadMode) {
    float pi = 3.141592;
    printf("Initialize Encoder#%d for device#%d with CPR = %d\n", EnNum, DeviceNum, CPR);
    AngleDeg = 0.0;
    AngleRad = 0.0;
    Count = 0;
    if (QuadMode == QUAD_X4)
        MaxCount = CPR * (QuadMode + 1);
    else
        MaxCount = CPR * QuadMode;
    Count2Deg = 360.0 / MaxCount;
    Count2Rad = 2 * pi / MaxCount;
    init();

}

Encoder::~Encoder() {
}

void Encoder::updateEncoder() {
    unsigned tmp;
    USB4_GetCount(DeviceNum, EnNum, &tmp);
    AdjustCount(tmp);
    AngleDeg = Count * Count2Deg;
    AngleRad = Count * Count2Rad;
}

void Encoder::AdjustCount(unsigned tmp) {
    Count = (long) tmp;
    if (Count > (MaxCount / 2))
        Count = -(MaxCount - Count);
}

float Encoder::getAngleDeg() {
    updateEncoder();
    return AngleDeg;
}

float Encoder::getAngleRad() {
    updateEncoder();
    return AngleRad;
}

long Encoder::getCount() {
    updateEncoder();
    return Count;
}

float Encoder::getResDeg() const {
    return Count2Deg;
}

float Encoder::getResRad() const {
    return Count2Rad;
}

long Encoder::getMaxCount() const {
    return MaxCount;
}

float Encoder::getRPM() {
    float tmp;
    USB4_GetRPM(DeviceNum, EnNum, & tmp);
    RPM = double(tmp);
    return RPM;
}

void Encoder::Reset() {
    printf("Rest count for Encoder#%d\n", EnNum);
    USB4_ResetCount(DeviceNum, EnNum); // Reset the counter to 0
    Count = 0;
    AngleDeg = 0.0;
    AngleRad = 0.0;
}

void Encoder::init() const {
    USB4_SetPresetValue(DeviceNum, EnNum, MaxCount - 1);    // Set the preset register to the CPR-1
    USB4_SetMultiplier(DeviceNum, EnNum, QUAD_X4);          // Set quadrature mode to x1, x2 or X4.
    USB4_SetCounterMode(DeviceNum, EnNum, 3);               // Set counter mode to modulo-N.
    USB4_SetForward(DeviceNum, EnNum, true);                // Determines the direction of counting.
    USB4_SetCounterEnabled(DeviceNum, EnNum, true);         // Enable the counter
    USB4_ResetCount(DeviceNum, EnNum);                      // Reset the counter to 0
}

