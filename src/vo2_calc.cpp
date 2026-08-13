/*
 * vo2_calc.cpp - VO2 Calculation Engine
 *
 * Venturi flow, air density, ATPS→STPD conversion, VO2/VCO2/RQ
 *
 * References:
 *   - https://en.wikipedia.org/wiki/Venturi_effect
 *   - https://en.wikipedia.org/wiki/Density_of_air
 *   - https://sites.uni.edu/dolgener/Instrumentation/Repeating%20material/VO2%20Computation.pdf
 *   - Ulrich Rissel's BTPS/STPD notes
 *
 * Copyright (C) 2025 TEEP Project
 * GPL V3
 */

#include <math.h>
#include "vo2_calc.h"

// ---- Venturi constants ----
static float venturiConstant = 0.0;
static float venturiArea1 = 0.0;
static float venturiArea2 = 0.0;

// ---- Air density state ----
static float inverseRhoCoeff = sqrt(2.0 / 1.123);  // Default: 35°C, 95% humidity
static float VATPStoSTDP = 1.0;

// Water vapor pressure table (hPa) for temperatures 20°C to 40°C
// Used for BTPS calculation and ATPS→STPD conversion
static const float moistVaporPressure[21] = {
    23.33, 24.93, 26.40, 28.13, 29.86,   // 20-24°C
    31.73, 33.60, 35.60, 37.86, 40.00,   // 25-29°C
    42.40, 44.93, 47.60, 50.26, 53.20,   // 30-34°C
    56.26, 59.46, 62.79, 66.26, 69.86,   // 35-39°C
    73.73                                  // 40°C
};

// ---- Venturi Tube ----

void venturiInit(int outerDiameterMM, int innerDiameterMM) {
    float d1 = (float)outerDiameterMM / 1000.0;  // Convert to meters
    float d2 = (float)innerDiameterMM / 1000.0;

    venturiArea1 = 3.14159f * d1 * d1 * 0.25f;
    venturiArea2 = 3.14159f * d2 * d2 * 0.25f;

    // Q = venturiConstant * sqrt(2/rho) * sqrt(dP)
    // venturiConstant = A1 / sqrt((A1/A2)^2 - 1)
    float ratio = venturiArea1 / venturiArea2;
    venturiConstant = venturiArea1 / sqrt(ratio * ratio - 1.0f);
}

float venturiGetConstant(void) {
    return venturiConstant;
}

// ---- Air Density & Volume Compensation ----

void calcAirDensity(float ambientPressure, float exhaleTemp) {
    float temperature_kelvin = exhaleTemp + 273.15f;

    // Clamp temperature to vapor pressure table range (20-40°C)
    int tempIdx = (int)exhaleTemp;
    if (tempIdx < 20) tempIdx = 20;
    if (tempIdx > 40) tempIdx = 40;
    tempIdx -= 20;  // Index into moistVaporPressure array

    // ATPS to STPD conversion factor
    // V_STPD = V_ATPS * (273.15/T_K) * ((P_amb - P_water) / 1013.25)
    VATPStoSTDP = 273.15f / temperature_kelvin;
    VATPStoSTDP *= (ambientPressure - moistVaporPressure[tempIdx]) / 1013.25f;

    // Air density at exhale conditions (BTPS: body temp, ambient pressure, saturated)
    // rho = (P_dry * M_dry + P_vapor * M_water) / (R * T)
    // M_dry = 28.9652 g/mol → 2.89652 (scaled for hPa)
    // M_water = 18.016 g/mol → 1.8016 (scaled for hPa)
    // R = 8.31446 J/(mol·K)
    float rho = (ambientPressure * 2.89652f + moistVaporPressure[tempIdx] * 1.8016f)
                / (8.31446f * temperature_kelvin);

    // Sanity check
    if (rho < 0.6f) rho = 0.6f;
    else if (rho > 1.6f) rho = 1.6f;

    inverseRhoCoeff = sqrt(2.0f / rho);
}

float getInverseRhoCoeff(void) {
    return inverseRhoCoeff;
}

float getATPStoSTDP(void) {
    return VATPStoSTDP;
}

// ---- VO2 Calculation ----

float calcVO2(float veMean, float initialO2, float currentO2) {
    // VO2 = VE * (FiO2 - FeO2)
    // Without CO2 sensor, assume inspired N2 ≈ expired N2 (Haldane transformation simplified)
    float depletedO2 = (initialO2 - currentO2) * 0.01f;  // Convert % to fraction
    if (depletedO2 < 0.0f) depletedO2 = 0.0f;
    return veMean * depletedO2;  // L/min
}

float calcVO2withCO2(float veMean, float initialO2, float currentO2,
                     float initialCO2, float currentCO2, float *vco2Out) {
    // With CO2 sensor: use Haldane transformation
    // Expired N2 fraction
    float feO2 = currentO2 * 0.01f;
    float feCO2 = currentCO2 * 0.01f;
    float feN2 = 1.0f - feO2 - feCO2;

    // VO2 = VE * (FeN2 * FiO2/FiN2 - FeO2)
    // FiO2 ≈ initialO2/100, FiN2 ≈ 0.7903 (≈ 1 - 0.2093 - 0.0004)
    float fiO2 = initialO2 * 0.01f;
    float fiN2 = 1.0f - fiO2 - initialCO2 * 0.01f;

    float vo2 = veMean * (feN2 * fiO2 / fiN2 - feO2);
    if (vo2 < 0.0f) vo2 = 0.0f;

    // VCO2 = VE * (FeCO2 - FiCO2)
    if (vco2Out) {
        float fiCO2 = initialCO2 * 0.01f;
        *vco2Out = veMean * (feCO2 - fiCO2);
        if (*vco2Out < 0.0f) *vco2Out = 0.0f;
    }

    return vo2;
}

// ---- Calories ----

float calcCaloriesPerMin(float vo2TotalLPerMin) {
    // 1 liter O2 consumed ≈ 4.86 kcal (average, depends on RQ)
    return vo2TotalLPerMin * 4.86f;
}

// ---- Utility ----

float lowPassFilter(float newValue, float oldValue, float gain) {
    return gain * newValue + (1.0f - gain) * oldValue;
}
