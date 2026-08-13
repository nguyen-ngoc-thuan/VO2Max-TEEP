/*
 * vo2_calc.h - VO2 Calculation Engine
 *
 * Copyright (C) 2025 TEEP Project
 * GPL V3
 */

#ifndef __VO2_CALC_H__
#define __VO2_CALC_H__

// ---- Venturi Tube ----

// Recalculate venturi constants for given diameters (mm)
void venturiInit(int outerDiameterMM, int innerDiameterMM);

// Get venturi constant (precomputed A1/sqrt((A1/A2)^2 - 1))
float venturiGetConstant(void);

// ---- Air Density & Volume Compensation ----

// Calculate air density and ATPS→STPD coefficient
// ambientPressure in hPa, exhaleTemp in °C
void calcAirDensity(float ambientPressure, float exhaleTemp);

// Get the inverse rho coefficient: sqrt(2/rho_exhale)
float getInverseRhoCoeff(void);

// Get the ATPS to STPD conversion factor
float getATPStoSTDP(void);

// ---- VO2 Calculation ----

// Calculate VO2 from VE and O2 readings (no CO2 sensor)
// veMean in L/min, initialO2 and currentO2 in %
// Returns VO2 in L/min
float calcVO2(float veMean, float initialO2, float currentO2);

// Calculate VO2 and VCO2 with CO2 sensor data
// Returns VO2 in L/min, vco2Out receives VCO2 in L/min
float calcVO2withCO2(float veMean, float initialO2, float currentO2,
                     float initialCO2, float currentCO2, float *vco2Out);

// ---- Calories ----

// Calculate calories per minute from VO2 total (L/min)
float calcCaloriesPerMin(float vo2TotalLPerMin);

// ---- Utility ----

// Exponential low-pass filter
float lowPassFilter(float newValue, float oldValue, float gain);

#endif // __VO2_CALC_H__
