/*
 * bleTask.h - BLE task interface (HR belt reader + HR output + Golden Cheetah)
 * Copyright (C) 2025 TEEP Project
 * GPL V3
 */
#ifndef __BLETASK_H__
#define __BLETASK_H__

void BLETask(void *params);

// HR Belt reader (client)
void BLEHRConnect(void);
void BLEHRDisconnect(void);
void BLEHRScan(void);
bool BLEHRIsConnected(void);
const char *BLEHRGetName(void);

// Golden Cheetah VO2 Master (server)
void BLEGCStart(void);
void BLEGCStop(void);
bool BLEGCIsConnected(void);

// HR Output for Zwift/Strava (server)
void BLEHROutputStart(void);
void BLEHROutputStop(void);
void BLEHROutputUpdate(uint8_t bpm, uint16_t energyKJ);

#endif
