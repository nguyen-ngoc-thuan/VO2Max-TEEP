/*
 * buttonsTask.h - Button handling interface
 * Copyright (C) 2025 TEEP Project, based on VO2Max-500pa by Lauri Peltonen
 * GPL V3
 */
#ifndef __BUTTONSTASK_H__
#define __BUTTONSTASK_H__

void buttonInit(void);
void IRAM_ATTR buttonInterrupt(void);
bool isButtonPressed(uint8_t which);
bool waitButtonPress(uint8_t *which, TickType_t timeout);
void changeScreen(bool forward);

#endif
