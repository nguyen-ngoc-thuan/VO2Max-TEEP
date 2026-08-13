/*
 * status.h - Status screen interface
 * Copyright (C) 2025 TEEP Project
 * GPL V3
 */
#ifndef __STATUS_H__
#define __STATUS_H__

void initScreens(void);
void showScreen(void);
void changeScreen(bool forward);
void statusInitial(void);  // Initial warm-up screen

#endif
