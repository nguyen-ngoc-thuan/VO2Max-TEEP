/*
 * buttonsTask.cpp - Button handling with interrupt-based debounce
 * Copyright (C) 2025 TEEP Project, based on VO2Max-500pa by Lauri Peltonen
 * GPL V3
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp32-hal-gpio.h>

#include "config.h"
#include "buttonsTask.h"

static EventGroupHandle_t buttonStatus;

#define BUTTON_1_PRESSED        0x0001
#define BUTTON_1_RELEASED       0x0004
#define BUTTON_2_PRESSED        0x0010
#define BUTTON_2_RELEASED       0x0040

void buttonInit(void) {
    buttonStatus = xEventGroupCreate();
    configASSERT(buttonStatus);
    xEventGroupSetBits(buttonStatus, BUTTON_1_RELEASED | BUTTON_2_RELEASED);

    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_2_PIN, INPUT_PULLUP);
}

void IRAM_ATTR buttonInterrupt(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (digitalRead(BUTTON_1_PIN)) {
        xEventGroupClearBitsFromISR(buttonStatus, BUTTON_1_PRESSED);
        xEventGroupSetBitsFromISR(buttonStatus, BUTTON_1_RELEASED, &xHigherPriorityTaskWoken);
    } else {
        xEventGroupClearBitsFromISR(buttonStatus, BUTTON_1_RELEASED);
        xEventGroupSetBitsFromISR(buttonStatus, BUTTON_1_PRESSED, &xHigherPriorityTaskWoken);
    }

    if (digitalRead(BUTTON_2_PIN)) {
        xEventGroupClearBitsFromISR(buttonStatus, BUTTON_2_PRESSED);
        xEventGroupSetBitsFromISR(buttonStatus, BUTTON_2_RELEASED, &xHigherPriorityTaskWoken);
    } else {
        xEventGroupClearBitsFromISR(buttonStatus, BUTTON_2_RELEASED);
        xEventGroupSetBitsFromISR(buttonStatus, BUTTON_2_PRESSED, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

bool isButtonPressed(uint8_t which) {
    if (which == BUTTON_LOWER)
        return (xEventGroupGetBits(buttonStatus) & BUTTON_1_PRESSED) != 0;
    if (which == BUTTON_UPPER)
        return (xEventGroupGetBits(buttonStatus) & BUTTON_2_PRESSED) != 0;
    return false;
}

// Wait for a button press, returns which button was pressed
// Returns true if a button was pressed, false on timeout
bool waitButtonPress(uint8_t *which, TickType_t timeout) {
    EventBits_t bits = xEventGroupWaitBits(buttonStatus,
        BUTTON_1_PRESSED | BUTTON_2_PRESSED,
        pdFALSE, pdFALSE, timeout);

    *which = 0;
    if (bits & BUTTON_1_PRESSED) {
        *which = BUTTON_LOWER;
        // Wait for release with debounce
        vTaskDelay(pdMS_TO_TICKS(50));
        xEventGroupWaitBits(buttonStatus, BUTTON_1_RELEASED, pdFALSE, pdFALSE, pdMS_TO_TICKS(2000));
        vTaskDelay(pdMS_TO_TICKS(50));
        return true;
    }
    if (bits & BUTTON_2_PRESSED) {
        *which = BUTTON_UPPER;
        vTaskDelay(pdMS_TO_TICKS(50));
        xEventGroupWaitBits(buttonStatus, BUTTON_2_RELEASED, pdFALSE, pdFALSE, pdMS_TO_TICKS(2000));
        vTaskDelay(pdMS_TO_TICKS(50));
        return true;
    }
    return false;
}
