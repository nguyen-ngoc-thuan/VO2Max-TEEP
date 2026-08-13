/*
 * files.h - Flash file storage interface
 * Copyright (C) 2025 TEEP Project, based on VO2Max-500pa by Lauri Peltonen
 * GPL V3
 */
#ifndef __FILES_H__
#define __FILES_H__

#include <stddef.h>
#include "config.h"

bool init_filesystem(void);
bool write_log_buffer_to_file(const char *buffer, size_t length, size_t block_size, unsigned int position);
bool open_data_file(void);
void close_data_file(void);
bool read_data_file(char *buffer, size_t bytes);

#endif
