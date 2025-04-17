#ifndef _SFIO_H
#define _SFIO_H

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "flash.h"

typedef enum
{
    SFIO_STREAM_SLOT,
    SFIO_STREAM_RAM,
}sfio_stream_type_t;

typedef struct {
    sfio_stream_type_t type;
    size_t offset;
    size_t size;
    union
	{
        uint8_t *ptr;	// RAM pointer for SFIO_STREAM_RAM
        uint32_t slot;   // Image slot for SFIO_STREAM_SLOT
    };
} sfio_stream_t;


size_t sfio_fread(void *ptr, size_t size, size_t count, sfio_stream_t *stream);

size_t sfio_fwrite(const void *ptr, size_t size, size_t count, sfio_stream_t *stream);

int sfio_fseek(sfio_stream_t *stream, long int offset, int origin);

#endif /* _SFIO_H */