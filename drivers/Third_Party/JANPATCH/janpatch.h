#ifndef _JANPATCH_H
#define _JANPATCH_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <simple_fileio.h>
#include <ring_buffer.h>

#define JANPATCH_STREAM sfio_stream_t

#ifndef JANPATCH_DEBUG
#define JANPATCH_DEBUG(...)  while (0) {} // printf(__VA_ARGS__)
#endif

#ifndef JANPATCH_ERROR
#define JANPATCH_ERROR(...)  printf(__VA_ARGS__)
#endif

// detect POSIX, and use FILE* in that case
#if !defined(JANPATCH_STREAM) && (defined (__unix__) || (defined (__APPLE__) && defined (__MACH__)))
#include <stdio.h>
#define JANPATCH_STREAM     FILE
#elif !defined(JANPATCH_STREAM)
#error "JANPATCH_STREAM not defined, and not on POSIX system. Please specify the JANPATCH_STREAM macro"
#endif

typedef struct {
    unsigned char*   buffer;
    size_t           size;
    uint32_t         current_page;
    size_t           current_page_size;
    JANPATCH_STREAM* stream;
    long int         position;
} janpatch_buffer;

typedef struct {
    // fread/fwrite buffers
    janpatch_buffer source_buffer;
    janpatch_buffer patch_buffer;
    janpatch_buffer target_buffer;

    // function signatures
    size_t (*fread)(void*, size_t, size_t, JANPATCH_STREAM*);
    size_t (*fwrite)(const void*, size_t, size_t, JANPATCH_STREAM*);
    int    (*fseek)(JANPATCH_STREAM*, long int, int);
    long   (*ftell)(JANPATCH_STREAM*);

    // progress callback
    void   (*progress)(uint8_t);

    // the combination of the size of both the source + patch files (that's the max. the target file can be)
    long   max_file_size;
}janpatch_ctx;

enum {
    JANPATCH_OPERATION_ESC = 0xa7,
    JANPATCH_OPERATION_MOD = 0xa6,
    JANPATCH_OPERATION_INS = 0xa5,
    JANPATCH_OPERATION_DEL = 0xa4,
    JANPATCH_OPERATION_EQL = 0xa3,
    JANPATCH_OPERATION_BKT = 0xa2
};



int janpatch(janpatch_ctx ctx, JANPATCH_STREAM *source, JANPATCH_STREAM *patch, JANPATCH_STREAM *target);

#endif // _JANPATCH_H
