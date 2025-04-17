#include "fileio.h"

int firm_read(uint32_t addr, void *ptr, long int offset, size_t count);
int firm_write(uint32_t addr, const void *ptr, long int offset, size_t count);

size_t sfio_fread(void *ptr, size_t size, size_t count, sfio_stream_t *stream)
{
    assert(size == 1);
    if (stream->offset + count > stream->size)
    {
        count = stream->size - stream->offset;
    }
    if (stream->type == SFIO_STREAM_SLOT)
    {
        firm_read(stream->slot, ptr, stream->offset, size * count);
    }
    else
    {
        memcpy(ptr, stream->ptr + stream->offset, ptr, size * count);
    }

    return count * size;
}

size_t sfio_fwrite(const void *ptr, size_t size, size_t count, sfio_stream_t *stream)
{
    assert(size == 1);
    if (stream->offset + count > stream->size)
    {
        count = stream->size - stream->offset;
    }
    if (stream->type == SFIO_STREAM_SLOT)
    {
        firm_write(stream->slot, ptr, stream->offset, size * count);
    }
    else
    {
        memcpy(stream->ptr + stream->offset, ptr, size * count);
    }

    return count * size;
}

int sfio_fseek(sfio_stream_t *stream, long int offset, int origin)
{
    assert(origin == SEEK_SET);
    if (offset > stream->size)
    {
        return -1;
    } else {
        stream->offset = offset;
    }
    return 0;
}

int firm_read(uint32_t addr, void *ptr, long int offset, size_t count)
{
    flash_read(addr + offset, ptr, count);
    return count;
}

int firm_write(uint32_t addr, const void *ptr, long int offset, size_t count)
{
    if (flash_write(addr + offset, (const uint8_t*)ptr, count)) {
        return count;
    }
    return 0;
}