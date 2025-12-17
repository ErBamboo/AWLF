#include "data_struct/ringbuffer.h"
#include <assert.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#define is_num_power_of_two(x) ((x) && !(((x) & ((x) - 1))))

bool ringbuf_init(Ringbuf_t rb, uint8_t* buff, unsigned int item_size, unsigned int item_count)
{
    if (!buff)
        return false;
    if (!is_num_power_of_two(item_count))
    {
        while (1) {}; // TODO : assert
    }
    rb->buf   = buff;
    rb->esize = item_size;
    rb->mask  = item_count - 1;
    rb->in = rb->out = 0;
    return true;
}

bool ringbuf_alloc(Ringbuf_t rb, unsigned int item_size, unsigned int item_count, void*(pmalloc)(size_t))
{
    unsigned int item_buf_size = roundup_pow_of_two(item_count);
    if (!pmalloc)
        rb->buf = malloc(item_buf_size * item_size);
    else
        rb->buf = pmalloc(item_buf_size * item_size);
    if (NULL == rb->buf)
    {
        return false;
    }
    memset(rb->buf, 0, item_buf_size * item_size);
    rb->esize = item_size;
    rb->mask  = item_buf_size - 1;
    rb->in = rb->out = 0;
    return true;
}

void free_ringbuf(Ringbuf_t rb, void (*pfree)(void*))
{
    if (rb->buf)
    {
        if (!pfree)
            free(rb->buf);
        else
            pfree(rb->buf);
        rb->buf = NULL;
    }
}

static void inline ringbuf_copy_in(Ringbuf_t rb, const void* src, unsigned int len, unsigned int off)
{
    unsigned int size  = rb->mask + 1;
    unsigned int esize = rb->esize;
    unsigned int l;

    off &= rb->mask;
    if (esize != 1)
    {
        off *= esize;
        size *= esize;
        len *= esize;
    }
    l = len < (size - off) ? len : (size - off);

    memcpy(rb->buf + off, src, l);
    memcpy(rb->buf, (const unsigned char*)src + l, len - l);
}

unsigned int ringbuf_in(Ringbuf_t rb, const void* buf, unsigned int item_count)
{
    unsigned int avail = ringbuf_avail(rb);
    if (item_count > avail)
        item_count = avail;

    ringbuf_copy_in(rb, buf, item_count, rb->in);

    rb->in += item_count;
    return item_count;
}

static void inline ringbuf_copy_out(Ringbuf_t rb, void* dst, unsigned int len, unsigned int off)
{
    unsigned int size  = rb->mask + 1;
    unsigned int esize = rb->esize;
    unsigned int l;

    off &= rb->mask;
    if (esize != 1)
    {
        off *= esize;
        size *= esize;
        len *= esize;
    }
    l = len < (size - off) ? len : (size - off);

    memcpy(dst, rb->buf + off, l);
    memcpy((unsigned char*)dst + l, rb->buf, len - l);
}

unsigned int ringbuf_out(Ringbuf_t rb, void* buf, unsigned int item_count)
{
    item_count = ringbuf_out_peek(rb, buf, item_count);
    rb->out += item_count;
    return item_count;
}

unsigned int ringbuf_out_peek(Ringbuf_t rb, void* buf, unsigned int len)
{
    unsigned int l;
    l = rb->in - rb->out;
    if (len > l)
    {
        len = l;
    }
    ringbuf_copy_out(rb, buf, len, rb->out);
    return len;
}

unsigned int ringbuf_get_item_linear_space(Ringbuf_t rb, void** dest)
{
    unsigned int out_off;
    unsigned int in_off;
    unsigned int linear_size;
    if (ringbuf_is_empty(rb))
    {
        *dest = NULL;
        return 0;
    }

    in_off  = rb->in & rb->mask;
    out_off = rb->out & rb->mask; // 获取写指针偏移量
    *dest   = &rb->buf[out_off];

    if (out_off < in_off)
        linear_size = in_off - out_off;
    else if (out_off > in_off || ringbuf_is_full(rb))
        linear_size = ringbuf_cap(rb) - out_off;

    return linear_size;
}

unsigned int ringbuf_get_avail_linear_size(Ringbuf_t rb, void** dest)
{
    unsigned int out_off;
    unsigned int in_off;
    unsigned int linear_size;

    if (ringbuf_is_full(rb))
    {
        *dest = NULL;
        return 0;
    }
    in_off  = rb->in & rb->mask;
    out_off = rb->out & rb->mask;
    *dest   = &rb->buf[in_off];
    if (out_off < in_off)
        linear_size = ringbuf_cap(rb) - in_off;
    else if (out_off > in_off)
        linear_size = out_off - in_off;
    else
        linear_size = ringbuf_cap(rb);
    return linear_size;
}
