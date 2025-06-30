#include <assert.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "ringbuffer.h"

#define is_num_power_of_two(x) ((x) && !(((x) & ((x) - 1))))

inline static unsigned int roundup_pow_of_two(unsigned int v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

// item count in buf
inline unsigned int ringbuf_len(const ringbuf_t rb)
{
	return rb->in - rb->out;
}

// max item count in buf
inline unsigned int ringbuf_cap(const ringbuf_t rb) 
{
	return rb->mask + 1;
}

// avail item count
inline unsigned int ringbuf_avail(const ringbuf_t rb) 
{
	return ringbuf_cap(rb) - ringbuf_len(rb);
}

inline bool ringbuf_is_full(const ringbuf_t rb) 
{
	return ringbuf_len(rb) > rb->mask;
}

inline bool ringbuf_is_empty(const ringbuf_t rb) 
{
	return rb->in == rb->out;
}

inline void ringbuf_update_out(ringbuf_t rb, unsigned int count)
{
    rb->out += count;
}

inline void ringbuf_update_in(ringbuf_t rb, unsigned int count)
{
    rb->in += count;
}

bool ringbuf_init(ringbuf_t rb, uint8_t* buff, unsigned int item_size, unsigned int item_count)
{
	if(!buff) return false;
	if(!is_num_power_of_two(item_count)) 
	{
		//@todo : log
		while(1){};
	}
	rb->buf = buff;
	rb->esize = item_size;
	rb->mask = item_count - 1;
	rb->in = rb->out = 0;
	return true;
}

bool ringbuf_alloc(ringbuf_t rb, unsigned int item_size, unsigned int item_count)
{
	unsigned int buf_size = roundup_pow_of_two(item_count);
	rb->buf = malloc(buf_size * item_size);
	if (NULL == rb->buf) {
		return false;
	}
	rb->esize = item_size;
	rb->mask = buf_size - 1;
	rb->in = rb->out = 0;
	return true;
}

void free_ringbuf(ringbuf_t rb)
{
	if (rb->buf)
	{
		free(rb->buf);
		rb->buf = NULL;
	}
}

static void inline ringbuf_copy_in(ringbuf_t rb, const void* src, unsigned int len, unsigned int off) 
{
	unsigned int size = rb->mask + 1;
	unsigned int esize = rb->esize;
	unsigned int l;

	off &= rb->mask;
	if (esize != 1) {
		off *= esize;
		size *= esize;
		len *= esize;
	}
	l = len < (size - off) ? len : (size - off);

	memcpy(rb->buf + off, src, l);
	memcpy(rb->buf, (const unsigned char*)src + l, len - l);

	/*
	 * make sure that the data in the fifo is up to date before
	 * incrementing the fifo->in index counter
	 */
	smp_wmb();
}

unsigned int ringbuf_in(ringbuf_t rb, const void* buf, unsigned int item_count) 
{
	unsigned int avail = ringbuf_avail(rb);
	if (item_count > avail)
		item_count = avail;

	ringbuf_copy_in(rb, buf, item_count, rb->in);

	rb->in += item_count;
	return item_count;
}

static void inline ringbuf_copy_out(ringbuf_t rb, void* dst, unsigned int len, unsigned int off) 
{
	unsigned int size = rb->mask + 1;
	unsigned int esize = rb->esize;
	unsigned int l;

	off &= rb->mask;
	if (esize != 1) {
		off *= esize;
		size *= esize;
		len *= esize;
	}
	l = len < (size - off) ? len : (size - off);

	memcpy(dst, rb->buf + off, l);
	memcpy((unsigned char*)dst + l, rb->buf, len - l);
	/*
	 * make sure that the data is copied before
	 * incrementing the fifo->out index counter
	 */
	smp_wmb();
}

unsigned int ringbuf_out(ringbuf_t rb, void* buf, unsigned int item_count) 
{
	item_count = ringbuf_out_peek(rb, buf, item_count);
	rb->out += item_count;
	return item_count;
}



unsigned int ringbuf_out_peek(ringbuf_t rb, void* buf, unsigned int len) 
{
	unsigned int l;
	l = rb->in - rb->out;
	if (len > l) {
		len = l;
	}
	ringbuf_copy_out(rb, buf, len, rb->out);
	return len;
}

unsigned int ringbuf_get_linear_space(ringbuf_t rb, void** dest)
{
	unsigned int out_off;
	unsigned int in_off;
	unsigned int linear_size;
	if (ringbuf_is_empty(rb)) return 0;

	in_off = rb->in & rb->mask;
	out_off = rb->out & rb->mask;		// 获取写指针偏移量
	*dest = &rb->buf[out_off * rb->esize];

	if (out_off < in_off)
		linear_size = in_off - out_off;
	if (out_off > in_off || ringbuf_is_full(rb))
		linear_size = ringbuf_cap(rb) - out_off;

	return linear_size;
}
