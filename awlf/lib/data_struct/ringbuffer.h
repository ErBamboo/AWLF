/**
 * @brief 单读单写的无锁环形队列
 * @note 无覆写策略，当剩余空间小于尝试写入数据时，真实写入大小为剩余空间大小
 * @note 本队列在同一时刻，可以有一个生产者+一个消费者访问，其余情况必须外部加锁保护以防止竞态
 */

#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdatomic.h>

#ifdef __cplusplus
extern "C"
{
#endif

// cpu级 内存屏障
#define smp_rmb()
#define smp_wmb()

typedef struct Ringbuf* Ringbuf_t;
typedef struct Ringbuf
{
    unsigned char*        buf;   // 数据缓冲区
    volatile unsigned int in;    // 写指针
    volatile unsigned int out;   // 读指针
    unsigned int          mask;  // 2^n - 1
    unsigned int          esize; // 容量
} Ringbuf_s;

bool         ringbuf_alloc(Ringbuf_t rb, unsigned int item_size, unsigned int item_count, void*(pmalloc)(size_t));
bool         ringbuf_init(Ringbuf_t rb, uint8_t* buff, unsigned int item_size, unsigned int item_count);
void         free_ringbuf(Ringbuf_t rb, void (*pfree)(void*));
unsigned int ringbuf_in(Ringbuf_t rb, const void* buf, unsigned int item_count);
unsigned int ringbuf_out(Ringbuf_t rb, void* buf, unsigned int item_count);
unsigned int ringbuf_out_peek(Ringbuf_t rb, void* buf, unsigned int len);
unsigned int ringbuf_get_item_linear_space(Ringbuf_t rb, void** dest);
unsigned int ringbuf_get_avail_linear_size(Ringbuf_t rb, void** dest);

// item count in buf
static inline unsigned int ringbuf_len(const Ringbuf_t rb)
{
    return rb->in - rb->out;
}

// max item count in buf
static inline unsigned int ringbuf_cap(const Ringbuf_t rb)
{
    return (rb->mask + 1);
}

// avail item count
static inline unsigned int ringbuf_avail(const Ringbuf_t rb)
{
    return ringbuf_cap(rb) - ringbuf_len(rb);
}

static inline bool ringbuf_is_full(const Ringbuf_t rb)
{
    return ringbuf_len(rb) > rb->mask;
}

static inline bool ringbuf_is_empty(const Ringbuf_t rb)
{
    return rb->in == rb->out;
}

static inline void ringbuf_update_out(Ringbuf_t rb, unsigned int count)
{
    unsigned int l = rb->in - rb->out;
    if (count > l)
        count = l;
    rb->out += count;
}

static inline void ringbuf_update_in(Ringbuf_t rb, unsigned int count)
{
    unsigned int avail = ringbuf_avail(rb);
    if (count > avail)
        count = avail;
    rb->in += count;
}

static inline unsigned int roundup_pow_of_two(unsigned int v)
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

#ifdef __cplusplus
} // extern "C"
#endif

#endif // RINGBUF_H
