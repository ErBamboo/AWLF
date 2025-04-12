#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdbool.h>
#include <stdatomic.h>

#ifdef  __GNUC__
 #define smp_wmb()  //__sync_synchronize        // 多核处理器(SMP)
#endif

// 本队列采取无覆写策略，保全数据完整性，当剩余空间小于尝试写入数据时，真实写入大小为剩余空间大小

// 对于同一个无锁队列，如果只有一个线程操作、或者只有一个线程读，一个线程写，那么使用无锁队列是安全的。
// 注意！若尝试重写此文件，务必了解无锁队列与普通队列的区别

typedef struct ringbuf* ringbuf_t;
typedef struct ringbuf {
    unsigned char* buf;     // 数据缓冲区
    unsigned int in;        // 写指针
    unsigned int out;       // 读指针
    unsigned int mask;      // 2^n - 1
    unsigned int esize;     // 容量
}ringbuf_s;


#ifdef __cplusplus
extern "C" {
#endif



bool ringbuf_alloc(ringbuf_t rb, unsigned int item_size, unsigned int item_count);
bool ringbuf_init(ringbuf_t rb, uint8_t* buff, unsigned int item_size, unsigned int item_count);
void free_ringbuf(ringbuf_t rb);
unsigned int ringbuf_in(ringbuf_t rb, const void* buf, unsigned int item_count);
unsigned int ringbuf_out(ringbuf_t rb, void* buf, unsigned int item_count);
unsigned int ringbuf_out_peek(ringbuf_t rb, void* buf, unsigned int len);
unsigned int ringbuf_get_linear_space(ringbuf_t rb, void** dest);

unsigned int ringbuf_len(const ringbuf_t rb);
unsigned int ringbuf_cap(const ringbuf_t rb);
unsigned int ringbuf_avail(const ringbuf_t rb);
bool ringbuf_is_full(const ringbuf_t rb);
bool ringbuf_is_empty(const ringbuf_t rb);
void ringbuf_update_out(ringbuf_t rb, unsigned int count);
void ringbuf_update_in(ringbuf_t rb, unsigned int count);


#ifdef __cplusplus
} // extern "C"
#endif

#endif // RINGBUF_H
