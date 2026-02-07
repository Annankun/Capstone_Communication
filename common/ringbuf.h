/*
 * ringbuf.h - Lock-free single-producer single-consumer ring buffer
 *
 * Designed for ISR→main-loop flow:
 *   - ISR calls ringbuf_push()   (producer)
 *   - main loop calls ringbuf_pop() (consumer)
 *
 * Size MUST be a power of 2.
 */
#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdint.h>

#define RINGBUF_SIZE  256u   /* must be power of 2 */
#define RINGBUF_MASK  (RINGBUF_SIZE - 1u)

typedef struct {
    volatile uint8_t  buf[RINGBUF_SIZE];
    volatile uint32_t head;   /* written by producer (ISR) */
    volatile uint32_t tail;   /* read by consumer (main)   */
    volatile uint32_t overflow_count;
} ringbuf_t;

static inline void ringbuf_init(ringbuf_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->overflow_count = 0;
}

/* Returns number of bytes available to read */
static inline uint32_t ringbuf_count(const ringbuf_t *rb)
{
    return (rb->head - rb->tail) & RINGBUF_MASK;
}

static inline int ringbuf_is_empty(const ringbuf_t *rb)
{
    return rb->head == rb->tail;
}

static inline int ringbuf_is_full(const ringbuf_t *rb)
{
    return ((rb->head + 1) & RINGBUF_MASK) == rb->tail;
}

/* Push one byte (call from ISR). Returns 0 on success, -1 on overflow. */
static inline int ringbuf_push(ringbuf_t *rb, uint8_t byte)
{
    uint32_t next = (rb->head + 1) & RINGBUF_MASK;
    if (next == rb->tail) {
        rb->overflow_count++;
        return -1;   /* full */
    }
    rb->buf[rb->head] = byte;
    rb->head = next;
    return 0;
}

/* Pop one byte (call from main loop). Returns 0 on success, -1 if empty. */
static inline int ringbuf_pop(ringbuf_t *rb, uint8_t *out)
{
    if (rb->head == rb->tail)
        return -1;   /* empty */
    *out = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) & RINGBUF_MASK;
    return 0;
}

#endif /* RINGBUF_H */
