#ifndef __CORE_LIST_H__
#define __CORE_LIST_H__
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#ifndef NULL
    #define NULL ((void*)0)
#endif
#define LIST_POISON1 NULL
#define LIST_POISON2 NULL

#ifndef offsetof
    /**
     * @brief 计算结构体成员(member)相对于结构体(type)首地址的偏移量
     * @param   type    结构体类型
     * @param   member  结构体成员名称
     * @retval  MEMBER成员相对于结构体首地址的偏移量
     */
    #define offsetof(type, member) (size_t)(&((type*)0)->member)
#endif

// 根据member的地址获取type的起始地址
/**
 * @brief 根据结构体成员(member)的地址(ptr)获取结构体(type)起始地址
 * @param   ptr     结构体成员(member)的地址(指针)
 * @param   type    结构体类型
 * @param   member  结构体成员名称
 * @return  type*   结构体起始地址
 */
#define container_of(ptr, type, member)                                                                                          \
    ({                                                                                                                           \
        const typeof(((type*)0)->member)* __mptr = (ptr); /* 声明了一个临时的结构体成员指针__mptr */                             \
        (type*)((char*)__mptr - offsetof(type, member));                                                                         \
    }) /* 根据成员偏移量计算出结构体起始地址 */

// 链表结构
struct list_head
{
    struct list_head* prev;
    struct list_head* next;
};

/*初始化API*********************************************************/
/**
 * @name  LIST_HEAD / INIT_LIST_HEAD
 * @brief 表头初始化, 将前驱/后继指针都指向自己
 * @param name 链表头名称
 */
#define LIST_HEAD_INIT(name) {&(name), &(name)}
#define LIST_HEAD(name)      struct list_head name = LIST_HEAD_INIT(name)
static inline void INIT_LIST_HEAD(struct list_head* list)
{
    list->next = list;
    list->prev = list;
}
/*添加元素API*********************************************************/
static inline void __list_add(struct list_head* new_node, struct list_head* prev, struct list_head* next)
{
    new_node->prev = prev;
    prev->next     = new_node;
    new_node->next = next;
    next->prev     = new_node;
}

/**
 * @brief 将一个新节点插入到链表头部之后
 *
 * @param new_node   新节点
 * @param head       链表头
 */
static inline void list_add(struct list_head* new_node, struct list_head* head)
{
    __list_add(new_node, head, head->next);
}

/**
 * @brief 将一个新节点插入到链表尾部之前
 *
 * @param new_node    新节点
 * @param head        链表头
 */
static inline void list_add_tail(struct list_head* new_node, struct list_head* head)
{
    __list_add(new_node, head->prev, head);
}
/*删除元素API*********************************************************/
static inline void __list_del(struct list_head* prev, struct list_head* next)
{
    prev->next = next;
    next->prev = prev;
}
/**
 * @brief 删除一个节点
 * @param entry 要删除的节点
 */
static inline void list_del(struct list_head* entry)
{
    __list_del(entry->prev, entry->next);
    entry->next = entry;
    entry->prev = entry;
}

/*检测链表空API*********************************************************/
static inline uint8_t list_empty(const struct list_head* head)
{
    return (head->next == head);
}
/*两表合并API*********************************************************/
static inline void __list_splice(struct list_head* list, struct list_head* head)
{
    struct list_head* first = list->next;
    struct list_head* last  = list->prev;
    struct list_head* at    = head->next;
    first->prev             = head;
    head->next              = first;
    last->next              = at;
    at->prev                = last;
}

/**
 * @brief 两表合并
 *
 * @param list 被吞并的表
 * @param head 被嵌入的表的头节点
 * @note  嵌入位置为head与head->next之间
 */
static inline void list_splice(struct list_head* list, struct list_head* head)
{
    if (!list_empty(list))
        __list_splice(list, head);
}

/*替换节点API*********************************************************/
/**
 * @brief 节点替换
 *
 * @param old_node 被替换
 * @param new_node 新节点
 */
static inline void list_replace(struct list_head* old_node, struct list_head* new_node)
{
    new_node->next       = old_node->next;
    new_node->next->prev = new_node;
    new_node->prev       = old_node->prev;
    new_node->prev->next = new_node;
}

/**
 * @brief 在list_replace的基础上，对被替换的节点进行初始化
 *
 * @param old_node 被替换
 * @param new_node 新节点
 */
static inline void list_replace_init(struct list_head* old_node, struct list_head* new_node)
{
    list_replace(old_node, new_node);
    INIT_LIST_HEAD(old_node);
}
/*链表转移*********************************************************/
/**
 * @brief 将list从原表中删除，并插入到head之后
 * @param list 要转移节点
 * @param head 被插入节点
 */
static inline void list_move(struct list_head* list, struct list_head* head)
{
    __list_del(list->prev, list->next); // 将list从原表中删除
    list_add(list, head);               // 将list插入到head之后
}

/**
 * @brief 将list从原表中删除，并插入到head之前
 * @param list 要转移节点
 * @param head 被插入节点
 */
static inline void list_move_tail(struct list_head* list, struct list_head* head)
{
    __list_del(list->prev, list->next); // 将list从原表中删除
    list_add_tail(list, head);          // 将list插入到head之前
}

/**
 * @brief 链表元素转容器结构体
 * @param ptr      链表元素
 * @param type     结构体类型
 * @param member   结构体成员名称
 * @return type*   结构体起始地址
 */
#define list_entry(ptr, type, member) container_of(ptr, type, member)

/**
 * @brief 获取链表第一个元素对应的结构体地址
 * @param ptr      链表头
 * @param type     结构体类型
 * @return type*   结构体起始地址
 */
#define list_first_entry(ptr, type, member) list_entry((ptr)->next, type, member)
/*遍历API*******************************************************/
/* 遍历链表 */
/**
 * @brief 链表遍历
 * @param pos      遍历指针，由User定义并传入
 */
#define list_for_each(pos, head) for (pos = (head)->next; pos != (head); pos = pos->next)

/**
 * @brief 链表安全遍历
 * @param pos      遍历指针，由User定义并传入
 * @param n        临时指针，由User定义并传入
 * @note  n为pos的下一个节点，避免在遍历链表的过程中因pos节点被释放而造成的断链
 */
#define list_for_each_safe(pos, n, head) for (pos = (head)->next, n = pos->next; pos != (head); pos = n, n = pos->next)

/**
 * @brief 链表前向遍历
 * @param pos      遍历指针，由User定义并传入
 * @param head     链表头
 */
#define list_for_each_prev(pos, head) for (pos = (head)->prev; pos != (head); pos = pos->prev)

/**
 * @brief 通过链表，遍历容器
 * @param pos      遍历指针，由User定义并传入
 * @param head     链表头节点指针
 * @param member     容器类型
 * @note  pos为链表元素对应的结构体类型, 而不是链表类型
 */
#define list_for_each_entry(pos, head, member)                                                                                   \
    for (pos = list_entry((head)->next, typeof(*pos), member); &pos->member != (head);                                           \
         pos = list_entry(pos->member.next, typeof(*pos), member))

/**
 * @brief 通过链表，安全遍历容器
 * @param pos      遍历指针，由User定义并传入
 * @param n        临时指针，由User定义并传入
 * @param head     链表头
 * @param type     结构体类型
 * @note  pos为链表元素对应的结构体类型, 而不是链表类型
 */
#define list_for_each_entry_safe(pos, n, head, member)                                                                           \
    for (pos = list_entry((head)->next, typeof(*pos), member), n = list_entry(pos->member.next, typeof(*pos), member);           \
         &pos->member != (head); pos = n, n = list_entry(n->member.next, typeof(*n), member))

#ifdef __cplusplus
}
#endif
#endif
