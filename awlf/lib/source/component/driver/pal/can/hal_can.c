#include "component/driver/pal/pal_can_dev.h"
#include <string.h>

/* CAN报文容器 */
typedef struct CanMsgContainer* CanMsgContainer_t;
typedef struct CanMsgContainer
{
    CanMsgList_s listNode;
    uint8_t      container[8];
} __awlf_packed CanMsgContainer_s;

static void* can_msgbuffer_alloc(struct list_head* freeListHead, uint32_t msgNum)
{
    size_t size      = msgNum * sizeof(CanMsgContainer_s);
    void*  msgBuffer = pvPortMalloc(size);
    if (msgBuffer == NULL)
        return NULL;
    memset(msgBuffer, 0, size);

    // 将所有CAN消息容器节点添加到空闲列表
    CanMsgContainer_t MsgList = (CanMsgContainer_t)msgBuffer;
    for (size_t i = 0; i < msgNum; i++)
    {
        INIT_LIST_HEAD(&MsgList[i].listNode.fifoListNode);
        INIT_LIST_HEAD(&MsgList[i].listNode.matchedListNode);
        list_add(&MsgList[i].listNode.fifoListNode, freeListHead);
        MsgList[i].listNode.userMsg.userBuf = MsgList[i].listNode.container; // 指向容器内存
    }
    return msgBuffer;
}

static CanAdapterInterface_s CAN_AdapterInterface = {
    .msgbuffer_alloc = can_msgbuffer_alloc,
};

CanAdapterInterface_t hal_can_get_classic_adapter_interface(void)
{
    return &CAN_AdapterInterface;
}
