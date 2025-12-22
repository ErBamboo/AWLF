#include "component/driver/pal/pal_can_dev.h"
#include <string.h>

/* CANFd报文容器 */
typedef struct CanFdMsgContainer* CanFdMsgContainer_t;
typedef struct CanFdMsgContainer
{
    CanMsgList_s listNode;
    uint8_t      container[64];
} __awlf_packed CanFdMsgContainer_s;

static void* canfd_msgbuffer_alloc(struct list_head* freeListHead, uint32_t msgNum)
{
    size_t size      = msgNum * sizeof(CanFdMsgContainer_s);
    void*  msgBuffer = pvPortMalloc(size);
    if (msgBuffer == NULL)
        return NULL;
    memset(msgBuffer, 0, size);

    // 将所有CAN FD消息容器节点添加到空闲列表
    CanFdMsgContainer_t MsgList = (CanFdMsgContainer_t)msgBuffer;
    for (size_t i = 0; i < msgNum; i++)
    {
        INIT_LIST_HEAD(&MsgList[i].listNode.fifoListNode);
        INIT_LIST_HEAD(&MsgList[i].listNode.matchedListNode);
        list_add(&MsgList[i].listNode.fifoListNode, freeListHead);
        MsgList[i].listNode.userMsg.userBuf = MsgList[i].listNode.container; // 指向容器内存
    }
    return msgBuffer;
}

static CanAdapterInterface_s CanFd_AdapterInterface = {
    .msgbuffer_alloc = canfd_msgbuffer_alloc,
};

CanAdapterInterface_t hal_can_get_canfd_adapter_interface(void)
{
    return &CanFd_AdapterInterface;
}