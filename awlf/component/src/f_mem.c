#include "../include/f_mem.h"

void* f_alloc(size_t size)
{
    return malloc(size);
}

void f_free(void* ptr)
{
    if(ptr == NULL) 
    {
        //@todo: log   
        return;
    }
    free(ptr);
    ptr = NULL;
}
