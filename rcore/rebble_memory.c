/* rebble_memory.c
 * routines for Allocating memory for system and apps. 
 * App memory gets alocated ont he app's own heap where sys has a global heap
 * RebbleOS
 *
 * Author: Barry Carter <barry.carter@gmail.com>
 */

#include "rebbleos.h"

extern size_t xPortGetFreeAppHeapSize(void);
extern void *pvPortAppMalloc(size_t);
extern void vPortAppFree(void *);

bool _sanity_check_app(size_t size);

void *_app_malloc(size_t size);
void *_app_calloc(size_t count, size_t size);
void _app_free(void *mem);

void *_system_malloc(size_t size);
void *_system_calloc(size_t count, size_t size);
void _system_free(void *mem);


/*
 * Initialise the memory core
 */
void rcore_memory_init(void)
{

}

/*
 * The public facing API for allocating memory
 */
void *rcore_malloc(size_t size)
{
    if (appmanager_current_task_is_app())
    {
        return _app_malloc(size);
    }
    
    return _system_malloc(size);
}

/*
 * The public facing API for callocating memory
 */
void *rcore_calloc(size_t count, size_t size)
{
    if (appmanager_current_task_is_app())
    {
        return _app_calloc(count, size);
    }
    
    return _system_calloc(count, size);
}

/*
 * The public facing API for releasing memory
 */
void rcore_free(void *mem)
{
    if (appmanager_current_task_is_app())
    {
        _app_free(mem);
        return;
    }
    
    _system_free(mem);
}



// /*
//  * Alloc and set memory. Basic checks are done so we don't 
//  * hit an exception or ask for stoopid sizes
//  */
// void *rbl_calloc(size_t count, size_t size)
// {
//     if (size > xPortGetFreeHeapSize())
//     {
//         printf("Malloc fail. Not enough heap for %d\n", size);
//         while(1);
//         return NULL;
//     }
//     
//     if (size > 100000)
//     {
//         printf("Malloc will fail. > 100Kb requested\n");
//         return NULL;
//     }
//     
//     return pvPortCalloc(1, size);
// }

bool _sanity_check_app(size_t size)
{
    if (size == 0)
    {
        KERN_LOG("main", APP_LOG_LEVEL_ERROR, "Malloc fail. size=0. Huh?");
        return false;
    }
    
    if (size > xPortGetFreeAppHeapSize())
    {
        KERN_LOG("main", APP_LOG_LEVEL_DEBUG, "Malloc fail. Not enough heap for %d", size);
        return false;
        while(1); // TODO for debugging
    }
    
    if (size > 100000)
    {
        KERN_LOG("main", APP_LOG_LEVEL_DEBUG, "Malloc will fail. > 100Kb requested");
        return false;
    }
    
    return true;
}


/*
 * Application memory allocation
 */


void *_app_malloc(size_t size)
{
    if(!_sanity_check_app(size))
        return NULL;
    
    return (void*)pvPortAppMalloc(size);
    
}

void *_app_calloc(size_t count, size_t size)
{
    if(!_sanity_check_app(size))
        return NULL;
    
    void *x = (void*)pvPortAppMalloc(count * size);
    
    if (x != NULL)
        memset(x, 0, count * size);
    return x;
}


void _app_free(void *mem)
{
    vPortAppFree(mem);
}


/*
 * System memory allocation
 */


void *_system_malloc(size_t size)
{
    // call into FreeRTOS heap4
    void *x = pvPortMalloc(size);
    return x;
}

void *_system_calloc(size_t count, size_t size)
{
    // call into FreeRTOS heap4
    void *x = pvPortMalloc(count * size);
    if (x != NULL)
        memset(x, 0, count * size);
    return x;
}


void _system_free(void *mem)
{
    vPortFree(mem);
}
