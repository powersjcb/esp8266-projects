extern "C" {
#include "user_interface.h"
}
uint32_t freeMemory()
{
return system_get_free_heap_size();
}
