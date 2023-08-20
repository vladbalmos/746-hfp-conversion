#include <stdio.h>

#ifdef DEBUG_MODE
    #define DEBUG(...) printf(__VA_ARGS__);
#else
    #define DEBUG(...) do {} while (0)
#endif

inline bool debug_mode() {
#ifdef DEBUG_MODE
    return true;
#else
    return false;
#endif
}