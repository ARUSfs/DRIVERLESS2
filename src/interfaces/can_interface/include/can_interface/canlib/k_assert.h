#ifndef _KASSERT_H
#define _KASSERT_H

// Kernel style assert without crashing

#ifdef NDEBUG

/**** No debug *****/
#define K_ASSERT(cond)

#else // NDEBUG

/**** Debug *****/
#ifdef __KERNEL__

/**** Linux kernel *****/
#define K_ASSERT(cond)                                                  \
do {                                                                    \
        if (unlikely(!(cond))) {                                        \
            pr_emerg("### K_ASSERT caught in %s(...) at %s:%d : %s\n",  \
                     __FUNCTION__, __FILE__, __LINE__, #cond);          \
            dump_stack();                                               \
        }                                                               \
} while (0)
#else // __KERNEL__

/**** Linux userspace *****/
#include <assert.h>
#define K_ASSERT(cond) assert(cond)

#endif /* __KERNEL__ */
#endif // NDEBUG
#endif // _KASSERT_H

