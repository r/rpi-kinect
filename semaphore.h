/*
 * some mutex related things -- init_MUTEX seems to have been
 * deprecated
 */
#ifndef init_MUTEX
#define init_MUTEX(sem) sema_init(sem, 1)
#endif
