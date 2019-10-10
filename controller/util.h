#ifndef UTIL_H
#define UTIL_H

#define CriticalVar()	register uint32_t	CSVar

#define CriticalEnter()	do { \
	CSVar = __get_PRIMASK();\
	__disable_irq(); \
	} while (0);


#define CriticalLeave() do { \
	__set_PRIMASK(CSVar); \
	} while (0);


#endif //!UTIL_H

