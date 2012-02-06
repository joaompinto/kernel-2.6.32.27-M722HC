
#ifndef __KDEBUG_H__
#define __KDEBUG_H__

#define KDEB_LEVEL	1

#if KDEB_LEVEL
#define KDEBUG(format, ...)	printk("==" format, ## __VA_ARGS__)
#else
#define KDEBUG(format, ...)	////////
#endif

/**
 KDEBUG("MGQ %s:%d,%s\n",__FILE__, __LINE__, __func__);
 */

#endif //

