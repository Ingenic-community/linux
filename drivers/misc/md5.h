/* md5.h */

#include <linux/types.h>
#include <linux/linkage.h>
#include <linux/module.h>

#if defined(CONFIG_X86) || defined(CONFIG_X86_64)
 asmlinkage
#endif
extern void md5_transform_CPUbyteorder(u_int32_t *, u_int32_t const *);

#if defined(CONFIG_X86) || defined(CONFIG_X86_64)
 asmlinkage
#endif
extern void md5_transform_CPUbyteorder_2x(u_int32_t *, u_int32_t const *, u_int32_t const *);
