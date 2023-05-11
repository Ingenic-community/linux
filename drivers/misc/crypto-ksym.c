#include <linux/module.h>
#include "aes.h"
#include "md5.h"
EXPORT_SYMBOL(aes_set_key);
EXPORT_SYMBOL(aes_encrypt);
EXPORT_SYMBOL(aes_decrypt);
EXPORT_SYMBOL(md5_transform_CPUbyteorder);
#if defined(CONFIG_X86_64)
EXPORT_SYMBOL(md5_transform_CPUbyteorder_2x);
#endif
#if defined(CONFIG_BLK_DEV_LOOP_INTELAES) && (defined(CONFIG_X86) || defined(CONFIG_X86_64))
asmlinkage extern void intel_aes_cbc_encrypt(const aes_context *, void *src, void *dst, size_t len, void *iv);
asmlinkage extern void intel_aes_cbc_decrypt(const aes_context *, void *src, void *dst, size_t len, void *iv);
asmlinkage extern void intel_aes_cbc_enc_4x512(aes_context **, void *src, void *dst, void *iv);
EXPORT_SYMBOL(intel_aes_cbc_encrypt);
EXPORT_SYMBOL(intel_aes_cbc_decrypt);
EXPORT_SYMBOL(intel_aes_cbc_enc_4x512);
#endif
