#ifndef __LIBSNP_INCLUDE_SNP_SHELL_H__
#define __LIBSNP_INCLUDE_SNP_SHELL_H__

#include <stdio.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define SNP_SHELL_PLACEHOLDER "PLACEHOLDER"      /**< 普通占位符 表示这里存在指令参数但不需要识别具体的参数类型 */


typedef int32_t (*SNP_SHELL_PROC_CB)(char **cmd_list, int32_t num, char *res_str, int32_t res_size);

void snp_shell_init(void);

int32_t snp_shell_setup(char **descriptor, int32_t num, SNP_SHELL_PROC_CB proc_cb);

int32_t snp_shell_exec(char *cmd_str, char *res_str, int32_t res_size);

#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_INCLUDE_SNP_SHELL_H__
