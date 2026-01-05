#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_MITIGATION_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

KSYMTAB_FUNC(get_uart_int_mask_register, "_gpl", "");
KSYMTAB_FUNC(set_uart_int_mask_register, "_gpl", "");
KSYMTAB_FUNC(get_uart_int_status_register, "_gpl", "");
KSYMTAB_FUNC(get_fpga_register, "_gpl", "");
KSYMTAB_FUNC(set_fpga_register, "_gpl", "");
KSYMTAB_FUNC(get_quart_global_int_mask_register, "_gpl", "");
KSYMTAB_FUNC(set_quart_global_int_mask_register, "_gpl", "");
KSYMTAB_FUNC(get_quart_global_int_status_register, "_gpl", "");
KSYMTAB_FUNC(get_video_matrix_base_address, "_gpl", "");

SYMBOL_CRC(get_uart_int_mask_register, 0x0f421d1f, "_gpl");
SYMBOL_CRC(set_uart_int_mask_register, 0x351bb452, "_gpl");
SYMBOL_CRC(get_uart_int_status_register, 0xae7f1092, "_gpl");
SYMBOL_CRC(get_fpga_register, 0x33354349, "_gpl");
SYMBOL_CRC(set_fpga_register, 0x9766e133, "_gpl");
SYMBOL_CRC(get_quart_global_int_mask_register, 0xcb93d262, "_gpl");
SYMBOL_CRC(set_quart_global_int_mask_register, 0xe2c88a2f, "_gpl");
SYMBOL_CRC(get_quart_global_int_status_register, 0x3f350cd9, "_gpl");
SYMBOL_CRC(get_video_matrix_base_address, 0x73b5de6a, "_gpl");

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x842c8e9d, "ioread16" },
	{ 0x6a86bc1, "iowrite16" },
	{ 0xaac17ff, "__platform_driver_register" },
	{ 0x3fa62849, "misc_deregister" },
	{ 0xdb6f1e24, "platform_get_resource" },
	{ 0x5fe6b44, "devm_ioremap" },
	{ 0x92997ed8, "_printk" },
	{ 0x96ec16d1, "_dev_err" },
	{ 0xfed93a2f, "misc_register" },
	{ 0xa9dcb22c, "platform_driver_unregister" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0xc036c054, "mutex_lock" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xa78af5f3, "ioread32" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0xb0e602eb, "memmove" },
	{ 0x37a0cba, "kfree" },
	{ 0x4e5b1d4a, "mutex_unlock" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x7a9ebab4, "module_layout" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "0A898EB7535B6E3B79F3675");
MODULE_INFO(rhelversion, "9.5");
