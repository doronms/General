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



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x2cf56265, "__dynamic_pr_debug" },
	{ 0xc036c054, "mutex_lock" },
	{ 0x4e5b1d4a, "mutex_unlock" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x1000e51, "schedule" },
	{ 0xa22c8377, "prepare_to_wait_event" },
	{ 0xde86f0d4, "finish_wait" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x13439a3a, "_dev_info" },
	{ 0xc58fa889, "devm_kmalloc" },
	{ 0x6e175678, "__init_waitqueue_head" },
	{ 0x5facd4d6, "rt_mutex_base_init" },
	{ 0xfe406214, "__mutex_rt_init" },
	{ 0xdb6f1e24, "platform_get_resource" },
	{ 0x96ec16d1, "_dev_err" },
	{ 0x5fe6b44, "devm_ioremap" },
	{ 0x60be3f9a, "platform_get_irq" },
	{ 0x6dc23832, "devm_request_threaded_irq" },
	{ 0xfed93a2f, "misc_register" },
	{ 0xa9dcb22c, "platform_driver_unregister" },
	{ 0xde97cf4e, "__wake_up" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0xaac17ff, "__platform_driver_register" },
	{ 0x3fa62849, "misc_deregister" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x7a9ebab4, "module_layout" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "02E66A929C3BB45B0484A8C");
MODULE_INFO(rhelversion, "9.5");
