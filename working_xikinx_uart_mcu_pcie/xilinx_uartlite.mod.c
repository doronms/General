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
	{ 0xdb6f1e24, "platform_get_resource" },
	{ 0x5fe6b44, "devm_ioremap" },
	{ 0x60be3f9a, "platform_get_irq" },
	{ 0x5facd4d6, "rt_mutex_base_init" },
	{ 0xa78af5f3, "ioread32" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0x8ad552d0, "uart_add_one_port" },
	{ 0x637aad5f, "mcu_pcie_register_irq_handler" },
	{ 0x94caa9e0, "_dev_warn" },
	{ 0x78534f62, "init_timer_key" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x3cf85989, "mod_timer" },
	{ 0x78100720, "__dynamic_dev_dbg" },
	{ 0xad7d5c88, "uart_write_wakeup" },
	{ 0xa9dcb22c, "platform_driver_unregister" },
	{ 0xf7fb6710, "__tty_insert_flip_string_flags" },
	{ 0xa88f32c5, "tty_flip_buffer_push" },
	{ 0x4a17ed66, "sysrq_mask" },
	{ 0x69884848, "uart_try_toggle_sysrq" },
	{ 0xc9993ea2, "handle_sysrq" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x92997ed8, "_printk" },
	{ 0xcb1c99b0, "uart_register_driver" },
	{ 0xaac17ff, "__platform_driver_register" },
	{ 0x755bd03d, "uart_unregister_driver" },
	{ 0x13439a3a, "_dev_info" },
	{ 0xf2b806ef, "uart_remove_one_port" },
	{ 0xc036c054, "mutex_lock" },
	{ 0x4e5b1d4a, "mutex_unlock" },
	{ 0x74ab9eac, "rt_spin_lock" },
	{ 0x993bb4e5, "uart_get_baud_rate" },
	{ 0x50c79c68, "uart_update_timeout" },
	{ 0x7554dd4e, "rt_spin_unlock" },
	{ 0xb3087f55, "timer_delete_sync" },
	{ 0x4a453f53, "iowrite32" },
	{ 0x50468119, "mcu_pcie_unregister_irq_handler" },
	{ 0x96ec16d1, "_dev_err" },
	{ 0xc58fa889, "devm_kmalloc" },
	{ 0x7a9ebab4, "module_layout" },
};

MODULE_INFO(depends, "mcu_pcie");


MODULE_INFO(srcversion, "C211A0DCE2749F89E8F177D");
MODULE_INFO(rhelversion, "9.5");
