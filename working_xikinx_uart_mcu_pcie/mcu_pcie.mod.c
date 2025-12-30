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

KSYMTAB_FUNC(mcu_pcie_register_irq_handler, "_gpl", "");
KSYMTAB_FUNC(mcu_pcie_unregister_irq_handler, "_gpl", "");
KSYMTAB_FUNC(mcu_pcie_get_pending, "_gpl", "");
KSYMTAB_FUNC(xdma_get_and_clear_pending, "_gpl", "");

SYMBOL_CRC(mcu_pcie_register_irq_handler, 0x637aad5f, "_gpl");
SYMBOL_CRC(mcu_pcie_unregister_irq_handler, 0x50468119, "_gpl");
SYMBOL_CRC(mcu_pcie_get_pending, 0x94376fdc, "_gpl");
SYMBOL_CRC(xdma_get_and_clear_pending, 0xd0801fda, "_gpl");

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xc036c054, "mutex_lock" },
	{ 0x4e5b1d4a, "mutex_unlock" },
	{ 0xe168c07c, "blocking_notifier_chain_register" },
	{ 0x78100720, "__dynamic_dev_dbg" },
	{ 0x70b77795, "blocking_notifier_chain_unregister" },
	{ 0x74ab9eac, "rt_spin_lock" },
	{ 0x7554dd4e, "rt_spin_unlock" },
	{ 0x1d840b93, "__pci_register_driver" },
	{ 0x13439a3a, "_dev_info" },
	{ 0x3c12dfe, "cancel_work_sync" },
	{ 0xacf6a1c5, "debugfs_remove" },
	{ 0x83fded40, "platform_device_unregister" },
	{ 0x70d3f49f, "pci_free_irq_vectors" },
	{ 0x85555940, "pci_release_regions" },
	{ 0x158d5c76, "pci_disable_device" },
	{ 0x467bb8b4, "blocking_notifier_call_chain" },
	{ 0xce7f7c2b, "pci_irq_vector" },
	{ 0xca6fabb5, "platform_device_alloc" },
	{ 0x96ec16d1, "_dev_err" },
	{ 0xc58fa889, "devm_kmalloc" },
	{ 0x6281e72, "platform_device_add_data" },
	{ 0xbc55d4b2, "platform_device_add_resources" },
	{ 0x7c964424, "platform_device_put" },
	{ 0x4ba10263, "platform_device_add" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x21c1f2f2, "pci_enable_device" },
	{ 0xc301d926, "pci_set_master" },
	{ 0x72d8c299, "pci_request_regions" },
	{ 0x5facd4d6, "rt_mutex_base_init" },
	{ 0xe5fab46, "__init_rwsem" },
	{ 0x5fe6b44, "devm_ioremap" },
	{ 0x94caa9e0, "_dev_warn" },
	{ 0xeb63adf2, "pci_alloc_irq_vectors" },
	{ 0x6dc23832, "devm_request_threaded_irq" },
	{ 0x3d0c2b4f, "debugfs_create_dir" },
	{ 0xedc48c0f, "debugfs_create_atomic_t" },
	{ 0x656e4a6e, "snprintf" },
	{ 0xb7d6b2a4, "pci_unregister_driver" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x7a9ebab4, "module_layout" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("pci:v000010EEd00009021sv*sd*bc*sc*i*");

MODULE_INFO(srcversion, "89F8F728E3B978494CA33B6");
MODULE_INFO(rhelversion, "9.5");
