#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x4c46c04, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x88068e9b, __VMLINUX_SYMBOL_STR(pci_unregister_driver) },
	{ 0xab405337, __VMLINUX_SYMBOL_STR(__pci_register_driver) },
	{ 0x1000e51, __VMLINUX_SYMBOL_STR(schedule) },
	{ 0x4f8b5ddb, __VMLINUX_SYMBOL_STR(_copy_to_user) },
	{ 0x4f6b400b, __VMLINUX_SYMBOL_STR(_copy_from_user) },
	{ 0xfa66f77c, __VMLINUX_SYMBOL_STR(finish_wait) },
	{ 0xd62c833f, __VMLINUX_SYMBOL_STR(schedule_timeout) },
	{ 0x5c8b5ce8, __VMLINUX_SYMBOL_STR(prepare_to_wait) },
	{ 0xc8b57c27, __VMLINUX_SYMBOL_STR(autoremove_wake_function) },
	{ 0x608106e7, __VMLINUX_SYMBOL_STR(current_task) },
	{ 0xcf21d241, __VMLINUX_SYMBOL_STR(__wake_up) },
	{ 0x2072ee9b, __VMLINUX_SYMBOL_STR(request_threaded_irq) },
	{ 0xf432dd3d, __VMLINUX_SYMBOL_STR(__init_waitqueue_head) },
	{ 0x8196f73b, __VMLINUX_SYMBOL_STR(misc_register) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0xc20ff9b, __VMLINUX_SYMBOL_STR(pci_bus_read_config_word) },
	{ 0x5d8b17fd, __VMLINUX_SYMBOL_STR(pci_iomap) },
	{ 0xf0fdf6cb, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x70ad5be4, __VMLINUX_SYMBOL_STR(pci_request_regions) },
	{ 0xd8be31ba, __VMLINUX_SYMBOL_STR(pci_enable_device) },
	{ 0xc6fd2075, __VMLINUX_SYMBOL_STR(pci_disable_device) },
	{ 0x1cfef2c8, __VMLINUX_SYMBOL_STR(pci_release_regions) },
	{ 0xf20dabd8, __VMLINUX_SYMBOL_STR(free_irq) },
	{ 0x10d240f1, __VMLINUX_SYMBOL_STR(misc_deregister) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("pci:v000010B5d00009050sv00001079sd00001640bc*sc*i*");
MODULE_ALIAS("pci:v000010B5d00009050sv00001079sd00001800bc*sc*i*");
MODULE_ALIAS("pci:v000010B5d00009050sv00001079sd00001842bc*sc*i*");
MODULE_ALIAS("pci:v000010B5d00009050sv00001079sd00001806bc*sc*i*");

MODULE_INFO(srcversion, "9A8DD800B101735C5BFC51A");
