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
	{ 0x7a707aea, "module_layout" },
	{ 0x28ae2f48, "pci_unregister_driver" },
	{ 0xb37f9677, "__pci_register_driver" },
	{ 0x1000e51, "schedule" },
	{ 0x4f6b400b, "_copy_from_user" },
	{ 0xfa66f77c, "finish_wait" },
	{ 0xd62c833f, "schedule_timeout" },
	{ 0x5c8b5ce8, "prepare_to_wait" },
	{ 0xc8b57c27, "autoremove_wake_function" },
	{ 0xa42dad06, "current_task" },
	{ 0x4f8b5ddb, "_copy_to_user" },
	{ 0xa1c76e0a, "_cond_resched" },
	{ 0xcf21d241, "__wake_up" },
	{ 0x2072ee9b, "request_threaded_irq" },
	{ 0xf432dd3d, "__init_waitqueue_head" },
	{ 0xde51ea6a, "misc_register" },
	{ 0x91715312, "sprintf" },
	{ 0x9fb390e, "pci_bus_read_config_word" },
	{ 0x48334927, "pci_iomap" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0x27e1a049, "printk" },
	{ 0x2dc1207f, "pci_request_regions" },
	{ 0x1d7b393a, "pci_enable_device" },
	{ 0x8101f80b, "pci_disable_device" },
	{ 0xc84025ff, "pci_release_regions" },
	{ 0xf20dabd8, "free_irq" },
	{ 0xcdf91cc8, "misc_deregister" },
	{ 0xbdfb6dbb, "__fentry__" },
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
