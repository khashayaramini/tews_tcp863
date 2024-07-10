#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
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

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x30ff7695, "module_layout" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0x9befafec, "device_create" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xde80cd09, "ioremap" },
	{ 0xb24af53b, "pci_set_master" },
	{ 0x85bd1608, "__request_region" },
	{ 0x58a78df9, "pci_enable_device" },
	{ 0x4f00afd3, "kmem_cache_alloc_trace" },
	{ 0xac1c4313, "kmalloc_caches" },
	{ 0xfb578fc5, "memset" },
	{ 0xc0b2bc48, "pci_unregister_driver" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0xb3f0559, "class_destroy" },
	{ 0x433f0b06, "__pci_register_driver" },
	{ 0x23295fe0, "__register_chrdev" },
	{ 0x52ea150d, "__class_create" },
	{ 0x3d258838, "pci_disable_device" },
	{ 0xedc03953, "iounmap" },
	{ 0xc1514a3b, "free_irq" },
	{ 0x1035c7c2, "__release_region" },
	{ 0x77358855, "iomem_resource" },
	{ 0x37a0cba, "kfree" },
	{ 0xc85ac280, "device_destroy" },
	{ 0x65487097, "__x86_indirect_thunk_rax" },
	{ 0x33a21a09, "pv_ops" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0xa751b1a4, "dma_alloc_attrs" },
	{ 0x1000e51, "schedule" },
	{ 0xcf2a6966, "up" },
	{ 0x6bd0e573, "down_interruptible" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x37110088, "remove_wait_queue" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0x4afb2238, "add_wait_queue" },
	{ 0xa22a96f7, "current_task" },
	{ 0xaad8c7d6, "default_wake_function" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x82ee90dc, "timer_delete_sync" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0xf9fd2771, "dma_free_attrs" },
	{ 0xa648e561, "__ubsan_handle_shift_out_of_bounds" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x24d273d1, "add_timer" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x92997ed8, "_printk" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xbdfb6dbb, "__fentry__" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("pci:v00001498d0000035Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00001498d0000016Bsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00001498d0000235Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00001498d0000835Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00001498d0000735Fsv*sd*bc*sc*i*");

MODULE_INFO(srcversion, "4382F8C11FA26EEB0613AB8");
