#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <soc/oppo/oppo_project.h>

#include <linux/io.h>
#include <linux/of.h>
#include <sec_boot_lib.h>

/////////////////////////////////////////////////////////////
static struct proc_dir_entry *oppoVersion = NULL;

static ProjectInfoCDTType *format = NULL;
static ProjectInfoCDTType projectInfo = {
	.nProject		= 0,
	.nModem			= 0,
	.nOperator		= 0,
	.nPCBVersion	= 0,
};

static unsigned int init_project_version(void)
{
	struct device_node *np = NULL;
	int ret = 0;
    printk("init_project_version start\n");
	if(format == NULL)
		format = &projectInfo;

	np = of_find_node_by_name(NULL,"oppo_project");
	if(!np){
		printk("init_project_version error1");
		return 0;
	}

	ret = of_property_read_u32(np,"nProject",&(format->nProject));
	if(ret)
	{
		printk("init_project_version error2");
		return 0;
	}

	ret = of_property_read_u32(np,"nModem",&(format->nModem));
	if(ret)
	{
		printk("init_project_version error3");
		return 0;
	}

	ret = of_property_read_u32(np,"nOperator",&(format->nOperator));
	if(ret)
	{
		printk("init_project_version error4");
		return 0;
	}

	ret = of_property_read_u32(np,"nPCBVersion",&(format->nPCBVersion));
	if(ret)
	{
		printk("init_project_version error5");
		return 0;
	}
	printk("KE Version Info :Project(%d) Modem(%d) Operator(%d) PCB(%d)\n",
		format->nProject,format->nModem,format->nOperator,format->nPCBVersion);

	return format->nProject;
}


unsigned int get_project(void)
{
	if(format)
		return format->nProject;
	else
		return init_project_version();
	return 0;
}

/* xiang.fei@PSW.MM.AudioDriver.Machine, 2018/05/28, Add for kernel driver */
EXPORT_SYMBOL(get_project);

unsigned int is_project(OPPO_PROJECT project )
{
	return (get_project() == project?1:0);
}

unsigned int get_PCB_Version(void)
{
	if(format)
		return format->nPCBVersion;
	return 0;
}

unsigned int get_Modem_Version(void)
{
	if(format)
		return format->nModem;
	return 0;
}

unsigned int get_Operator_Version(void)
{
	if (format == NULL) {
		init_project_version();
	}
	if(format)
		return format->nOperator;
	return 0;
}

//this module just init for creat files to show which version
static ssize_t prjVersion_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;
	len = sprintf(page,"%d",get_project());

	if(len > *off)
		len -= *off;
	else
		len = 0;

	if(copy_to_user(buf,page,(len < count ? len : count))){
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

static struct file_operations prjVersion_proc_fops = {
	.read = prjVersion_read_proc,
	.write = NULL,
};


static ssize_t pcbVersion_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(page,"%d",get_PCB_Version());

	if(len > *off)
	   len -= *off;
	else
	   len = 0;

	if(copy_to_user(buf,page,(len < count ? len : count))){
	   return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);

}

static struct file_operations pcbVersion_proc_fops = {
	.read = pcbVersion_read_proc,
};


static ssize_t operatorName_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(page,"%d",get_Operator_Version());

	if(len > *off)
	   len -= *off;
	else
	   len = 0;

	if(copy_to_user(buf,page,(len < count ? len : count))){
	   return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);

}

static struct file_operations operatorName_proc_fops = {
	.read = operatorName_read_proc,
};


static ssize_t modemType_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(page,"%d",get_Modem_Version());

	if(len > *off)
	   len -= *off;
	else
	   len = 0;

	if(copy_to_user(buf,page,(len < count ? len : count))){
	   return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);

}

static struct file_operations modemType_proc_fops = {
	.read = modemType_read_proc,
};

static ssize_t secureType_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;
        #ifndef VENDOR_EDIT
	//LiBin@Prd6.BaseDrv, 2016/11/03,, Add for MTK secureType node
	//void __iomem *oem_config_base;
	uint32_t secure_oem_config = 0;

	//oem_config_base = ioremap(0x58034 , 10);
	//secure_oem_config = __raw_readl(oem_config_base);
	//iounmap(oem_config_base);
        printk(KERN_EMERG "lycan test secure_oem_config 0x%x\n", secure_oem_config);
        #else
	int secure_oem_config = 0;
	secure_oem_config = sec_schip_enabled();
	printk(KERN_EMERG "lycan test secure_oem_config %d\n", secure_oem_config);
        #endif /* VENDOR_EDIT */

	len = sprintf(page,"%d", secure_oem_config);

	if(len > *off)
	   len -= *off;
	else
	   len = 0;

	if(copy_to_user(buf,page,(len < count ? len : count))){
	   return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);

}


static struct file_operations secureType_proc_fops = {
	.read = secureType_read_proc,
};


static int __init oppo_project_init(void)
{
	int ret = 0;
	struct proc_dir_entry *pentry;

	if(oppoVersion) //if oppoVersion is not null means this proc dir has inited
	{
		return ret;
	}

	oppoVersion =  proc_mkdir("oppoVersion", NULL);
	if(!oppoVersion) {
		pr_err("can't create oppoVersion proc\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("prjVersion", S_IRUGO, oppoVersion, &prjVersion_proc_fops);
	if(!pentry) {
		pr_err("create prjVersion proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("pcbVersion", S_IRUGO, oppoVersion, &pcbVersion_proc_fops);
	if(!pentry) {
		pr_err("create pcbVersion proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("operatorName", S_IRUGO, oppoVersion, &operatorName_proc_fops);
	if(!pentry) {
		pr_err("create operatorName proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("modemType", S_IRUGO, oppoVersion, &modemType_proc_fops);
	if(!pentry) {
		pr_err("create modemType proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("secureType", S_IRUGO, oppoVersion, &secureType_proc_fops);
	if(!pentry) {
		pr_err("create secureType proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	return ret;
ERROR_INIT_VERSION:
	//remove_proc_entry("oppoVersion", NULL);
	return -ENOENT;
}


static int __init boot_oppo_project_core(void)
{
	init_project_version();
	oppo_project_init();
	return 0;
}

core_initcall(boot_oppo_project_core);

MODULE_DESCRIPTION("OPPO project version");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Joshua <gyx@oppo.com>");