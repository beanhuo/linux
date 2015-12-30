
#define pr_fmt(fmt) "Memory log: " fmt

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/vmalloc.h>
#include <linux/syscalls.h>
#include <linux/workqueue.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/io.h>

static struct work_struct mm_log_wq_work;
static unsigned int mm_log_enable;
static unsigned int mm_log_buf_len = 10240;
static unsigned int mm_log_reserved_len = 128;
static unsigned int mm_log_common_len = 10240 - 128;
static char *mm_log_buf_start;
static unsigned int mm_log_buf_index;
static unsigned int roll_back;
static char *log_file_name = "/mm_log.result";
static unsigned int mm_log_boot;
static spinlock_t mm_log_lock;

static int __init mm_log_setup(char *str)
{
	mm_log_boot = 1;
	mm_log_buf_len = memparse(str, NULL);
	mm_log_common_len = mm_log_buf_len - mm_log_reserved_len;
	mm_log_enable = 1;

	return 1;
}
__setup("mm_log=", mm_log_setup);

static int mm_log_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "memory log:\t%s\n",
			mm_log_enable ? "enable" : "disable");
	seq_printf(m, "buffer start:\t0x%p\n", mm_log_buf_start);
	seq_printf(m, "buffer length:\t0x%x(%d)\n",
			mm_log_buf_len, mm_log_buf_len);
	seq_printf(m, "index:\t\t0x%x(%d)\n",
			mm_log_buf_index, mm_log_buf_index);
	seq_printf(m, "file name:\t\t%s\n", log_file_name);
	seq_printf(m, "roll back:\t%d\n", roll_back);

	return 0;
}

static int mm_log_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, mm_log_proc_show, NULL);
}

static ssize_t mm_log_proc_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *pos)
{
	char option[10];

	if (count <= 0 || count >= 10)
		return count;

	if (copy_from_user(option, buffer, count))
		return -EFAULT;
	option[count] = '\0';
	if (!strcmp(option, "start\n")) {
		mm_log_buf_start = vzalloc(mm_log_buf_len);
		if (!mm_log_buf_start) {
			pr_err("%s: vmalloc failed!\n", __func__);
			return -ENOMEM;
		}
		mm_log_common_len = mm_log_buf_len - mm_log_reserved_len;
		mm_log_enable = 1;
		goto out;
	}
	if (!strcmp(option, "stop\n")) {
		mm_log_enable = 0;
		goto out;
	}
	if (!strcmp(option, "save\n")) {
		mm_log_enable = 0;
		schedule_work(&mm_log_wq_work);
		goto out;
	}
	if (!strcmp(option, "clean\n")) {
		mm_log_buf_index = 0;
		roll_back = 0;
		memset(mm_log_buf_start, 0, mm_log_buf_len);
		goto out;
	}

	pr_err("%s: Unknown str\n", __func__);
out:
	pr_info("mm_log_enable is set to %d\n", mm_log_enable);
	return count;
}

static const struct file_operations mm_log_proc_fops = {
	.open		= mm_log_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= mm_log_proc_write,
};

static int buf_length_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "buffer length:\t\t0x%x(%d)\n",
			mm_log_buf_len, mm_log_buf_len);

	return 0;
}

static int buf_length_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, buf_length_proc_show, NULL);
}

static ssize_t buffer_length_proc_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *pos)
{
	char option[20];
	unsigned int multiplier = 1;
	unsigned int base = 10;

	if (count <= 0 || count >= 10)
		return count;

	if (copy_from_user(option, buffer, count))
		return -EFAULT;

	option[count] = '\0';
	pr_info("option is %s", option);

	if (option[count - 2] == 'k' || option[count - 2] == 'K')
		multiplier = 1024;
	else if (option[count - 2] == 'm' || option[count - 2] == 'M')
		multiplier = 1024*1024;
	else if (option[count - 2] < '0' || option[count - 2] > '9')
		return -EFAULT;

	if (option[0] == '0' && (option[1] == 'x' || option[1] == 'X'))
		base = 16;

	mm_log_buf_len = simple_strtoul(option, NULL, base);
	mm_log_buf_len *= multiplier;
	pr_info("buffer length is set to %d\n", mm_log_buf_len);

	return count;
}

static const struct file_operations buf_length_proc_fops = {
	.open		= buf_length_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= buffer_length_proc_write,
};

static int file_name_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "name:\t\t%s\n", log_file_name);

	return 0;
}

static int file_name_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, file_name_proc_show, NULL);
}

static ssize_t file_name_proc_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *pos)
{
	char *name = NULL;

	if (count <= 0)
		return count;
	name = kmalloc(count, GFP_KERNEL);
	if (!name) {
		pr_err("No memory.\n");
		return -ENOMEM;
	}
	if (copy_from_user(name, buffer, count - 1))
		return -EFAULT;

	name[count - 1] = '\0';
	pr_info("name is %s\n", name);
	log_file_name = name;
	pr_info("file name is set to %s\n", log_file_name);

	return count;
}

static const struct file_operations file_name_proc_fops = {
	.open		= file_name_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= file_name_proc_write,
};

static int count_tail_zeros(const char *start, int len)
{
	int zeros = 0, pos = 0;

	pos = len - 1;
	while (*(start + pos) == 0x00) {
		zeros++;
		pos--;
		if (zeros > mm_log_reserved_len || pos <= 0) {
			pr_err("%s zeros too much or pos=%d error!\n",
				__func__, pos);
			break;
		}
	}

	return zeros;
}

static int count_head_offset(const char *start)
{
	int offset = 0;

	while (*(start + offset) != '[') {
		offset++;
		if (offset > mm_log_reserved_len) {
			pr_err("%s find head error!\n", __func__);
			offset = 0;
			break;
		}
	}

	return offset;
}

static void mm_log_to_file(struct work_struct *work)
{
	unsigned int fd;
	int zeros = 0, offset = 0;

	if (mm_log_buf_start == NULL) {
		pr_info("mm_log need start first\n");
		return;
	}

	fd = sys_open(log_file_name, O_CREAT | O_RDWR, 0644);
	pr_info("fd is 0x%x ,buf_start is 0x%p, length is 0x%x, index is 0x%x\n",
		fd, mm_log_buf_start, mm_log_buf_len, mm_log_buf_index);
	if (roll_back) {
		zeros = count_tail_zeros(mm_log_buf_start + mm_log_buf_index,
					mm_log_buf_len - mm_log_buf_index);
		offset = count_head_offset(mm_log_buf_start + mm_log_buf_index);
		sys_write(fd, mm_log_buf_start + mm_log_buf_index + offset,
			mm_log_buf_len - mm_log_buf_index - zeros);
	}
	offset = count_head_offset(mm_log_buf_start);
	sys_write(fd, mm_log_buf_start + offset, mm_log_buf_index);
	sys_close(fd);
	vfree(mm_log_buf_start);
	mm_log_buf_start = NULL;
	mm_log_buf_index = 0;
	roll_back = 0;
}

void mm_log_print(const struct device *dev, const char *fmt, ...)
{
	va_list args;
	u64 t_nsec;
	unsigned long rem_nsec;
	unsigned long flags;

	if (mm_log_enable) {
		t_nsec = ktime_to_ns(ktime_get());
		rem_nsec = do_div(t_nsec, 1000000000);
		spin_lock_irqsave(&mm_log_lock, flags);
		mm_log_buf_index += scnprintf(
				mm_log_buf_start + mm_log_buf_index,
				mm_log_buf_len - mm_log_buf_index,
				"[%5lu.%09lu] ", (unsigned long)t_nsec,
				rem_nsec);
		if (dev)
			mm_log_buf_index += scnprintf(
				mm_log_buf_start + mm_log_buf_index,
				mm_log_buf_len - mm_log_buf_index,
				"%s ", dev_name(dev));
		va_start(args, fmt);
		mm_log_buf_index += vscnprintf(
				mm_log_buf_start + mm_log_buf_index,
				mm_log_buf_len - mm_log_buf_index, fmt, args);
		va_end(args);
		if (mm_log_buf_index + 1 >= mm_log_common_len) {
			memset(mm_log_buf_start + mm_log_buf_index, 0,
				mm_log_buf_len - mm_log_buf_index);
			mm_log_buf_index = 0;
			roll_back = 1;
		}
		spin_unlock_irqrestore(&mm_log_lock, flags);
	}
}
EXPORT_SYMBOL_GPL(mm_log_print);

static __init int mm_log_proc_init(void)
{
	struct proc_dir_entry *p;
	struct proc_dir_entry *proc_mm_log;

	proc_mm_log = proc_mkdir("mm_log", NULL);
	if (proc_mm_log == NULL)
		return -ENOMEM;

	p = proc_create("mm_log", 0644, proc_mm_log, &mm_log_proc_fops);
	if (!p)
		return -ENOMEM;
	p = proc_create("buffer_length", 0644, proc_mm_log,
			&buf_length_proc_fops);
	if (!p)
		return -ENOMEM;
	p = proc_create("file_name", 0644, proc_mm_log, &file_name_proc_fops);
	if (!p)
		return -ENOMEM;

	spin_lock_init(&mm_log_lock);

	if (mm_log_boot) {
		mm_log_buf_start = vzalloc(mm_log_buf_len);
		if (!mm_log_buf_start) {
			pr_err("%s: vmalloc failed!\n", __func__);
			return -ENOMEM;
		}
	}
	INIT_WORK(&mm_log_wq_work, mm_log_to_file);
	pr_info("%s success\n", __func__);

	return 0;
}

module_init(mm_log_proc_init);

MODULE_DESCRIPTION("Memory log module");
MODULE_AUTHOR("Peter Pan<peterpandong@micron.com>");
MODULE_LICENSE("GPL v2");
