#include "evlog.h"

static char ev_buf[4096];

ssize_t ev_log_read(struct file *file, char __user *buf,
			   size_t count, loff_t *ppos)
{
	struct ev_log *log = file->private_data;
	struct ev_entry *entry;
	unsigned long flags;
	int size = 0;
	unsigned n, id, max;
	ktime_t now, t;
	
	max = log->max;
	now = ktime_get();
	local_irq_save(flags);
	n = (log->next - 1) & (max - 1);
	entry = log->entry;
	while (n != log->next) {
		t = ktime_sub(now, entry[n].when);
		id = entry[n].id;
		if (id) {
			const char *str;
			if (id < log->num_strings)
				str = log->strings[id];
			else
				str = "UNKNOWN";
			size += scnprintf(ev_buf + size, 4096 - size,
					  "%8ld.%03ld %08x %s\n",
					  ((long)t.tv64), ((long)t.tv64) / 1000000,
					  entry[n].arg, str);
		}
		n = (n - 1) & (max - 1);
	}
	log->fault = 0;
	local_irq_restore(flags);
	return simple_read_from_buffer(buf, count, ppos, ev_buf, size);
}

void ev_log_write(struct ev_log *log, unsigned id, unsigned arg)
{
	struct ev_entry *entry;
	unsigned long flags;
	local_irq_save(flags);

	if (log->fault) {
		if (log->fault == 1)
			goto done;
		log->fault--;
	}

	entry = log->entry + log->next;
	entry->when = ktime_get();
	entry->id = id;
	entry->arg = arg;
	log->next = (log->next + 1) & (log->max - 1);
done:
	local_irq_restore(flags);
}

void ev_log_freeze(struct ev_log *log, unsigned count)
{
	unsigned long flags;
	local_irq_save(flags);
	log->fault = count;
	local_irq_restore(flags);
}

int ev_log_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations ev_log_ops = {
	.read = ev_log_read,
	.open = ev_log_open,
};

int ev_log_init(struct ev_log *log)
{
	debugfs_create_file(log->name, 0444, 0, log, &ev_log_ops);
	return 0;
}

