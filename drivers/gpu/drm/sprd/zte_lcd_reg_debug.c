#include "zte_lcd_common.h"

#define DTYPE_GEN_READ		0x04	/* long read, 0 parameter */
#define DTYPE_GEN_WRITE	0x03	/* short write, 0 parameter */

#define DTYPE_DCS_READ		0x06	/* read */
#define DTYPE_DCS_WRITE	0x05	/* short write, 0 parameter */

/*
*echo ff988100 > gwrite (0x13,0x29) or echo 51ff > dwrite (0x15,0x39)
*echo 5401 > gread(0x14,0x24), then cat gread
*dread (0x06) sometimes read nothing,return error
*file path: sys/lcd_reg_debug
*/

struct zte_lcd_reg_debug zte_lcd_reg_debug;
extern struct sprd_panel *g_zte_ctrl_pdata;
#define SYSFS_FOLDER_NAME "reg_debug"

#ifdef CONFIG_ZTE_LCD_SPI_PANEL
extern int mdss_spi_tx_command(const void *buf);
extern int mdss_spi_tx_parameter(const void *buf, size_t len);
extern int mdss_spi_read_data(u8 reg_addr, u8 *data, u8 len);
extern int dump_times;

static void zte_lcd_reg_rw_func(struct mipi_dsi_device *ctrl, struct zte_lcd_reg_debug *reg_debug)
{
	unsigned char cmd_2c[] = {0x2c};

	if ((!reg_debug) || (!ctrl))
		return;

	/*if debug this func,define ZTE_LCD_REG_DEBUG 1*/
	#ifdef ZTE_LCD_REG_DEBUG
	for (i = 0; i < reg_debug->length; i++)
		ZTE_LCD_INFO("rwbuf[%d]= %x\n", i, reg_debug->wbuf[i]);
	#endif

	switch (reg_debug->is_read_mode) {
	case REG_READ_MODE:
		mdss_spi_read_data(reg_debug->wbuf[0], reg_debug->rbuf, reg_debug->wbuf[1]);
		mdss_spi_tx_command(&cmd_2c[0]);
		break;
	case REG_WRITE_MODE:
		mdss_spi_tx_command(&reg_debug->wbuf[0]);
		mdss_spi_tx_parameter(&reg_debug->wbuf[1], reg_debug->length-1);
		mdss_spi_tx_command(&cmd_2c[0]);
		break;
	}
}
#else
static void zte_lcd_reg_rw_func(struct mipi_dsi_device *ctrl, struct zte_lcd_reg_debug *reg_debug)
{
	int read_length = -1;

	if ((!reg_debug) || (!ctrl))
		return;

	/*if debug this func,define ZTE_LCD_REG_DEBUG 1*/
	#ifdef ZTE_LCD_REG_DEBUG
	for (i = 0; i < reg_debug->length; i++)
		ZTE_LCD_INFO("rwbuf[%d]= %x\n", i, reg_debug->wbuf[i]);
	#endif

	switch (reg_debug->is_read_mode) {
	case REG_READ_MODE:
		read_length = reg_debug->wbuf[1];
		mipi_dsi_set_maximum_return_packet_size(ctrl, read_length);
		if (zte_lcd_reg_debug.dtype == DTYPE_DCS_READ)
			mipi_dsi_dcs_read(ctrl, reg_debug->wbuf[0], reg_debug->rbuf, read_length);
		else
			mipi_dsi_generic_read(ctrl, &reg_debug->wbuf[0], 1, reg_debug->rbuf, read_length);

		break;
	case REG_WRITE_MODE:
		if (zte_lcd_reg_debug.dtype == DTYPE_DCS_WRITE)
			mipi_dsi_dcs_write_buffer(ctrl, reg_debug->wbuf, reg_debug->length);
		else
			mipi_dsi_generic_write(ctrl, reg_debug->wbuf, reg_debug->length);

		break;
	default:
		ZTE_LCD_ERROR("%s:rw error\n", __func__);
		break;
	}

}
#endif

static void get_user_sapce_data(const char *buf, size_t count)
{
	int i = 0, length = 0;
	char lcd_status[ZTE_REG_LEN*2] = { 0 };

	if (count >= sizeof(lcd_status)) {
		ZTE_LCD_INFO("count=%zu,sizeof(lcd_status)=%zu\n", count, sizeof(lcd_status));
		return;
	}

	strlcpy(lcd_status, buf, count);
	memset(zte_lcd_reg_debug.wbuf, 0, ZTE_REG_LEN);
	memset(zte_lcd_reg_debug.rbuf, 0, ZTE_REG_LEN);

	/*if debug this func,define ZTE_LCD_REG_DEBUG 1*/
	#ifdef ZTE_LCD_REG_DEBUG
	for (i = 0; i < count; i++)
		ZTE_LCD_INFO("lcd_status[%d]=%c  %d\n", i, lcd_status[i], lcd_status[i]);
	#endif
	for (i = 0; i < count; i++) {
		if (isdigit(lcd_status[i]))
			lcd_status[i] -= '0';
		else if (isalpha(lcd_status[i]))
			lcd_status[i] -= (isupper(lcd_status[i]) ? 'A' - 10 : 'a' - 10);
	}
	for (i = 0, length = 0; i < (count-1); i = i+2, length++) {
		zte_lcd_reg_debug.wbuf[length] = lcd_status[i]*16 + lcd_status[1+i];
	}

	zte_lcd_reg_debug.length = length; /*length is use space write data number*/
}

static ssize_t sysfs_show_read(struct device *d, struct device_attribute *attr, char *buf)
{
	int i = 0, len = 0, count = 0;
	char *s = NULL;
	char *data_buf = NULL;

	data_buf = kzalloc(ZTE_REG_LEN * REG_MAX_LEN, GFP_KERNEL);
	if (!data_buf)
		return -ENOMEM;

	s = data_buf;
	for (i = 0; i < zte_lcd_reg_debug.length; i++) {
		len = snprintf(s, 20, "rbuf[%02d]=%02x ", i, zte_lcd_reg_debug.rbuf[i]);
		s += len;
		if ((i+1)%8 == 0) {
			len = snprintf(s, 20, "\n");
			s += len;
		}
	}

	count = snprintf(buf, PAGE_SIZE, "read back:\n%s\n", data_buf);
	kfree(data_buf);
	return count;
}
static ssize_t sysfs_store_dread(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int i = 0, length = 0;

	get_user_sapce_data(buf, count);
	length = zte_lcd_reg_debug.wbuf[1];
	if (length < 1) {
		ZTE_LCD_ERROR("%s:read length is 0\n", __func__);
		return count;
	}

	zte_lcd_reg_debug.is_read_mode = REG_READ_MODE;
	zte_lcd_reg_debug.dtype = DTYPE_DCS_READ;

	ZTE_LCD_INFO("dtype = %x read cmd = %x length = %x\n", zte_lcd_reg_debug.dtype,
		zte_lcd_reg_debug.wbuf[0], length);
	zte_lcd_reg_rw_func(g_zte_ctrl_pdata->slave, &zte_lcd_reg_debug);

	zte_lcd_reg_debug.length = length;
	for (i = 0; i < length; i++)
		ZTE_LCD_INFO("read zte_lcd_reg_debug.rbuf[%d]=0x%02x\n", i, zte_lcd_reg_debug.rbuf[i]);

	return count;
}

static ssize_t sysfs_store_gread(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int i = 0, length = 0;

	get_user_sapce_data(buf, count);
	length = zte_lcd_reg_debug.wbuf[1];
	if (length < 1) {
		ZTE_LCD_INFO("%s:read length is 0\n", __func__);
		return count;
	}

	zte_lcd_reg_debug.is_read_mode = REG_READ_MODE; /* if 1 read ,0 write*/

	zte_lcd_reg_debug.dtype = DTYPE_GEN_READ;

	ZTE_LCD_INFO("dtype = %x read cmd = %x num = %x\n", zte_lcd_reg_debug.dtype, zte_lcd_reg_debug.wbuf[0], length);
	zte_lcd_reg_rw_func(g_zte_ctrl_pdata->slave, &zte_lcd_reg_debug);

	zte_lcd_reg_debug.length = length;
	for (i = 0; i < length; i++)
		ZTE_LCD_INFO("read zte_lcd_reg_debug.rbuf[%d]=0x%02x\n", i, zte_lcd_reg_debug.rbuf[i]);

	return count;
}

static ssize_t sysfs_store_dwrite(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int length = 0;

	get_user_sapce_data(buf, count);
	length = zte_lcd_reg_debug.length;

	zte_lcd_reg_debug.is_read_mode = REG_WRITE_MODE; /* if 1 read ,0 write*/

	zte_lcd_reg_debug.dtype = DTYPE_DCS_WRITE;

	zte_lcd_reg_rw_func(g_zte_ctrl_pdata->slave, &zte_lcd_reg_debug);
	ZTE_LCD_INFO("dtype = 0x%02x,write cmd = 0x%02x,length = 0x%02x\n", zte_lcd_reg_debug.dtype,
		zte_lcd_reg_debug.wbuf[0], length);

	return count;
}

static ssize_t sysfs_store_gwrite(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int length = 0;

	get_user_sapce_data(buf, count);
	length = zte_lcd_reg_debug.length;

	zte_lcd_reg_debug.is_read_mode = REG_WRITE_MODE;

	zte_lcd_reg_debug.dtype = DTYPE_GEN_WRITE;

	zte_lcd_reg_rw_func(g_zte_ctrl_pdata->slave, &zte_lcd_reg_debug);
	ZTE_LCD_INFO("dtype = 0x%02x write cmd = 0x%02x length = 0x%02x\n", zte_lcd_reg_debug.dtype,
		zte_lcd_reg_debug.wbuf[0], length);

	return count;
}

static ssize_t sysfs_store_mipiclk(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value = 0, tmp = -1, ret = count;

	tmp = kstrtoint(buf, 10, &value);
	if (tmp) {
		ZTE_LCD_ERROR("kstrtouint error!\n");
		ret = tmp;
	} else {
		ZTE_LCD_INFO("you can read/write /sys/class/display/dphy0/freq to modify mipiclk\n");
		ZTE_LCD_INFO("count=%zu value=%d\n", count, value);
	}

	return ret;
}

static ssize_t sysfs_show_reserved(struct device *d, struct device_attribute *attr, char *buf)
{
	int i = 0, len = 0, count = 0;
	char *s = NULL;
	char *data_buf = NULL;

	data_buf = kzalloc(ZTE_REG_LEN * REG_MAX_LEN, GFP_KERNEL);
	if (!data_buf)
		return -ENOMEM;

	s = data_buf;
	for (i = 0; i < zte_lcd_reg_debug.length; i++) {
		len = snprintf(s, 20, "rbuf[%02d]=%02x ", i, zte_lcd_reg_debug.wbuf[i]);
		s += len;
	if ((i+1)%8 == 0) {
			len = snprintf(s, 20, "\n");
			s += len;
		}
	}
	len = snprintf(s, 100, "\n%s", zte_lcd_reg_debug.reserved);
	s += len;

	count = snprintf(buf, PAGE_SIZE, "read back:\n%s\n", data_buf);
	kfree(data_buf);
	return count;
}

static ssize_t sysfs_store_reserved(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int i = 0, value = 0;
	int tmp = -1;

	get_user_sapce_data(buf, count);
	for (i = 0; i < zte_lcd_reg_debug.length; i++)
		ZTE_LCD_INFO("write data [%d]=0x%02x\n", i, zte_lcd_reg_debug.wbuf[i]);

	#ifdef CONFIG_ZTE_LCD_SPI_PANEL
	if (zte_lcd_reg_debug.wbuf[0] == 0x55)
		dump_times = 0;
	else
		dump_times = -1;
	ZTE_LCD_INFO("dump_times=%d\n", dump_times);
	#endif

	tmp = kstrtouint(buf, 10, &value);
	if (tmp) {
		ZTE_LCD_ERROR("kstrtouint error!\n");
	} else {
		ZTE_LCD_INFO("count=%zu value=%d\n", count, value);
		snprintf(zte_lcd_reg_debug.reserved, ZTE_REG_LEN, "reserved str=%d", value);
	}
/******************************* add code here ************************************************/
	return count;
}

static ssize_t sysfs_show_initcode(struct device *d, struct device_attribute *attr, char *buf)
{
	const struct dsi_cmd_desc *cmds = g_zte_ctrl_pdata->info.cmds[CMD_CODE_INIT];
	int size = g_zte_ctrl_pdata->info.cmds_len[CMD_CODE_INIT];
	int i = 0;
	uint16_t len = 0;
	uint32_t count = 0;

	ZTE_LCD_INFO("%s", __func__);

	if ((cmds == NULL) || (size == 0))
		return -EINVAL;

	while (size > 0) {
		len = (cmds->wc_h << 8) | cmds->wc_l;

		count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->data_type);
		count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->wait);
		count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->wc_h);
		count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->wc_l);
		for (i = 0; i < len; i++) {
			count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->payload[i]);
		}

		count += snprintf(buf + count, PAGE_SIZE - count, "\n");
		cmds = (const struct dsi_cmd_desc *)(cmds->payload + len);
		size -= (len + 4);
	}

	return count;
}

#define LCD_INITCODE_FILE_NAME "/sdcard/initcode.bin"
static ssize_t sysfs_store_initcode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	static int first_flag = 1;
	struct file *pfile = NULL;
	off_t fsize = 0;
	loff_t pos = 0;
	mm_segment_t old_fs;
	void *p = NULL;

	ZTE_LCD_INFO("%s", __func__);

	pfile = filp_open(LCD_INITCODE_FILE_NAME, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		ZTE_LCD_ERROR("error occurred while opening file %s.", LCD_INITCODE_FILE_NAME);
		return -EIO;
	}

	fsize = file_inode(pfile)->i_size;
	p = kzalloc(fsize, GFP_KERNEL);
	if (p == NULL) {
		ZTE_LCD_ERROR("%s kzalloc failed", __func__);
		filp_close(pfile, NULL);
		return -ENOMEM;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, p, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	if (first_flag == 0) {
		kfree(g_zte_ctrl_pdata->info.cmds[CMD_CODE_INIT]);
	}
	first_flag = 0;
	g_zte_ctrl_pdata->info.cmds[CMD_CODE_INIT] = p;
	g_zte_ctrl_pdata->info.cmds_len[CMD_CODE_INIT] = fsize;

	return count;
}

static ssize_t sysfs_show_esdcode(struct device *d, struct device_attribute *attr, char *buf)
{
	const struct dsi_cmd_desc *cmds = g_zte_ctrl_pdata->info.cmds[CMD_CODE_ESD_SPECIAL];
	int size = g_zte_ctrl_pdata->info.cmds_len[CMD_CODE_ESD_SPECIAL];
	int i = 0;
	uint16_t len = 0;
	uint32_t count = 0;

	ZTE_LCD_INFO("%s", __func__);

	if ((cmds == NULL) || (size == 0))
		return -EINVAL;

	while (size > 0) {
		len = (cmds->wc_h << 8) | cmds->wc_l;

		count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->data_type);
		count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->wait);
		count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->wc_h);
		count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->wc_l);
		for (i = 0; i < len; i++) {
			count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->payload[i]);
		}

		count += snprintf(buf + count, PAGE_SIZE - count, "\n");
		cmds = (const struct dsi_cmd_desc *)(cmds->payload + len);
		size -= (len + 4);
	}

	return count;
}

#define LCD_ESDCODE_FILE_NAME "/sdcard/esdcode.bin"
static ssize_t sysfs_store_esdcode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	static int first_flag = 1;
	struct file *pfile = NULL;
	off_t fsize = 0;
	loff_t pos = 0;
	mm_segment_t old_fs;
	void *p = NULL;

	ZTE_LCD_INFO("%s", __func__);

	pfile = filp_open(LCD_ESDCODE_FILE_NAME, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		ZTE_LCD_ERROR("error occurred while opening file %s.", LCD_ESDCODE_FILE_NAME);
		return -EIO;
	}

	fsize = file_inode(pfile)->i_size;
	p = kzalloc(fsize, GFP_KERNEL);
	if (p == NULL) {
		ZTE_LCD_ERROR("%s kzalloc failed", __func__);
		filp_close(pfile, NULL);
		return -ENOMEM;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, p, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	if (first_flag == 0) {
		kfree(g_zte_ctrl_pdata->info.cmds[CMD_CODE_ESD_SPECIAL]);
	}
	first_flag = 0;
	g_zte_ctrl_pdata->info.cmds[CMD_CODE_ESD_SPECIAL] = p;
	g_zte_ctrl_pdata->info.cmds_len[CMD_CODE_ESD_SPECIAL] = fsize;

	return count;
}


static DEVICE_ATTR(dread, 0600, sysfs_show_read, sysfs_store_dread);
static DEVICE_ATTR(gread, 0600, sysfs_show_read, sysfs_store_gread);
static DEVICE_ATTR(dwrite, 0600, NULL, sysfs_store_dwrite);
static DEVICE_ATTR(gwrite, 0600, NULL, sysfs_store_gwrite);
static DEVICE_ATTR(mipiclk, 0600, NULL, sysfs_store_mipiclk);
static DEVICE_ATTR(initcode, 0600, sysfs_show_initcode, sysfs_store_initcode);
static DEVICE_ATTR(esdcode, 0600, sysfs_show_esdcode, sysfs_store_esdcode);
static DEVICE_ATTR(reserved, 0600, sysfs_show_reserved, sysfs_store_reserved);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_dread.attr,
	&dev_attr_gread.attr,
	&dev_attr_dwrite.attr,
	&dev_attr_gwrite.attr,
	&dev_attr_mipiclk.attr,
	&dev_attr_initcode.attr,
	&dev_attr_esdcode.attr,
	&dev_attr_reserved.attr,
	NULL,
};

static struct attribute_group sysfs_attr_group = {
	.attrs = sysfs_attrs,
};

void zte_lcd_reg_debug_func(void)
{
	int ret = -1;

	struct kobject *vkey_obj = NULL;

	vkey_obj = kobject_create_and_add(SYSFS_FOLDER_NAME, g_zte_ctrl_pdata->zte_lcd_ctrl->kobj);
	if (!vkey_obj) {
		ZTE_LCD_ERROR("%s:unable to create kobject\n", __func__);
		return;
	}

	ret = sysfs_create_group(vkey_obj, &sysfs_attr_group);
	if (ret) {
		ZTE_LCD_ERROR("%s:failed to create attributes\n", __func__);
	}
}
