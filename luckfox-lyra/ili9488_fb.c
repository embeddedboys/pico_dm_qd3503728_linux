#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>

#include <linux/wait.h>
#include <linux/spinlock.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/regmap.h>

#include <linux/uaccess.h>

#include <linux/fb.h>
#include <video/mipi_display.h>

#define DRV_NAME "ili9488_drv"

struct ili9488_par;

struct ili9488_operations {
    int (*reset)(struct ili9488_par *par);
    int (*clear)(struct ili9488_par *par);
    int (*idle)(struct ili9488_par *par, bool on);
    int (*blank)(struct ili9488_par *par, bool on);
    int (*sleep)(struct ili9488_par *par, bool on);
    int (*set_addr_win)(struct ili9488_par *par, int xs, int ys, int xe, int ye);
};

struct ili9488_display {
    u32                     xres;
    u32                     yres;
    u32                     bpp;
    u32                     fps;
    u32                     rotate;
    u32                     xs_off;
    u32                     xe_off;
    u32                     ys_off;
    u32                     ye_off;

    char *gamma;
    int gamma_num;
    int gamma_len;
};

struct ili9488_par {

    struct device           *dev;
    u8                      *buf;
    struct {
        void *buf;
        size_t len;
    } txbuf;
    struct {
        struct gpio_desc *reset;
        struct gpio_desc *dc;
        struct gpio_desc *cs;
        struct gpio_desc *blk;
        struct gpio_desc *rd;
        struct gpio_desc *wr;
        struct gpio_desc *db[16];
    } gpio;

    spinlock_t              dirty_lock;
    struct completion       complete;

    /* device specific */
    u32                     refr_mode;
    u32                     wait;
    u32                     busy;

    const struct ili9488_operations        *tftops;
    const struct ili9488_display           *display;

    struct fb_info          *fbinfo;
    struct fb_ops           *fbops;

    u32             pseudo_palette[16];

    u32             dirty_lines_start;
    u32             dirty_lines_end;

};

#define gpio_put(d, v) gpiod_set_raw_value(d, v)
int fbtft_write_gpio16_wr(struct ili9488_par *par, void *buf, size_t len)
{
	volatile u16 data;
	int i;
#ifndef DO_NOT_OPTIMIZE_FBTFT_WRITE_GPIO
	static volatile u16 prev_data;
#endif

	while (len) {
		data = *(u16 *)buf;

		/* Start writing by pulling down /WR */
		gpio_put(par->gpio.wr, 0);

		/* Set data */
#ifndef DO_NOT_OPTIMIZE_FBTFT_WRITE_GPIO
		if (data == prev_data) {
			gpio_put(par->gpio.wr, 1); /* used as delay */
		} else {
			for (i = 0; i < 16; i++) {
				if ((data & 1) != (prev_data & 1))
					gpio_put(par->gpio.db[i],
							data & 1);
				data >>= 1;
				prev_data >>= 1;
			}
		}
#else
		for (i = 0; i < 16; i++) {
			gpio_put(par->gpio.db[i], data & 1);
			data >>= 1;
		}
#endif
		/* Pullup /WR */
		gpio_put(par->gpio.wr, 1);

#ifndef DO_NOT_OPTIMIZE_FBTFT_WRITE_GPIO
		prev_data = *(u16 *)buf;
#endif
		buf += 2;
		len -= 2;
	}

	return 0;
}

static inline void fbtft_write_buf_dc(struct ili9488_par *par, void *buf, size_t len, int dc)
{
    gpio_put(par->gpio.dc, dc);
    fbtft_write_gpio16_wr(par, buf, len);
}

#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__}) / sizeof(int))
static int ili9488_write_reg(struct ili9488_par *par, int len, ...)
{
    u16 *buf = (u16 *)par->buf;
    va_list args;
    int i;

    va_start(args, len);

    *buf = (u16)va_arg(args, unsigned int);
    fbtft_write_buf_dc(par, buf, sizeof(u16), 0);
    len--;

    /* if there no params */
    if (len == 0)
        goto exit_no_param;

    for (i = 0; i < len; i++)
        *buf++ = (u16)va_arg(args, unsigned int);

    len *= 2;
    fbtft_write_buf_dc(par, par->buf, len, 1);
    va_end(args);

exit_no_param:
    va_end(args);
    return 0;
}
#define write_reg(par, ...) \
    ili9488_write_reg(par, NUMARGS(__VA_ARGS__), __VA_ARGS__)

static int ili9488_reset(struct ili9488_par *par)
{
    gpio_put(par->gpio.reset, 1);
    mdelay(10);
    gpio_put(par->gpio.reset, 0);
    mdelay(10);
    gpio_put(par->gpio.reset, 1);
    mdelay(10);
    return 0;
}

static int ili9488_init_display(struct ili9488_par *priv)
{
    ili9488_reset(priv);

    // Positive Gamma Control
    write_reg(priv, 0xE0, 0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F);

    // Negative Gamma Control
    write_reg(priv, 0xE1, 0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F);

    write_reg(priv, 0xC0, 0x17, 0x15);          // Power Control 1
    write_reg(priv, 0xC1, 0x41);                // Power Control 2
    write_reg(priv, 0xC5, 0x00, 0x12, 0x80);    // VCOM Control
    write_reg(priv, 0x36, 0x28);                // Memory Access Control
    write_reg(priv, 0x3A, 0x55);                // Pixel Interface Format RGB565 8080 16-bit
    write_reg(priv, 0xB0, 0x00);                // Interface Mode Control

    // Frame Rate Control
    // write_reg(priv, 0xB1, 0xD0, 0x11);       // 60Hz
    write_reg(priv, 0xB1, 0xD0, 0x14);          // 90Hz

    write_reg(priv, 0xB4, 0x02);                // Display Inversion Control
    write_reg(priv, 0xB6, 0x02, 0x02, 0x3B);    // Display Function Control
    write_reg(priv, 0xB7, 0xC6);                // Entry Mode Set
    write_reg(priv, 0xF7, 0xA9, 0x51, 0x2C, 0x82);  // Adjust Control 3
    write_reg(priv, 0x11);                      // Exit Sleep
    mdelay(60);
    write_reg(priv, 0x29);                      // Display on

    return 0;
}

static int ili9488_blank(struct ili9488_par *par, bool on)
{
    if (on)
        write_reg(par, MIPI_DCS_SET_DISPLAY_OFF);
    else
        write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
    return 0;
}

static int ili9488_set_addr_win(struct ili9488_par *par, int xs, int ys, int xe,
                                int ye)
{
    dev_dbg(par->dev, "xs = %d, xe = %d, ys = %d, ye = %d\n", xs, xe, ys, ye);

    write_reg(par, MIPI_DCS_SET_COLUMN_ADDRESS,
              ((xs >> BITS_PER_BYTE)), (xs & 0xFF),
              ((xe >> BITS_PER_BYTE)), (xe & 0xFF));

    write_reg(par, MIPI_DCS_SET_PAGE_ADDRESS,
              ((ys >> BITS_PER_BYTE)), (ys & 0xFF),
              ((ye >> BITS_PER_BYTE)), (ye & 0xFF));

    write_reg(par, MIPI_DCS_WRITE_MEMORY_START);

    return 0;
}

// static int ili9488_idle(struct ili9488_par *par, bool on)
// {
//     if (on)
//         write_reg(par, MIPI_DCS_EXIT_IDLE_MODE);
//     else
//         write_reg(par, MIPI_DCS_EXIT_IDLE_MODE);

//     return 0;
// }

// static int ili9488_sleep(struct ili9488_par *par, bool on)
// {
//     if (on) {
//         write_reg(par, MIPI_DCS_SET_DISPLAY_OFF);
//         write_reg(par, MIPI_DCS_ENTER_SLEEP_MODE);
//     } else {
//         write_reg(par, MIPI_DCS_EXIT_SLEEP_MODE);
//         write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
//     }

//     return 0;
// }

static int ili9488_clear(struct ili9488_par *priv)
{
    u32 width = priv->display->xres;
    u32 height = priv->display->yres;
    u16 clear = 0x0;
    int x, y;

    printk("clearing screen(%d x %d) ...\n", width, height);

    ili9488_set_addr_win(priv, 0, 0, width, height);

    gpio_put(priv->gpio.dc, 1);
    for (x = 0; x < width; x++)
        for (y = 0; y < height; y++)
            fbtft_write_gpio16_wr(priv, &clear, sizeof(u16));

    return 0;
}

static const struct ili9488_operations default_ili9488_ops = {
    // .idle  = ili9488_idle,
    .clear = ili9488_clear,
    // .blank = ili9488_blank,
    .reset = ili9488_reset,
    // .sleep = ili9488_sleep,
    .set_addr_win = ili9488_set_addr_win,
};

static int ili9488_request_one_gpio(struct ili9488_par *par,
                                    const char *name, int index,
                                    struct gpio_desc **gpiop)
{
    struct device *dev = par->dev;
    struct device_node *np = dev->of_node;
    int gpio, flags, rc = 0;
    enum of_gpio_flags of_flags;

    if (of_find_property(np, name, NULL)) {
        gpio = of_get_named_gpio_flags(np, name, index, &of_flags);
        if (gpio == -ENOENT)
            return 0;
        if (gpio == -EPROBE_DEFER)
            return gpio;
        if (gpio < 0) {
            dev_err(dev,
                    "failed to get '%s' from DT\n", name);
            return gpio;
        }

        flags = (of_flags & OF_GPIO_ACTIVE_LOW) ? GPIOF_OUT_INIT_LOW :
                GPIOF_OUT_INIT_HIGH;
        rc = devm_gpio_request_one(dev, gpio, flags,
                                   dev->driver->name);
        if (rc) {
            dev_err(dev,
                    "gpio_request_one('%s'=%d) failed with %d\n",
                    name, gpio, rc);
            return rc;
        }
        if (gpiop)
            *gpiop = gpio_to_desc(gpio);
        pr_debug("%s : '%s' = GPIO%d\n",
                 __func__, name, gpio);
    }

    return rc;
}

static int ili9488_request_gpios(struct ili9488_par *par)
{
    int rc;
    int i;
    pr_debug("%s, configure from dt\n", __func__);

    rc = ili9488_request_one_gpio(par, "reset", 0, &par->gpio.reset);
    if (rc)
        return rc;
    rc = ili9488_request_one_gpio(par, "dc", 0, &par->gpio.dc);
    if (rc)
        return rc;
    rc = ili9488_request_one_gpio(par, "wr", 0, &par->gpio.wr);
    if (rc)
        return rc;
    rc = ili9488_request_one_gpio(par, "led", 0, &par->gpio.blk);
    if (rc)
        return rc;

    for (i = 0; i < 16; i++) {
		rc = ili9488_request_one_gpio(par, "db", i,
					     &par->gpio.db[i]);
    }

    return 0;
}

/* returns 0 if the property is not present */
static u32 __maybe_unused fbtft_property_value(struct device *dev, const char *propname)
{
    int ret;
    u32 val = 0;

    ret = device_property_read_u32(dev, propname, &val);
    if (ret == 0)
        dev_info(dev, "%s: %s = %u\n", __func__, propname, val);

    return val;
}

static int ili9488_of_config(struct ili9488_par *par)
{
    int rc;

    printk("%s\n", __func__);
    rc = ili9488_request_gpios(par);
    if (rc) {
        dev_err(par->dev, "Request gpios failed!\n");
        return rc;
    }
    return 0;

    /* request xres and yres from dt */
}

// #define MADCTL_BGR BIT(3) /* bitmask for RGB/BGR order */
// #define MADCTL_MV BIT(5) /* bitmask for page/column order */
// #define MADCTL_MX BIT(6) /* bitmask for column address order */
// #define MADCTL_MY BIT(7) /* bitmask for page address order */
// static int ili9488_set_var(struct ili9488_par *par)
// {
//     u8 madctl_par = 0;

//     switch (par->fbinfo->var.rotate) {
//     case 0:
//         break;
//     case 90:
//         madctl_par |= (MADCTL_MV | MADCTL_MY);
//         break;
//     case 180:
//         madctl_par |= (MADCTL_MX | MADCTL_MY);
//         break;
//     case 270:
//         madctl_par |= (MADCTL_MV | MADCTL_MX);
//         break;
//     default:
//         return -EINVAL;

//     }

//     write_reg(par, MIPI_DCS_SET_ADDRESS_MODE, madctl_par);
//     return 0;
// }

static int ili9488_hw_init(struct ili9488_par *par)
{
    printk("%s, Display Panel initializing ...\n", __func__);
    ili9488_init_display(par);

    if (par->gpio.blk)
        gpio_put(par->gpio.blk, 1);
    // ili9488_set_var(par);
    // ili9488_set_gamma(par, default_curves);
    // ili9488_clear(par);

    return 0;
}

#define RED(a)      ((((a) & 0xf800) >> 11) << 3)
#define GREEN(a)    ((((a) & 0x07e0) >> 5) << 2)
#define BLUE(a)     (((a) & 0x001f) << 3)

#define to_rgb565(r,g,b) ((r) << 11 | (g) << 5 | (b))

static inline u16 rgb565_to_grayscale(u16 rgb565)
{
    int r,g,b;
    u16 gray;

    r = RED(rgb565);
    g = GREEN(rgb565);
    b = BLUE(rgb565);

    gray = ((r + g + b) / 3);

    /* map to rgb565 format */
    r = b = gray * 31 / 255;  // 0 ~ 31
    g = gray * 63 / 255;

    return cpu_to_be16(to_rgb565(r, g, b));
}

static inline u16 rgb565_to_grayscale_byweight(u16 rgb565)
{
    int r,g,b;
    u16 gray;

    /* get each channel and expand them to 8 bit */
    r = RED(rgb565);
    g = GREEN(rgb565);
    b = BLUE(rgb565);

    /* convert rgb888 to grayscale */
    gray = ((r * 77 + g * 151 + b * 28) >> 8); // 0 ~ 255

    /* map to rgb565 format */
    r = b = gray * 31 / 255;  // 0 ~ 31
    g = gray * 63 / 255;

    return cpu_to_be16(to_rgb565(r, g, b));
}

static int write_vmem(struct ili9488_par *par, size_t offset, size_t len)
{
    u16 *vmem16;
    __be16 *txbuf16 = par->txbuf.buf;
    size_t remain;
    size_t to_copy;
    size_t tx_array_size;
    int i;

    dev_dbg(par->dev, "%s, offset = %d, len = %d\n", __func__, offset, len);

    remain = len / 2;
    vmem16 = (u16 *)(par->fbinfo->screen_buffer + offset);

    gpio_put(par->gpio.dc, 1);

    /* non-buffered spi write */
    if (!par->txbuf.buf)
        return fbtft_write_gpio16_wr(par, vmem16, len);

    tx_array_size = par->txbuf.len / 2;

    while (remain) {
        to_copy = min(tx_array_size, remain);
        dev_dbg(par->fbinfo->device, "to_copy=%zu, remain=%zu\n",
                to_copy, remain - to_copy);

        for (i = 0; i < to_copy; i++)
            // txbuf16[i] = vmem16[i] << 8 | vmem16[i] >> 8;
            txbuf16[i] = vmem16[i];

        /* send batch to device */
        fbtft_write_gpio16_wr(par, txbuf16, to_copy * 2);

        vmem16 = vmem16 + to_copy;
        remain -= to_copy;
    }
    return 0;
}

static void update_display(struct ili9488_par *par, unsigned int start_line,
                           unsigned int end_line)
{
    size_t offset, len;

    dev_dbg(par->dev, "%s, start_line : %d, end_line : %d\n", __func__, start_line, end_line);

    // par->tftops->idle(par, false);
    /* write vmem to display then call refresh routine */
    /*
     * when this was called, driver should wait for busy pin comes low
     * until next frame refreshed
     */
    if (start_line > end_line) {
        dev_dbg(par->dev, "start line never should bigger than end line !!!!!\n");
        start_line = 0;
        end_line = par->fbinfo->var.yres - 1;
    }

    if (start_line > par->fbinfo->var.yres - 1 ||
        end_line > par->fbinfo->var.yres - 1) {
        dev_dbg(par->dev, "invaild start line or end line !!!!!\n");
        start_line = 0;
        end_line = par->fbinfo->var.yres - 1;
    }

    /* for each column, refresh dirty rows */
    par->tftops->set_addr_win(par, 0, start_line, par->fbinfo->var.xres - 1, end_line);

    offset = start_line * par->fbinfo->fix.line_length;
    len = (end_line - start_line + 1) * par->fbinfo->fix.line_length;

    write_vmem(par, offset, len);

    // par->tftops->idle(par, true);
}

static void ili9488_mkdirty(struct fb_info *info, int y, int height)
{
    struct ili9488_par *par = info->par;
    struct fb_deferred_io *fbdefio = info->fbdefio;

    dev_dbg(info->dev, "%s, y : %d, height : %d\n", __func__, y, height);

    if (y == -1) {
        y = 0;
        height = info->var.yres;
    }

    /* mark dirty lines here, but update all for now */
    spin_lock(&par->dirty_lock);
    if (y < par->dirty_lines_start)
        par->dirty_lines_start = y;
    if (y + height - 1 > par->dirty_lines_end)
        par->dirty_lines_end = y + height - 1;
    spin_unlock(&par->dirty_lock);

    schedule_delayed_work(&info->deferred_work, fbdefio->delay);
}

static void ili9488_deferred_io(struct fb_info *info, struct list_head *pagelist)
{
    struct ili9488_par *par = info->par;
    unsigned int dirty_lines_start, dirty_lines_end;
    unsigned int y_low = 0, y_high = 0;
    unsigned long index;
    struct page *page;
    int count = 0;

    spin_lock(&par->dirty_lock);
    dirty_lines_start = par->dirty_lines_start;
    dirty_lines_end = par->dirty_lines_end;

    /* clean dirty markers */
    par->dirty_lines_start = par->fbinfo->var.yres - 1;
    par->dirty_lines_end = 0;
    spin_unlock(&par->dirty_lock);

    list_for_each_entry(page, pagelist, lru) {
        count++;
        index = page->index << PAGE_SHIFT;
        y_low = index / info->fix.line_length;
        y_high = (index + PAGE_SIZE - 1) / info->fix.line_length;
        dev_dbg(info->device,
                "page->index=%lu y_low=%d y_high=%d\n",
                page->index, y_low, y_high);

        if (y_high > info->var.yres - 1)
            y_high = info->var.yres - 1;
        if (y_low < dirty_lines_start)
            dirty_lines_start = y_low;
        if (y_high > dirty_lines_end)
            dirty_lines_end = y_high;
    }

    dev_dbg(info->device,
            "%s, dirty_line  start : %d, end : %d\n",
            __func__, dirty_lines_start, dirty_lines_end);
    update_display(par, dirty_lines_start, dirty_lines_end);
}

static void ili9488_fb_fillrect(struct fb_info *info,
                                const struct fb_fillrect *rect)
{
    dev_dbg(info->dev,
            "%s: dx=%d, dy=%d, width=%d, height=%d\n",
            __func__, rect->dx, rect->dy, rect->width, rect->height);

    sys_fillrect(info, rect);
    ili9488_mkdirty(info, rect->dy, rect->height);
}

static void ili9488_fb_copyarea(struct fb_info *info,
                                const struct fb_copyarea *area)
{
    dev_dbg(info->dev,
            "%s: dx=%d, dy=%d, width=%d, height=%d\n",
            __func__,  area->dx, area->dy, area->width, area->height);

    sys_copyarea(info, area);
    ili9488_mkdirty(info, area->dy, area->height);
}

static void ili9488_fb_imageblit(struct fb_info *info,
                                 const struct fb_image *image)
{
    dev_dbg(info->dev,
            "%s: dx=%d, dy=%d, width=%d, height=%d\n",
            __func__,  image->dx, image->dy, image->width, image->height);
    sys_imageblit(info, image);

    ili9488_mkdirty(info, image->dy, image->height);
}

static ssize_t ili9488_fb_write(struct fb_info *info, const char __user *buf,
                                size_t count, loff_t *ppos)
{
    ssize_t res;
    dev_dbg(info->dev,
            "%s: count=%zd, ppos=%llu\n", __func__,  count, *ppos);

    res = fb_sys_write(info, buf, count, ppos);

    ili9488_mkdirty(info, -1, 0);
    return 0;
}

/* from pxafb.c */
static unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
    chan &= 0xffff;
    chan >>= 16 - bf->length;
    return chan << bf->offset;
}

static int ili9488_fb_setcolreg(unsigned int regno, unsigned int red,
                                unsigned int green, unsigned int blue,
                                unsigned int transp, struct fb_info *info)
{
    unsigned int val;
    int ret = 1;

    /* printk("%s(regno=%u, red=0x%X, green=0x%X, blue=0x%X, trans=0x%X)\n",
           __func__, regno, red, green, blue, transp); */

    if (regno >= 256)   /* no. of hw registers */
        return 1;
    /*
    * Program hardware... do anything you want with transp
    */

    switch (info->fix.visual) {
    case FB_VISUAL_TRUECOLOR:
        if (regno < 16) {
            val  = chan_to_field(red, &info->var.red);
            val |= chan_to_field(green, &info->var.green);
            val |= chan_to_field(blue, &info->var.blue);

            ((u32 *)(info->pseudo_palette))[regno] = val;
            ret = 0;
        }
        break;
    case FB_VISUAL_MONO01:
        ((u32 *)(info->pseudo_palette))[regno] =
                    (red << info->var.red.offset) |
                    (green << info->var.green.offset) |
                    (blue << info->var.blue.offset) |
                    (transp << info->var.transp.offset);
        ret = 0;
        break;
    }

    return ret;
}

static int ili9488_fb_blank(int blank, struct fb_info *info)
{
    struct ili9488_par *par = info->par;
    int ret = -EINVAL;

    switch (blank) {
    case FB_BLANK_POWERDOWN:
    case FB_BLANK_VSYNC_SUSPEND:
    case FB_BLANK_HSYNC_SUSPEND:
    case FB_BLANK_NORMAL:
        ret = ili9488_blank(par, true);
        break;
    case FB_BLANK_UNBLANK:
        ret = ili9488_blank(par, false);
        break;
    }
    return ret;
}

static const struct ili9488_display display = {
    .xres = 480,
    .yres = 320,
    .bpp = 16,
    .fps = 60,
};

static int ili9488_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct ili9488_par *par;
    struct fb_deferred_io *fbdefio;
    int width, height, bpp, rotate;
    struct fb_info *info;
    struct fb_ops *fbops;
    u8 *vmem = NULL;
    int vmem_size;
    int rc;

    printk("%s\n", __func__);
    /* memory resource alloc */

    rotate = display.rotate;
    bpp = display.bpp;
    switch (rotate) {
    case 90:
    case 270:
        width = display.yres;
        height = display.xres;
        break;
    default:
        width = display.xres;
        height = display.yres;
        break;
    }

    vmem_size = (width * height * bpp) / BITS_PER_BYTE;
    printk("vmem_size : %d\n", vmem_size);
    vmem = vzalloc(vmem_size);
    if (!vmem)
        goto alloc_fail;

    fbops = devm_kzalloc(dev, sizeof(struct fb_ops), GFP_KERNEL);
    if (!fbops)
        goto alloc_fail;

    fbdefio = devm_kzalloc(dev, sizeof(struct fb_deferred_io), GFP_KERNEL);
    if (!fbdefio)
        goto alloc_fail;

    /* framebuffer info setup */
    info = framebuffer_alloc(sizeof(struct ili9488_par), dev);
    if (!info) {
        dev_err(dev, "failed to alloc framebuffer!\n");
        return -ENOMEM;
    }

    info->screen_buffer = vmem;
    info->fbops = fbops;
    info->fbdefio = fbdefio;

    fbops->owner        = dev->driver->owner;
    fbops->fb_read      = fb_sys_read;
    fbops->fb_write     = ili9488_fb_write;
    fbops->fb_fillrect  = ili9488_fb_fillrect;
    fbops->fb_copyarea  = ili9488_fb_copyarea;
    fbops->fb_imageblit = ili9488_fb_imageblit;
    fbops->fb_setcolreg = ili9488_fb_setcolreg;
    fbops->fb_blank     = ili9488_fb_blank;

    snprintf(info->fix.id, sizeof(info->fix.id), "%s", dev->driver->name);
    info->fix.type            =       FB_TYPE_PACKED_PIXELS;
    info->fix.visual          =       FB_VISUAL_TRUECOLOR;
    info->fix.xpanstep        =       0;
    info->fix.ypanstep        =       0;
    info->fix.ywrapstep       =       0;
    info->fix.line_length     =       width * bpp / BITS_PER_BYTE;
    info->fix.accel           =       FB_ACCEL_NONE;
    info->fix.smem_len        =       vmem_size;

    info->var.rotate          =       rotate;
    info->var.xres            =       width;
    info->var.yres            =       height;
    info->var.xres_virtual    =       info->var.xres;
    info->var.yres_virtual    =       info->var.yres;

    info->var.bits_per_pixel  =       bpp;
    info->var.nonstd          =       1;
    info->var.grayscale       =       0;

    switch (info->var.bits_per_pixel) {
    case 1:
    case 2:
    case 4:
    case 8:
        info->var.red.offset = info->var.green.offset = info->var.blue.offset = 0;
        info->var.red.length = info->var.green.length = info->var.blue.length = 8;
        break;

    case 16:
        info->var.red.offset      =       11;
        info->var.red.length      =       5;
        info->var.green.offset    =       5;
        info->var.green.length    =       6;
        info->var.blue.offset     =       0;
        info->var.blue.length     =       5;
        info->var.transp.offset   =       0;
        info->var.transp.length   =       0;
        break;
    default:
        dev_err(dev, "color depth %d not supported\n",
                info->var.bits_per_pixel);
        break;
    }

    info->flags = FBINFO_FLAG_DEFAULT | FBINFO_VIRTFB;

    fbdefio->delay = HZ / display.fps;
    fbdefio->deferred_io = ili9488_deferred_io;
    fb_deferred_io_init(info);

    /* ili9488 self setup */
    par = info->par;
    info->pseudo_palette = &par->pseudo_palette;

    par->fbinfo = info;
    par->dev = dev;

    par->buf = devm_kzalloc(dev, 128, GFP_KERNEL);
    if (!par->buf) {
        dev_err(dev, "failed to alloc buf memory!\n");
        return -ENOMEM;
    }

    // par->txbuf.buf = devm_kzalloc(dev, PAGE_SIZE, GFP_KERNEL);
    // if (!par->txbuf.buf) {
    //     dev_err(dev, "failed to alloc txbuf!\n");
    //     return -ENOMEM;
    // }
    // par->txbuf.len = PAGE_SIZE;

    par->tftops = &default_ili9488_ops;
    par->display = &display;

    dev_set_drvdata(dev, par);
    platform_set_drvdata(pdev, par);

    spin_lock_init(&par->dirty_lock);
    init_completion(&par->complete);
    ili9488_of_config(par);
    ili9488_hw_init(par);

    update_display(par, 0, par->fbinfo->var.yres - 1);
    /* framebuffer register */
    rc = register_framebuffer(info);
    if (rc < 0) {
        dev_err(dev, "framebuffer register failed with %d!\n", rc);
        goto alloc_fail;
    }

    printk("%zu KB buffer memory\n", par->txbuf.len >> 10);
    printk("%d KB video memory\n", info->fix.smem_len >> 10);

    return 0;

alloc_fail:
    vfree(vmem);
    return 0;
}

static int ili9488_remove(struct platform_device *pdev)
{
    struct ili9488_par *par = platform_get_drvdata(pdev);

    printk("%s\n", __func__);
    fb_deferred_io_cleanup(par->fbinfo);

    unregister_framebuffer(par->fbinfo);
    framebuffer_release(par->fbinfo);

    par->tftops->clear(par);
    return 0;
}

static int __maybe_unused ili9488_runtime_suspend(struct device *dev)
{
    // struct ili9488_par *par = dev_get_drvdata(dev);

    // par->tftops->sleep(par, true);

    return 0;
}

static int __maybe_unused ili9488_runtime_resume(struct device *dev)
{
    // struct ili9488_par *par = dev_get_drvdata(dev);

    // par->tftops->sleep(par, false);

    return 0;
}

static int __maybe_unused ili9488_runtime_idle(struct device *dev)
{
    // struct ili9488_par *par = dev_get_drvdata(dev);

    // par->tftops->idle(par, true);

    return 0;
}

static const struct of_device_id ili9488_dt_ids[] = {
    { .compatible = "ilitek,ili9488" },
    { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(of, ili9488_dt_ids);

#if CONFIG_PM
static const struct dev_pm_ops ili9488_pm_ops = {
    SET_RUNTIME_PM_OPS(ili9488_runtime_suspend,
                       ili9488_runtime_resume,
                       ili9488_runtime_idle)
};
#else
static const struct dev_pm_ops ili9488_pm_ops = {
    SET_RUNTIME_PM_OPS(NULL, NULL, NULL)
};
#endif

static struct platform_driver ili9488_plat_drv = {
    .probe    = ili9488_probe,
    .remove   = ili9488_remove,
    .driver   = {
        .name           = DRV_NAME,
        .of_match_table = of_match_ptr(ili9488_dt_ids),
        // .pm             = &ili9488_pm_ops
    },
};

module_platform_driver(ili9488_plat_drv);

MODULE_AUTHOR("embeddedboys <writeforever@foxmail.com>");
MODULE_DESCRIPTION("ili9488 based 16-Bit 8080 LCD-TFT display framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ili9488");
