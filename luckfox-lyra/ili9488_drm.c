#include "drm/drm_print.h"
#include "drm/drm_simple_kms_helper.h"
#include "linux/compiler_attributes.h"
#include "linux/dev_printk.h"
#include "linux/err.h"
#include <linux/init.h>
#include <linux/acpi.h>
#include <linux/backlight.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_connector.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_modes.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_rect.h>
#include <video/mipi_display.h>

#include <video/mipi_display.h>

#define DRV_NAME "ili9488_drm_drv"

struct ili9488_priv;

struct ili9488_operations {
    int (*reset)(struct ili9488_priv *priv);
    int (*clear)(struct ili9488_priv *priv);
    int (*idle)(struct ili9488_priv *priv, bool on);
    int (*blank)(struct ili9488_priv *priv, bool on);
    int (*sleep)(struct ili9488_priv *priv, bool on);
    int (*set_addr_win)(struct ili9488_priv *priv, int xs, int ys, int xe, int ye);
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

struct ili9488_priv {

    u64 dma_mask;
    struct device           *dev;
    u8                      *buf;

    struct {
        struct gpio_desc *reset;
        struct gpio_desc *dc;
        struct gpio_desc *cs;
        struct gpio_desc *blk;
        struct gpio_desc *rd;
        struct gpio_desc *wr;
        struct gpio_desc *db[16];
    } gpio;

    const struct ili9488_operations        *tftops;
    const struct ili9488_display           *display;

    /* DRM specific data */
    u16 *tx_buf;
    u32 pixel_format;
    struct drm_device drm;
    struct drm_simple_display_pipe pipe;
    struct drm_connector connector;
    struct drm_display_mode mode;
};

#define gpio_put(d, v) gpiod_set_raw_value(d, v)
int fbtft_write_gpio16_wr(struct ili9488_priv *priv, void *buf, size_t len)
{
	volatile u16 data;
	int i;
#ifndef DO_NOT_OPTIMIZE_FBTFT_WRITE_GPIO
	static volatile u16 prev_data;
#endif

	while (len) {
		data = *(u16 *)buf;

		/* Start writing by pulling down /WR */
		gpio_put(priv->gpio.wr, 0);

		/* Set data */
#ifndef DO_NOT_OPTIMIZE_FBTFT_WRITE_GPIO
		if (data == prev_data) {
			gpio_put(priv->gpio.wr, 1); /* used as delay */
		} else {
			for (i = 0; i < 16; i++) {
				if ((data & 1) != (prev_data & 1))
					gpio_put(priv->gpio.db[i],
							data & 1);
				data >>= 1;
				prev_data >>= 1;
			}
		}
#else
		for (i = 0; i < 16; i++) {
			gpio_put(priv->gpio.db[i], data & 1);
			data >>= 1;
		}
#endif
		/* Pullup /WR */
		gpio_put(priv->gpio.wr, 1);

#ifndef DO_NOT_OPTIMIZE_FBTFT_WRITE_GPIO
		prev_data = *(u16 *)buf;
#endif
		buf += 2;
		len -= 2;
	}

	return 0;
}

static inline void fbtft_write_buf_dc(struct ili9488_priv *priv, void *buf, size_t len, int dc)
{
    gpio_put(priv->gpio.dc, dc);
    fbtft_write_gpio16_wr(priv, buf, len);
}

#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__}) / sizeof(int))
static int ili9488_write_reg(struct ili9488_priv *priv, int len, ...)
{
    u16 *buf = (u16 *)priv->buf;
    va_list args;
    int i;

    va_start(args, len);

    *buf = (u16)va_arg(args, unsigned int);
    fbtft_write_buf_dc(priv, buf, sizeof(u16), 0);
    len--;

    /* if there no privams */
    if (len == 0)
        goto exit_no_privam;

    for (i = 0; i < len; i++)
        *buf++ = (u16)va_arg(args, unsigned int);

    len *= 2;
    fbtft_write_buf_dc(priv, priv->buf, len, 1);
    va_end(args);

exit_no_privam:
    va_end(args);
    return 0;
}
#define write_reg(priv, ...) \
    ili9488_write_reg(priv, NUMARGS(__VA_ARGS__), __VA_ARGS__)

static int ili9488_reset(struct ili9488_priv *priv)
{
    gpio_put(priv->gpio.reset, 1);
    mdelay(10);
    gpio_put(priv->gpio.reset, 0);
    mdelay(10);
    gpio_put(priv->gpio.reset, 1);
    mdelay(10);
    return 0;
}

static int ili9488_init_display(struct ili9488_priv *priv)
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

static int __maybe_unused ili9488_blank(struct ili9488_priv *priv, bool on)
{
    if (on)
        write_reg(priv, MIPI_DCS_SET_DISPLAY_OFF);
    else
        write_reg(priv, MIPI_DCS_SET_DISPLAY_ON);
    return 0;
}

static int ili9488_set_addr_win(struct ili9488_priv *priv, int xs, int ys, int xe,
                                int ye)
{
    dev_dbg(priv->dev, "xs = %d, xe = %d, ys = %d, ye = %d\n", xs, xe, ys, ye);

    write_reg(priv, MIPI_DCS_SET_COLUMN_ADDRESS,
              ((xs >> BITS_PER_BYTE)), (xs & 0xFF),
              ((xe >> BITS_PER_BYTE)), (xe & 0xFF));

    write_reg(priv, MIPI_DCS_SET_PAGE_ADDRESS,
              ((ys >> BITS_PER_BYTE)), (ys & 0xFF),
              ((ye >> BITS_PER_BYTE)), (ye & 0xFF));

    write_reg(priv, MIPI_DCS_WRITE_MEMORY_START);

    return 0;
}

// static int ili9488_idle(struct ili9488_priv *priv, bool on)
// {
//     if (on)
//         write_reg(priv, MIPI_DCS_EXIT_IDLE_MODE);
//     else
//         write_reg(priv, MIPI_DCS_EXIT_IDLE_MODE);

//     return 0;
// }

// static int ili9488_sleep(struct ili9488_priv *priv, bool on)
// {
//     if (on) {
//         write_reg(priv, MIPI_DCS_SET_DISPLAY_OFF);
//         write_reg(priv, MIPI_DCS_ENTER_SLEEP_MODE);
//     } else {
//         write_reg(priv, MIPI_DCS_EXIT_SLEEP_MODE);
//         write_reg(priv, MIPI_DCS_SET_DISPLAY_ON);
//     }

//     return 0;
// }

static int ili9488_clear(struct ili9488_priv *priv)
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

static int ili9488_request_one_gpio(struct ili9488_priv *priv,
                                    const char *name, int index,
                                    struct gpio_desc **gpiop)
{
#if 0
    struct device *dev = priv->dev;

    *gpiop = devm_gpiod_get_index(dev, name, index, GPIOD_OUT_LOW);
    if (IS_ERR(*gpiop)) {
        dev_err(dev, "Failed to request %s gpio, index %d\n", name, index);
        return PTR_ERR(*gpiop);
    }

    return 0;
#else
    struct device *dev = priv->dev;
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
#endif
}

static int ili9488_request_gpios(struct ili9488_priv *priv)
{
    int rc;
    int i;

    rc = ili9488_request_one_gpio(priv, "reset", 0, &priv->gpio.reset);
    if (rc)
        return rc;
    rc = ili9488_request_one_gpio(priv, "dc", 0, &priv->gpio.dc);
    if (rc)
        return rc;
    rc = ili9488_request_one_gpio(priv, "wr", 0, &priv->gpio.wr);
    if (rc)
        return rc;
    rc = ili9488_request_one_gpio(priv, "led", 0, &priv->gpio.blk);
    if (rc)
        return rc;

    for (i = 0; i < 16; i++) {
		rc = ili9488_request_one_gpio(priv, "db", i,
					     &priv->gpio.db[i]);
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

static int ili9488_of_config(struct ili9488_priv *priv)
{
    int rc;

    rc = ili9488_request_gpios(priv);
    if (rc) {
        dev_err(priv->dev, "Request gpios failed!\n");
        return rc;
    }
    return 0;

    /* request xres and yres from dt */
}

// #define MADCTL_BGR BIT(3) /* bitmask for RGB/BGR order */
// #define MADCTL_MV BIT(5) /* bitmask for page/column order */
// #define MADCTL_MX BIT(6) /* bitmask for column address order */
// #define MADCTL_MY BIT(7) /* bitmask for page address order */
// static int ili9488_set_var(struct ili9488_priv *priv)
// {
//     u8 madctl_priv = 0;

//     switch (priv->fbinfo->var.rotate) {
//     case 0:
//         break;
//     case 90:
//         madctl_priv |= (MADCTL_MV | MADCTL_MY);
//         break;
//     case 180:
//         madctl_priv |= (MADCTL_MX | MADCTL_MY);
//         break;
//     case 270:
//         madctl_priv |= (MADCTL_MV | MADCTL_MX);
//         break;
//     default:
//         return -EINVAL;

//     }

//     write_reg(priv, MIPI_DCS_SET_ADDRESS_MODE, madctl_priv);
//     return 0;
// }

static int ili9488_hw_init(struct ili9488_priv *priv)
{
    printk("%s, Display Panel initializing ...\n", __func__);
    ili9488_init_display(priv);

    if (priv->gpio.blk)
        gpio_put(priv->gpio.blk, 1);
    // ili9488_set_var(priv);
    // ili9488_set_gamma(priv, default_curves);
    // ili9488_clear(priv);

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

static inline struct ili9488_priv *drm_to_ili9488(struct drm_device *drm)
{
    return container_of(drm, struct ili9488_priv, drm);
}

static enum drm_mode_status ili9488_mode_valid(struct drm_simple_display_pipe *pipe,
					                           const struct drm_display_mode *mode)
{
    struct ili9488_priv *priv = drm_to_ili9488(pipe->crtc.dev);

    return drm_crtc_helper_mode_valid_fixed(&pipe->crtc, mode, &priv->mode);
}

static void ili9488_pipe_enable(struct drm_simple_display_pipe *pipe,
				struct drm_crtc_state *crtc_state,
				struct drm_plane_state *plane_state)
{
    struct ili9488_priv *priv = drm_to_ili9488(pipe->crtc.dev);

    DRM_DEBUG_KMS("\n");
    ili9488_hw_init(priv);
    ili9488_clear(priv);
}

static void ili9488_pipe_disable(struct drm_simple_display_pipe *pipe)
{
    DRM_DEBUG_KMS("\n");
}

static int ili9488_buf_copy(void *dst, struct drm_framebuffer *fb,
		      struct drm_rect *clip, bool swap)
{
    struct drm_gem_object *gem = drm_gem_fb_get_obj(fb, 0);
	struct iosys_map map[DRM_FORMAT_MAX_PLANES];
	struct iosys_map data[DRM_FORMAT_MAX_PLANES];
	struct iosys_map dst_map = IOSYS_MAP_INIT_VADDR(dst);
	int ret;

	ret = drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE);
	if (ret)
		return ret;

	ret = drm_gem_fb_vmap(fb, map, data);
	if (ret)
		goto out_drm_gem_fb_end_cpu_access;

	switch (fb->format->format) {
	case DRM_FORMAT_RGB565:
		if (swap)
			drm_fb_swab(&dst_map, NULL, data, fb, clip, !gem->import_attach);
		else
			drm_fb_memcpy(&dst_map, NULL, data, fb, clip);
		break;
	case DRM_FORMAT_XRGB8888:
		drm_fb_xrgb8888_to_rgb565(&dst_map, NULL, data, fb, clip, swap);
		break;
	default:
		drm_err_once(fb->dev, "Format is not supported: %p4cc\n",
			     &fb->format->format);
		ret = -EINVAL;
	}

	drm_gem_fb_vunmap(fb, map);
out_drm_gem_fb_end_cpu_access:
	drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);

	return ret;
}

static void ili9488_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect)
{
    struct ili9488_priv *priv = drm_to_ili9488(fb->dev);
    unsigned int height = rect->y2 - rect->y1;
    unsigned int width = rect->x2 - rect->x1;
    bool swap = false;
    int ret = 0;
    bool full;
    void *tr;

    full = width == fb->width && height == fb->height;

    tr = priv->tx_buf;

    ret = ili9488_buf_copy(tr, fb, rect, swap);
    if (ret) {
        DRM_DEBUG_KMS("err on buffer copy!\n");
        return;
    }

    ili9488_set_addr_win(priv, rect->x1, rect->y1, rect->x2, rect->y2);

    gpio_put(priv->gpio.dc, 1);
    fbtft_write_gpio16_wr(priv, tr, width * height * 2);
}

static void ili9488_pipe_update(struct drm_simple_display_pipe *pipe,
                                struct drm_plane_state *old_state)
{
    struct drm_plane_state *state = pipe->plane.state;
    struct drm_framebuffer *fb = state->fb;
    struct drm_rect rect;
    int idx;

    DRM_DEBUG_KMS("\n");
    if (!pipe->crtc.state->active)
        return;

    if (WARN_ON(!fb))
		return;

	if (!drm_dev_enter(fb->dev, &idx))
		return;

    if (drm_atomic_helper_damage_merged(old_state, state, &rect))
        ili9488_fb_dirty(fb, &rect);

    drm_dev_exit(idx);
}

static const struct ili9488_display display = {
    .xres = 480,
    .yres = 320,
    .bpp = 16,
    .fps = 60,
};

static const struct drm_simple_display_pipe_funcs ili9488_pipe_funcs = {
    .mode_valid = ili9488_mode_valid,
    .enable = ili9488_pipe_enable,
    .disable = ili9488_pipe_disable,
    .update = ili9488_pipe_update,
};

static int ili9488_connector_get_modes(struct drm_connector *connector)
{
	struct ili9488_priv *priv = drm_to_ili9488(connector->dev);

	return drm_connector_helper_get_modes_fixed(connector, &priv->mode);
}

static const struct drm_connector_helper_funcs priv_connector_hfuncs = {
	.get_modes = ili9488_connector_get_modes,
};

static const struct drm_connector_funcs priv_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_mode_config_funcs ili9488_mode_config_funcs = {
    .fb_create = drm_gem_fb_create_with_dirty,
    .atomic_check = drm_atomic_helper_check,
    .atomic_commit = drm_atomic_helper_commit,
};

static const uint32_t ili9488_formats[] = {
    DRM_FORMAT_RGB565,
    DRM_FORMAT_XRGB8888,
};

static const struct drm_display_mode ili9488_disp_mode = {
    DRM_MODE_INIT(30, 480, 320, 85, 55),
};

DEFINE_DRM_GEM_DMA_FOPS(ili9488_fops);

static const struct drm_driver ili9488_drm_driver = {
    .driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
    .fops = &ili9488_fops,
    DRM_GEM_DMA_DRIVER_OPS_VMAP,
    .name = "ili9488",
    .desc = "Ilitek ILI9488 DRM Driver",
    .date = "20250506",
    .major = 0,
    .minor = 1,
    .patchlevel = 0,
};

static int ili9488_drm_dev_init_with_formats(struct ili9488_priv *priv,
                const struct drm_simple_display_pipe_funcs *funcs,
                const uint32_t *formats, unsigned int formats_count,
                const struct drm_display_mode *mode, size_t tx_buf_size)
{
    static const uint64_t modifiers[] = {
		DRM_FORMAT_MOD_LINEAR,
		DRM_FORMAT_MOD_INVALID
	};
    struct drm_device *drm = &priv->drm;
    int rc;

    pr_info("%s\n", __func__);

    rc = drm_mode_config_init(drm);
    if (rc) {
        pr_err("failed to init mode config\n");
        return rc;
    }

    priv->tx_buf = devm_kmalloc(drm->dev, tx_buf_size, GFP_KERNEL);
    if (!priv->tx_buf)
        return -ENOMEM;

    drm_mode_copy(&priv->mode, mode);
    pr_info("mode: %ux%u\n", priv->mode.hdisplay, priv->mode.vdisplay);

    drm_connector_helper_add(&priv->connector, &priv_connector_hfuncs);
    rc = drm_connector_init(drm, &priv->connector, &priv_connector_funcs,
                            DRM_MODE_CONNECTOR_USB);
    if (rc) {
        pr_err("failed to init connector\n");
        return rc;
    }

    rc = drm_simple_display_pipe_init(drm, &priv->pipe, funcs, formats, formats_count, modifiers, &priv->connector);
    if (rc) {
        pr_err("failed to init pipe\n");
        return rc;
    }

    drm_plane_enable_fb_damage_clips(&priv->pipe.plane);

    drm->mode_config.funcs = &ili9488_mode_config_funcs;
    drm->mode_config.min_width = priv->mode.hdisplay;
    drm->mode_config.max_width = priv->mode.hdisplay;
    drm->mode_config.min_height = priv->mode.vdisplay;
    drm->mode_config.max_height = priv->mode.vdisplay;
    priv->pixel_format = formats[0];

    DRM_DEBUG_KMS("mode: %ux%u", priv->mode.hdisplay, priv->mode.vdisplay);

    return 0;
}

static int ili9488_drm_dev_init(struct ili9488_priv *priv,
                const struct drm_simple_display_pipe_funcs *funcs,
                const struct drm_display_mode *mode)
{
    ssize_t bufsize = mode->vdisplay * mode->hdisplay * sizeof(u16);

    priv->drm.mode_config.preferred_depth = 16;

    pr_info("%s\n", __func__);

    return ili9488_drm_dev_init_with_formats(priv, funcs, ili9488_formats,
                        ARRAY_SIZE(ili9488_formats), mode, bufsize);
}

static int ili9488_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    // int width, height, bpp, rotate;
    struct ili9488_priv *priv;
    struct drm_device *drm;
    int ret;

    printk("%s\n", __func__);

    priv = devm_drm_dev_alloc(dev, &ili9488_drm_driver, struct ili9488_priv, drm);
    if (IS_ERR(priv)) {
        printk("Failed to allocate memory for ili9488_priv\n");
        return PTR_ERR(priv);
    }

    platform_set_drvdata(pdev, priv);
    drm = &priv->drm;

    priv->dev = dev;
    priv->buf = devm_kzalloc(dev, 128, GFP_KERNEL);
    if (!priv->buf) {
        dev_err(dev, "failed to alloc buf memory!\n");
        return -ENOMEM;
    }
    priv->tftops = &default_ili9488_ops;
    priv->display = &display;

    ili9488_of_config(priv);

    ret = ili9488_drm_dev_init(priv, &ili9488_pipe_funcs, &ili9488_disp_mode);
    if (ret) {
        dev_err(dev, "failed to init drm dev\n");
        return -ENODEV;
    }

    drm_mode_config_reset(drm);

    ret = drm_dev_register(drm, 0);
    if (ret) {
        dev_err(dev, "failed to register drm device\n");
        return ret;
    }

    drm_fbdev_generic_setup(drm, 0);

    return 0;
}

static int ili9488_remove(struct platform_device *pdev)
{
    struct ili9488_priv *priv = platform_get_drvdata(pdev);
    struct drm_device *drm = &priv->drm;
    printk("%s\n", __func__);

    drm_dev_unplug(drm);
    drm_atomic_helper_shutdown(drm);
    return 0;
}

static const struct of_device_id ili9488_dt_ids[] = {
    { .compatible = "ilitek,ili9488" },
    { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(of, ili9488_dt_ids);

// #if CONFIG_PM
// static const struct dev_pm_ops ili9488_pm_ops = {
//     SET_RUNTIME_PM_OPS(ili9488_runtime_suspend,
//                        ili9488_runtime_resume,
//                        ili9488_runtime_idle)
// };
// #else
// static const struct dev_pm_ops ili9488_pm_ops = {
//     SET_RUNTIME_PM_OPS(NULL, NULL, NULL)
// };
// #endif

static struct platform_driver ili9488_plat_drv = {
    .driver   = {
        .name           = DRV_NAME,
        .of_match_table = of_match_ptr(ili9488_dt_ids),
        // .pm             = &ili9488_pm_ops
    },
    .probe    = ili9488_probe,
    .remove   = ili9488_remove,
};

module_platform_driver(ili9488_plat_drv);

MODULE_AUTHOR("Zheng Hua <hua.zheng@embeddedboys.com>");
MODULE_DESCRIPTION("ili9488 based 16-Bit 8080 bitbang LCD-TFT display DRM driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ili9488");
