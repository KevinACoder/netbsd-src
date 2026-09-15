#ifndef _RTW89_LINUX_FIRMWARE_H_
#define _RTW89_LINUX_FIRMWARE_H_
#include "rtw89_compat.h"

/*
 * firmware(9) bridge.  The sync request_firmware() loads the image straight
 * into kernel memory; the images are embedded in the miniroot / installed
 * under /libdata/firmware/if_rtw89 by the ramdisk list.
 */
struct device;

/* the Linux shape fw.c iterates (data/size) plus our free hook */
struct firmware {
	size_t			size;
	const u8		*data;
	void			*priv;
};

int	rtw89_request_firmware_nowait(const char *name, struct device *dev,
	    struct firmware **fw_out, void (*cont)(const struct firmware *,
	    void *), void *context);
int	rtw89_request_firmware(const struct firmware **fw_out, const char *name,
	    struct device *dev);
void	rtw89_release_firmware(const struct firmware *fw);

#define	request_firmware(pfw, name, dev)				\
	rtw89_request_firmware((pfw), (name), (dev))
#define	request_firmware_nowait(module, uevent, name, dev, gfp, context, cont) \
	rtw89_request_firmware_nowait((name), (dev), NULL, (cont), (context))
#define	release_firmware(fw)	rtw89_release_firmware(fw)
#define	firmware_request_nowarn(pfw, name, dev)				\
	rtw89_request_firmware((pfw), (name), (dev))

#endif
