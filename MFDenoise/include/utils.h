#ifndef _UTILS_H_
#define __UTILS_H_
#include <stdio.h>
#include <hardware/rga.h>

int rga_copy_and_scale(int src_w, int src_h, int src_addr, int src_fmt, int dst_w, int dst_h, int dst_addr, int dst_fmt);
#endif
