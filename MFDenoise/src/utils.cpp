#include "include/utils.h"
#include <fcntl.h>

int rga_copy_and_scale(int src_w, int src_h, int src_addr, int src_fmt, int dst_w, int dst_h, int dst_addr, int dst_fmt)
{

    int gfd_rga = -1;
    struct rga_req  Rga_Request;
    memset(&Rga_Request,0x0,sizeof(Rga_Request));

    gfd_rga = open("/dev/rga",O_RDWR,0);
    if(gfd_rga < 0)
    {
        //ALOGE(" rga open err");
        return -1;
    }

    if (src_fmt != dst_fmt) {
        Rga_Request.yuv2rgb_mode |= 1<<4;
    }

    Rga_Request.src.uv_addr =  (int)src_addr;
    Rga_Request.src.vir_w = src_w;
    Rga_Request.src.vir_h = src_h;
    Rga_Request.src.format = src_fmt;

    Rga_Request.src.act_w = Rga_Request.src.vir_w;
    Rga_Request.src.act_h = Rga_Request.src.vir_h;
    Rga_Request.src.x_offset = 0;
    Rga_Request.src.y_offset = 0;

    Rga_Request.dst.uv_addr =(int)dst_addr;
    Rga_Request.dst.vir_w = dst_w;
    Rga_Request.dst.vir_h = dst_h;
    Rga_Request.dst.act_w = Rga_Request.dst.vir_w;
    Rga_Request.dst.act_h = Rga_Request.dst.vir_h;

    Rga_Request.dst.format = dst_fmt;
    Rga_Request.clip.xmin = 0;
    Rga_Request.clip.xmax = Rga_Request.dst.vir_w - 1;
    Rga_Request.clip.ymin = 0;
    Rga_Request.clip.ymax = Rga_Request.dst.vir_h - 1;
    Rga_Request.dst.x_offset = 0;
    Rga_Request.dst.y_offset = 0;

    Rga_Request.sina = 0;
    Rga_Request.cosa = 0x10000;
	
    /*if(Rga_Request.src.act_w != Rga_Request.dst.act_w
        || Rga_Request.src.act_h != Rga_Request.dst.act_h)
    {*/
        Rga_Request.scale_mode = 1;
        Rga_Request.rotate_mode = 1;
    //}
    //Rga_Request.render_mode = pre_scaling_mode;
    Rga_Request.alpha_rop_flag |= (1 << 5);

    Rga_Request.mmu_info.mmu_en    = 1;
    Rga_Request.mmu_info.mmu_flag  = ((2 & 0x3) << 4) | 1;

    if(ioctl(gfd_rga, RGA_BLIT_SYNC, &Rga_Request) != 0)
    {
/*
        ALOGE("%s(%d):  RGA_BLIT_ASYNC Failed ", __FUNCTION__, __LINE__);
        ALOGE("src info: yrgb_addr=%x, uv_addr=%x,v_addr=%x,"
             "vir_w=%d,vir_h=%d,format=%d,"
             "act_x_y_w_h [%d,%d,%d,%d] ",
                Rga_Request.src.yrgb_addr, Rga_Request.src.uv_addr ,Rga_Request.src.v_addr,
                Rga_Request.src.vir_w ,Rga_Request.src.vir_h ,Rga_Request.src.format ,
                Rga_Request.src.x_offset ,
                Rga_Request.src.y_offset,
                Rga_Request.src.act_w ,
                Rga_Request.src.act_h
            );

        ALOGE("dst info: yrgb_addr=%x, uv_addr=%x,v_addr=%x,"
             "vir_w=%d,vir_h=%d,format=%d,"
             "clip[%d,%d,%d,%d], "
             "act_x_y_w_h [%d,%d,%d,%d] ",

                Rga_Request.dst.yrgb_addr, Rga_Request.dst.uv_addr ,Rga_Request.dst.v_addr,
                Rga_Request.dst.vir_w ,Rga_Request.dst.vir_h ,Rga_Request.dst.format,
                Rga_Request.clip.xmin,
                Rga_Request.clip.xmax,
                Rga_Request.clip.ymin,
                Rga_Request.clip.ymax,
                Rga_Request.dst.x_offset ,
                Rga_Request.dst.y_offset,
                Rga_Request.dst.act_w ,
                Rga_Request.dst.act_h

            );
*/
        close(gfd_rga);
        return -1;


    }
    close(gfd_rga);
    return 0;
}
	
