#include "../include/format.h"



Format::Format(vector <string> &_pathVec,int _width,int _height)
{
    int n = _pathVec.size();
    for(int i = 0;i < n;i++)
    {
        pathVec.push_back(_pathVec[i]);
    }
    width = _width;
    height = _height;
    buffSize = width*height*1.5;
}

Format::~Format()
{

}


bool Format::getYchannel(vector <Mat> &_outImg)
{
    bool ret = true;
    int n = pathVec.size();
    byte* buf = new byte [buffSize+1];

    for(int i = 0;i < n;i++)
    {
        memset(buf,0,sizeof(buf));
        ifstream readPic(pathVec[i].c_str(),ios::binary|ios::in);  // 打开并读yuv数据
        if(!readPic.is_open())
        {
            cout<<"Can not open picture !";
            return false;
        }

        readPic.read((char*)buf,buffSize);
        readPic.close();

        Mat gray(Size(width, height),CV_8UC1,buf);//CV_16UC1
        Mat out = gray.clone();
        _outImg.push_back(out);
//        imwrite("Ychannel.jpg",_outImg[i]);
    }

    delete []buf;
    return ret;
}

bool Format::decode(vector <Mat> &_outImg)
{
    bool ret = true;
    int n = pathVec.size();
    unsigned char* buf = new byte [width*height+width/2*height/2+width/2*height/2+1];
    unsigned char* dst = new byte [width*height+width/2*height/2+width/2*height/2+1];
    memset(dst,0,sizeof(dst));
    for(int i = 0;i < n;i++)
    {
        ifstream readPic(pathVec[i].c_str(), ios::in | ios::binary);  // 打开并读yuv数据
        readPic.read((char*)buf,width*height+width/2*height/2+width/2*height/2);
        readPic.close();

        Mat out;
        yvu420ps2rgba(buf,out);

        _outImg.push_back(out);
        //        imwrite("format.jpg",out);
    }

    delete []buf;
    delete []dst;
    return ret;
}

bool Format::yvu420ps2rgba( byte yuv420sp[],Mat &out)
{

    bool ret = true;
    const int frameSize = width * height;
    unsigned int *rgb = new unsigned int[frameSize];
    memset(rgb,0,frameSize);

    for (int j = 0, yp = 0; j < height; j++)
    {
        int uvp = frameSize + (j >> 1) * width, u = 0, v = 0;
        for (int i = 0; i < width; i++, yp++)
        {
            int y = (0xff & ((int) yuv420sp[yp])) - 16;
            if (y < 0)
                y = 0;
            if ((i & 1) == 0)
            {
                v = (0xff & yuv420sp[uvp++]) - 128;
                u = (0xff & yuv420sp[uvp++]) - 128;
            }

            int y1192 = 1192 * y;
            int r = (y1192 + 1634 * v);
            int g = (y1192 - 833 * v - 400 * u);
            int b = (y1192 + 2066 * u);

            const  int maxValue = 262143;//   pow(2,18) - 1
            if (r < 0)
                r = 0;
            else if (r > maxValue)
                r = maxValue;
            if (g < 0)
                g = 0;
            else if (g > maxValue)
                g = maxValue;
            if (b < 0)
                b = 0;
            else if (b > maxValue)
                b = maxValue;

            rgb[yp] = 0xff000000 | ((r << 6) & 0xff0000) | ((g >> 2) & 0xff00) | ((b >> 10) & 0xff);
        }
    }

    Mat rgbImg(Size(width, height),CV_8UC4,rgb);
    out = rgbImg.clone();

    delete []rgb;
    return ret;
}

//void Format::yv12Tnv12(byte src[],byte des[])
//{
//    int wh = width * height;
//    //旋转Y
//    int k = 0;
//    for(int i=0;i<width;i++)
//    {
//        for(int j=0;j<height;j++)
//        {
//            des[k] = src[width*j + i];
//            k++;
//        }
//    }

//    for(int i=0;i<width;i+=2)
//    {
//        for(int j=0;j<height/2;j++)
//        {
//            des[k] = src[wh+ width*j + i];
//            des[k+1]=src[wh + width*j + i+1];
//            k+=2;
//        }
//    }
//    return ;
//}

//bool Format::YUV420_To_BGR24(unsigned char *puc_y, unsigned char *puc_u,
//                             unsigned char *puc_v, unsigned char *puc_rgb, int width_y, int height_y)
//{
//    if (!puc_y || !puc_u || !puc_v || !puc_rgb)
//    {
//        return false;
//    }

//    //初始化变量
//    int baseSize = width_y * height_y;
//    int rgbSize = baseSize * 3;

//    byte* rgbData  = new byte[rgbSize];
//    memset(rgbData, 0, rgbSize);

//    /* 变量声明 */
//    int temp = 0;

//    byte* rData = rgbData;                  //r分量地址
//    byte* gData = rgbData + baseSize;       //g分量地址
//    byte* bData = gData   + baseSize;       //b分量地址

//    int uvIndex =0, yIndex =0;

//    //YUV->RGB 的转换矩阵
//    //double  Yuv2Rgb[3][3] = {1, 0, 1.4022,
//    //    1, -0.3456, -0.7145,
//    //    1, 1.771,   0};

//    for(int y=0; y < height_y; y++)
//    {
//        for(int x=0; x < width_y; x++)
//        {
//            uvIndex        = (y>>1) * (width_y>>1) + (x>>1);
//            yIndex         = y * width_y + x;

//            /* r分量 */
//            temp          = (int)(puc_y[yIndex] + (puc_v[uvIndex] - 128) * 1.4022);
//            rData[yIndex] = temp<0 ? 0 : (temp > 255 ? 255 : temp);

//            /* g分量 */
//            temp          = (int)(puc_y[yIndex] + (puc_u[uvIndex] - 128) * (-0.3456) +
//                                  (puc_v[uvIndex] - 128) * (-0.7145));
//            gData[yIndex] = temp < 0 ? 0 : (temp > 255 ? 255 : temp);

//            /* b分量 */
//            temp          = (int)(puc_y[yIndex] + (puc_u[uvIndex] - 128) * 1.771);
//            bData[yIndex] = temp < 0 ? 0 : (temp > 255 ? 255 : temp);
//        }
//    }

//    //将R,G,B三个分量赋给img_data
//    int widthStep = width_y*3;
//    for (int y = 0; y < height_y; y++)
//    {
//        for (int x = 0; x < width_y; x++)
//        {
//            puc_rgb[y * widthStep + x * 3 + 2] = rData[y * width_y + x];   //R
//            puc_rgb[y * widthStep + x * 3 + 1] = gData[y * width_y + x];   //G
//            puc_rgb[y * widthStep + x * 3 + 0] = bData[y * width_y + x];   //B
//        }
//    }

//    if (!puc_rgb)
//    {
//        return false;
//    }
//    delete [] rgbData;
//    return true;
//}

//Mat Format::YUV420_To_Mat(unsigned char* pYUV420, int width, int height)
//{
//    if (!pYUV420)
//    {
//        assert("pYUV420 is NULL");
//    }

//    //初始化变量
//    int baseSize = width*height;
//    int imgSize = baseSize*3;
//    byte* pRGB24  = new byte[imgSize];

//    memset(pRGB24,  0, imgSize);

//    byte* yData = pYUV420;                           //y分量地址
//    byte* uData = pYUV420 + baseSize;       //u分量地址
//    byte* vData = uData  + (baseSize>>2);   //v分量地址

//    if(YUV420_To_BGR24(yData, uData, vData, pRGB24, width, height) == false || !pRGB24)
//    {
//        assert("YUV420_To_BGR24 has failed");
//    }

//    Mat rgbImg(Size(width, height),CV_8UC3,pRGB24);
//    Mat out = rgbImg.clone();

//    delete [] pRGB24;
//    return out;
//}


