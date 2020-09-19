#include "menbashou_pose.h"

using namespace std;
using namespace cv;

#define PI 3.1415926
int menbashou_pose(Mat image,float weizi[6])
{
     int flag=0;
    Mat Src,Src_save,jiance,jiance_clone,zhanshi,zhanshi_1,zhanshi_2,HLStu,lvbo,erzhi_zsy,erzhi_gd,erzhi_gd_countour;
     Src=image;
    if(Src.cols == 0)
        cout<<"kong"<<endl;
    imshow("Src111",Src);
    Src_save=Src.clone();
    Size2f image_size,jiance_size;
    image_size.width = Src.cols;
    image_size.height =Src.rows;
    Point2f left_t, right_d;
    left_t.x=0;
    left_t.y=100;
    right_d.x=2200;
    right_d.y=1800;
    jiance=Src.clone();
    jiance=jiance(Rect(left_t.x,left_t.y, right_d.x, right_d.y));
    imshow("jiance",jiance);
    jiance_size.width = jiance.cols;
    jiance_size.height =jiance.rows;
    cvtColor(jiance,HLStu,CV_BGR2HLS);
    //cvtColor(jiance, graytu, CV_BGR2GRAY);
    medianBlur(HLStu, lvbo, 5);//中值滤波

    //int blockSize = 25;
    //int constValue = -5;
    //adaptiveThreshold(lvbo, erzhi_zsy, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, blockSize, constValue);
    //threshold(lvbo, erzhi_gd, 80, 255, CV_THRESH_BINARY);

    //zhanshi=erzhi_zsy.clone();
    //resize(erzhi_zsy, zhanshi, Size2f(0.5*jiance_size.width,0.5*jiance_size.height));

    int low_H = 50, low_L = 10, low_S = 0;
    int high_H = 170, high_L = 120, high_S = 50;//max:179,255,255
    inRange(lvbo,Scalar(low_H,low_L,low_S),Scalar(high_H,high_L,high_S),erzhi_gd);
    imshow("erzhi",erzhi_gd);
    resize(erzhi_gd, zhanshi, Size2f(0.3*jiance_size.width,0.3*jiance_size.height));
    imshow("zhanshi",zhanshi);

    vector<vector<Point>> contours;
    vector<vector<Point>> contourstrue;
    vector<Vec4i> hierarchy;
    vector<Vec4i> hierarchytrue;
    findContours(erzhi_gd,contours,hierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_NONE,Point(0.0f,0.0f));
    cout<<"contours.size():  "<<contours.size()<<endl;
    //erzhi_gd_countour=erzhi_gd.clone();

    for(int i=0;i<contours.size();i++)
    {
        float tmparea=fabs(contourArea(contours[i]));
        //if (contours[i].size()>1  && tmparea>100000)
        if (tmparea>3000 && tmparea<15000)
        {
            contourstrue.push_back(contours[i]);
            cout<<"tmparea:  "<<tmparea<<endl;
            cout<<"contours[i].size():  "<<contours[i].size()<<endl;
        }
    }
    contours.clear();
    contours.swap(contourstrue);
    contourstrue.clear();
    erzhi_gd_countour=Mat::zeros(erzhi_gd.size(),CV_8UC3);
    for (int i = 0; i < contours.size(); i++)
    {
        drawContours(erzhi_gd_countour,contours,i,Scalar(255,255,255),1,8);

    }
    vector<RotatedRect> box_zu;
    for (int i = 0; i < contours.size(); i++)
    {
        RotatedRect box = fitEllipse(contours[i]);//椭圆拟合
        Point2f center;//矩形的质心
        Size2f size;//矩形的边长
        float angle;//旋转角度
        center.x=box.center.x;
        center.y=box.center.y;
        size.width=box.size.width;
        size.height=box.size.height;
        angle=box.angle;
        float ellipse_area=PI * size.width * size.height/4;//椭圆面积
        float tmparea=fabs(contourArea(contours[i]));//轮廓面积
        //ellipse(erzhi_gd_countour, box, Scalar(0,0,255), 1, CV_AA);
        cout<<"tmparea:  "<<tmparea<<"  ellipse_area:  "<<ellipse_area<<endl;
        if (tmparea > ellipse_area * 0.8 && tmparea < ellipse_area * 1.8 && size.width/size.height>0.6 && size.width/size.height<1.4)
        {
            contourstrue.push_back(contours[i]);
            box_zu.push_back(box);
            ellipse(erzhi_gd_countour, box, Scalar(0,0,255), 1, CV_AA);
        }
    }
    contours.clear();
    contours.swap(contourstrue);
    contourstrue.clear();
    RotatedRect box_end;
    vector<Point> contour_end;
    if (contours.size() > 1)//如果选择的结果大于1个，通过长短轴比值寻找最圆的圆
    {
        float scale_q, scale_h;
        for (int i=0;i<(contours.size()-1);i++)
        {
            scale_q=box_zu[i].size.width / box_zu[i].size.height +box_zu[i].size.height / box_zu[i].size.width ;
            scale_h=box_zu[i+1].size.width / box_zu[i+1].size.height +box_zu[i+1].size.height / box_zu[i+1].size.width ;
            if (scale_q > scale_h)
            {
                box_end=box_zu[i+1];
                contour_end=contours[i+1];
            }
            else
            {
                box_end=box_zu[i];
                contour_end=contours[i];
            }
        }
        flag=1;
    }
    else if (contours.size() == 1)
    {
        box_end=box_zu[0];
        contour_end=contours[0];
        flag=1;
    }
    else
    {
        cout<<"  未检测到目标  "<<endl;
        return flag;
    }
    resize(erzhi_gd_countour, zhanshi_1, Size2f(0.3*jiance_size.width,0.3*jiance_size.height));
    imshow("zhanshi_1",zhanshi_1);
    box_end.center.x=box_end.center.x+left_t.x;
    box_end.center.y=box_end.center.y+left_t.y;
    ellipse(Src_save, box_end, Scalar(255,0,0), 3, CV_AA);
    resize(Src_save, zhanshi_2, Size2f(0.3*image_size.width,0.3*image_size.height));
    imshow("zhanshi_2",zhanshi_2);
    //waitKey();
    process(box_end.center.x,box_end.center.y, box_end.size.width,box_end.size.height,box_end.angle,weizi);
    cout<<"weizi:  "<<weizi[0]<<"  "<<weizi[1]<<"  "<<weizi[2]<<"  "<<weizi[3]<<"  "<<weizi[4]<<"  "<<weizi[5]<<endl;
    //waitKey();
    return flag;
}
