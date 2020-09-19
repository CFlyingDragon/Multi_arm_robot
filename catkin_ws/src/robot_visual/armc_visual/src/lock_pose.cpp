#include "lock_pose.h"

using namespace std;
using namespace cv;

int lock_pose(Mat image,float tag_T_cam[4][4])
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
    jiance=jiance(Rect2f(left_t.x,left_t.y, right_d.x, right_d.y));
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

    int low_H = 50, low_L = 120, low_S = 130;
    int high_H = 130, high_L = 230, high_S = 255;
    inRange(lvbo,Scalar(low_H,low_L,low_S),Scalar(high_H,high_L,high_S),erzhi_gd);
    imshow("erzhi",erzhi_gd);
    resize(erzhi_gd, zhanshi, Size2f(0.5*jiance_size.width,0.5*jiance_size.height));
    imshow("zhanshi",zhanshi);

    vector<vector<Point>> contours;
    vector<vector<Point>> contourstrue;
    vector<Vec4i> hierarchy;
    vector<Vec4i> hierarchytrue;
    findContours(erzhi_gd,contours,hierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_NONE,Point(0.0f,0.0f));
    //erzhi_gd_countour=erzhi_gd.clone();
    erzhi_gd_countour=Mat::zeros(erzhi_gd.size(),CV_8UC3);
    for(int i=0;i<contours.size();i++)
    {
        float tmparea=fabs(contourArea(contours[i]));
        //if (contours[i].size()>1  && tmparea>100000)
        if (tmparea>1000000 && tmparea<3000000)
        {
            contourstrue.push_back(contours[i]);
            cout<<"tmparea:  "<<tmparea<<endl;
            cout<<"contours[i].size():  "<<contours[i].size()<<endl;
        }
    }
    for (int i = 0; i < contourstrue.size(); i++)
    {
       drawContours(erzhi_gd_countour,contourstrue,i,Scalar(255,255,255),1,8);
    }
    resize(erzhi_gd_countour, zhanshi_1, Size2f(0.5*jiance_size.width,0.5*jiance_size.height));
    imshow("zhanshi_1",zhanshi_1);

    vector<Point2f> corner;
    bool isDetect;
    //*********矩形检测*************
    flag = RectDetect(erzhi_gd_countour, &isDetect,corner);//need the gray picture
    if ((isDetect == 1) && !(corner.empty()))
    //if ( !(corner.empty()))
    //if (isDetect == 1)
    {
       cout << "going to do for calc" << endl;
       for (int ct = 0; ct < 4; ct++)
       {
            corner[ct].x = corner[ct].x + left_t.x;
            corner[ct].y = corner[ct].y + left_t.y;
       }
        // draw the point in origin picture
       circle(Src_save, corner[0], 5, Scalar(225, 0, 225), 3, 8); //线宽为3，颜色为紫色
       cv::putText(Src_save, to_string(0), cv::Point(corner[0].x - 34, corner[0].y), CV_FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(0, 0, 255), 2);

       circle(Src_save, corner[1], 5, Scalar(225, 0, 225), 3, 8);
       cv::putText(Src_save, to_string(1), cv::Point(corner[1].x + 14, corner[1].y), CV_FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(0, 0, 255), 2);

       circle(Src_save, corner[2], 5, Scalar(225, 0, 225), 3, 8);
       cv::putText(Src_save, to_string(2), cv::Point(corner[2].x + 14, corner[2].y), CV_FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(0, 0, 255), 2);

       circle(Src_save, corner[3], 5, Scalar(225, 0, 225), 3, 8);
       cv::putText(Src_save, to_string(3), cv::Point(corner[3].x - 34, corner[3].y), CV_FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(0, 0, 255), 2);

       line(Src_save, Point(corner[0].x, corner[0].y), Point(corner[1].x, corner[1].y),Scalar(0, 255, 0),2);  //画出边缘图
       line(Src_save, Point(corner[1].x, corner[1].y), Point(corner[2].x, corner[2].y),Scalar(0, 255, 0),2);  //画出边缘图
       line(Src_save, Point(corner[2].x, corner[2].y), Point(corner[3].x, corner[3].y),Scalar(0, 255, 0),2);  //画出边缘图
       line(Src_save, Point(corner[3].x, corner[3].y), Point(corner[0].x, corner[0].y),Scalar(0, 255, 0),2);  //画出边缘图

       imwrite("origin_circle.bmp", Src_save);
       resize(Src_save, zhanshi_2, Size2f(0.4*image_size.width,0.4*image_size.height));
       imshow("zhanshi_2",zhanshi_2);
       //waitKey();

       // add PnP method
       //特征点的像素坐标
       cout << "\n\n\nenter pnp isDetect:" << isDetect << endl;
       cout << "**corner**" << corner << endl;
       //特征点世界坐标
       vector<cv::Point3f> Points3D;
       Points3D.push_back(cv::Point3f(-57.5, 45, 0));  //P1 三维坐标的单位是毫米
       Points3D.push_back(cv::Point3f(57.5, 45, 0));   //P2
       Points3D.push_back(cv::Point3f(57.5, -45, 0));  //P3
       Points3D.push_back(cv::Point3f(-57.5, -45, 0)); //P4

       //the parameter of camera
       double camD[9] = {1061.63809405752, 0, 669.823184750678,
                                    0, 1061.72911715432, 356.265577591959,
                                    0, 0, 1};
       cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64FC1, camD);
       //畸变参数
       double distCoeffD[5] = {-0.0533687268157436, 0.123316714357672, -0.00129228763177168, -0.00215893504838259, 0};
       //double转mat类型
       cv::Mat distCoeffs = cv::Mat(1, 5, CV_64FC1, distCoeffD);

       //初始化输出矩阵
       cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
       cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

       cv::solvePnP(Points3D, corner, cameraMatrix, distCoeffs, rvec, tvec, false);
       //旋转向量变旋转矩阵
       std::cout << "rvec=\n"<< rvec << endl;
       std::cout << "tvec=\n"<< tvec << endl;
       Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
       cv::Rodrigues(rvec, rotation_matrix);
       std::cout << "rotation_matrix=\n"<< rotation_matrix << endl;
       cout << "\n\n\n";

       double tvec_f[3]= {tvec.at<double>(0, 0),tvec.at<double>(1, 0),tvec.at<double>(2, 0)};
       /*vector<float> tvec_f;
       for (int i = 0; i < tvec.rows; ++i)
         for (int j = 0; j < tvec.cols; ++j)
           tvec_f.push_back(tvec.at<unsigned char>(i, j));*/
       for (int i=0;i<3;i++)
       {
           tag_T_cam[i][3]= tvec_f[i];
           tag_T_cam[3][i]=0;
           for (int j=0;j<3;j++)
           {
               tag_T_cam[i][i]=rotation_matrix.at<double>(0, 0);
           }
       }
       tag_T_cam[3][3]=1;
    }
    cout<<"jieshu"<<endl;


    //waitKey();
    return flag;
}
