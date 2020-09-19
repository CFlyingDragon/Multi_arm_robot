#include "armc_visual/capture.h"
#include "FlyCapture2.h"

using namespace cv;
using namespace std;
using namespace FlyCapture2;

//将所有函数定义在统一个文件中
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

//密码锁检测，返回检测是否成功，位姿齐次矩阵引用
int lock_capture_pose(float tag_T_cam[4][4])
{
	FlyCapture2::Error error;
	Camera camera;
	CameraInfo camInfo;

	// Connect the camera
	error = camera.Connect( 0 );
	if ( error != PGRERROR_OK )
	{
		std::cout << "Failed to connect to camera" << std::endl;     
		return false;
	}

	// Get the camera info and print it out
	error = camera.GetCameraInfo( &camInfo );
	if ( error != PGRERROR_OK )
	{
		std::cout << "Failed to get camera info from camera" << std::endl;     
		return false;
	}
	std::cout << camInfo.vendorName << " "
		<< camInfo.modelName << " " 
		<< camInfo.serialNumber << std::endl;

	error = camera.StartCapture();
	if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
	{
		std::cout << "Bandwidth exceeded" << std::endl;     
		return false;
	}
	else if ( error != PGRERROR_OK )
	{
		std::cout << "Failed to start image capture" << std::endl;     
		return false;
	} 

	// capture loop
	char key = 0;
	char pic_name[40];
	int ROI_Count=1;
        int flag=0;
	while(key != 'q')
	{
		// Get the image
		Image rawImage;
		error = camera.RetrieveBuffer( &rawImage );
		if ( error != PGRERROR_OK )
		{
			std::cout << "capture error" << std::endl;
			continue;
		}

		// convert to rgb
		Image rgbImage;
		rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );

		// convert to OpenCV Mat
		unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
		cv::Mat image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
		cv::imshow("image", image);
                //cv::waitKey();
        flag=lock_pose(image,tag_T_cam);
        if (flag == 1)
        {
            cout<<"密码锁识别成功"<<endl;
            break;
        }
        else
        {
            if (ROI_Count>10)
            {
               cout<<"密码锁识别失败"<<endl;
                break;
            }
        }
        ROI_Count=ROI_Count+1;
                //key = cv::waitKey(30);
        /*if(key==32){      //the Ascii of "Space key" is 32
			sprintf(pic_name,"../pic_saved/pic_%d.bmp",ROI_Count);
			cv::imwrite(pic_name,image);	
			ROI_Count++;
        }  */
	}

	error = camera.StopCapture();
	if ( error != PGRERROR_OK )
	{
		// This may fail when the camera was removed, so don't show 
		// an error message
	}  

	camera.Disconnect();
        cout<<"a3"<<endl;

    return flag;
}

//门把手检测
int handle_capture_pose(float twist[6])
{
    FlyCapture2::Error error;
    Camera camera;
    CameraInfo camInfo;

    // Connect the camera
    error = camera.Connect( 0 );
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to connect to camera" << std::endl;
        return false;
    }

    // Get the camera info and print it out
    error = camera.GetCameraInfo( &camInfo );
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to get camera info from camera" << std::endl;
        return false;
    }
    std::cout << camInfo.vendorName << " "
        << camInfo.modelName << " "
        << camInfo.serialNumber << std::endl;

    error = camera.StartCapture();
    if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        std::cout << "Bandwidth exceeded" << std::endl;
        return false;
    }
    else if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to start image capture" << std::endl;
        return false;
    }

    // capture loop
    char key = 0;
    char pic_name[40];
    int ROI_Count=1;
    int flag;
    while(key != 'q')
    {
        // Get the image
        Image rawImage;
        error = camera.RetrieveBuffer( &rawImage );
        if ( error != PGRERROR_OK )
        {
            std::cout << "capture error" << std::endl;
            continue;
        }

        // convert to rgb
        Image rgbImage;
        rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );

        // convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
        cv::Mat image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
        cv::imshow("image", image);
        //cv::waitKey();
        flag=menbashou_pose(image,twist);
        if (flag == 1)
        {
            cout<<"门把手识别成功"<<endl;
            break;
        }
        else
        {
            if (ROI_Count>10)
            {
                cout<<"门把手识别失败"<<endl;
                break;
            }
        }
        //key = cv::waitKey(30);
        /*if(key==32){      //the Ascii of "Space key" is 32
            sprintf(pic_name,"../pic_saved/pic_%d.bmp",ROI_Count);
            cv::imwrite(pic_name,image);
            ROI_Count++;
        }  */
        ROI_Count=ROI_Count+1;
    }

    error = camera.StopCapture();
    if ( error != PGRERROR_OK )
    {
        // This may fail when the camera was removed, so don't show
        // an error message
    }

    camera.Disconnect();

    return flag;
}
