#include "armc_visual/Rect_Detect.h"
Mat PIC,PIC_copy,PIC_copy1,PIC_copy2,PIC_copy3,PIC_copy4,xianshi;
int img_x,img_y;

double  lineslope(Vec4f line)
{//斜率 返回弧度	
	double dsx = line[0] - line[2];
	double dsy = line[1] - line[3] + DBL_MIN;
	return atan(dsy / dsx); // 
}

double Linedistance(Vec4f line)
{ //直线长度 
	return sqrt((line[0] - line[2])*(line[0] - line[2]) + (line[1] - line[3] )*(line[1] - line[3] ));
}

double Pointdistance(float  x1, float  y1, float x2, float y2)
{ //点点距离 
	return		   sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}


Point2f getCrossPoint(Vec4f LineA, Vec4f LineB)
{/*函数功能：求两条直线交点*/
	Point2f crossPoint;
	float a1 = LineA[1] - LineA[3];
    float b1 = LineA[2] - LineA[0];
    float c1 = LineA[0]*LineA[3]-LineA[2]*LineA[1];
    float a3 = LineB[1] - LineB[3];
    float b3 = LineB[2] - LineB[0];
    float c3 = LineB[0]*LineB[3]-LineB[2]*LineB[1];
    float D = a1*b3 - a3*b1;
    float x = (b1*c3 - b3*c1) / D;
    float y = (a3*c1 - a1*c3) / D;

    //cout << "D: "  << D << endl;
    crossPoint.x = x;
    crossPoint.y = y;
    return crossPoint;
}

/*绘制可能组成矩形的4条直线*/
void DrawRect(Vec4f L1,Vec4f L2,Vec4f L3,Vec4f L4){
	cout << "*****enter DrawRect****" << endl;

	vector<Vec4f> DrawLines;
	DrawLines.push_back(L1);
	DrawLines.push_back(L2);
	DrawLines.push_back(L3);
	DrawLines.push_back(L4);
	
	Mat pic;   	PIC.copyTo(pic);
	for (int i = 0; i < DrawLines.size(); i++){
		line(pic, Point(DrawLines.at(i)[0], DrawLines.at(i)[1]), Point(DrawLines.at(i)[2], DrawLines.at(i)[3]),Scalar(255, 0, 0),2);  //画出边缘图
	}
    //imshow("pic",pic);
	//imwrite("../image/res/rect_ansys.bmp",pic);
	
        //waitKey(4);
	
}


/*实现排列组合:进行选择*/
vector<vector<int>> res_select;
void func_select(int n,int k,int a[],int m)
{
    int i;
    vector<int> values;				
	//当k==0的时候，将数组里面的个数存储
    if( k == 0 )
    {		
        for(int j= 0; j < m; ++j){
            values.push_back(a[j]);
            //cout<<"aj"<<a[j]<<endl;
            //cout<<"case1:  "<<"  j=  "<<j<<"  k=  "<<k<<"  m=  "<<m<<"  a=  "<<a[j]<<endl;
		}
        //cout<<"一轮"<<endl;
		res_select.push_back(values);
    }
    else
    {
        for( i = n; i >= k; --i )
        {
            a[m] = i;
            func_select(i-1,k-1,a,m+1);
            //cout<<"case2:  "<<"  i=  "<<i<<"  k=  "<<k<<"  m=  "<<m<<"  n=  "<<n<<"  a=  "<<a[m]<<endl;
        } 
    }

}

// 对直线进行聚合，主要聚合：共线、且直线上一点位于另一直线上的两条直线,原版本
vector<Vec4f> line_cluster(vector<Vec4f> DetLines)
{
	int vec_size = DetLines.size();
    double Dis_p2line1,Dis_p2line2;
    double lenth[6],length_max;
    int Num_index[500];
	int Uindex=0;
	vector<Vec4f> cluster_line;
	vector<Vec4f> NCreat_line;  
    //cout << "vec_size:" << vec_size<< endl;

	for(int i=0;i<vec_size-1;i++){

		// cout << " line_A: " << line_A << " line_B: " << line_B << " line_C: " << line_C << endl;
        for(int j=i+1;j<vec_size;j++)
        {
            bool pingxing=false;
            if(abs(lineslope(DetLines[i])-lineslope(DetLines[j]))<0.52 ||abs(lineslope(DetLines[i])-lineslope(DetLines[j]))>2.62)//平行条件：小于30°
            {
             pingxing=true;
            }
            //if((i!=j)&& abs(lineslope(DetLines[i])-lineslope(DetLines[j]))<0.1){
            if((i!=j)&& pingxing){
                double distance1=Linedistance(DetLines[i]);
                double distance2=Linedistance(DetLines[j]);
                if ((distance1 > distance2) || (distance1 == distance2))
                {
                    // 计算直线Detlines[i]的直线方程
                    double line_A = DetLines.at(i)[3] - DetLines.at(i)[1];
                    double line_B = DetLines.at(i)[0] - DetLines.at(i)[2];
                    double line_C = DetLines.at(i)[2]*DetLines.at(i)[1] -  DetLines.at(i)[0]*DetLines.at(i)[3];
                    // 计算点到直线的距离,计算直线两端点到另一直线的距离
                    Dis_p2line1 = abs(line_A*DetLines.at(j)[0]+line_B*DetLines.at(j)[1]+line_C)/sqrt(line_A*line_A+line_B*line_B);
                    Dis_p2line2 = abs(line_A*DetLines.at(j)[2]+line_B*DetLines.at(j)[3]+line_C)/sqrt(line_A*line_A+line_B*line_B);
                }
                else
                {
                    // 计算直线Detlines[j]的直线方程
                    double line_A = DetLines.at(j)[3] - DetLines.at(j)[1];
                    double line_B = DetLines.at(j)[0] - DetLines.at(j)[2];
                    double line_C = DetLines.at(j)[2]*DetLines.at(j)[1] -  DetLines.at(j)[0]*DetLines.at(j)[3];
                    // 计算点到直线的距离,计算直线两端点到另一直线的距离
                    Dis_p2line1 = abs(line_A*DetLines.at(i)[0]+line_B*DetLines.at(i)[1]+line_C)/sqrt(line_A*line_A+line_B*line_B);
                    Dis_p2line2 = abs(line_A*DetLines.at(i)[2]+line_B*DetLines.at(i)[3]+line_C)/sqrt(line_A*line_A+line_B*line_B);
                }



                /*cout << "Dis_p2line1: "<< Dis_p2line1 << "  Dis_p2line2: "<< Dis_p2line2<< endl;

                //逐个显示所绘制的直线
                 PIC_copy1=PIC_copy.clone();
                     line(PIC_copy, Point(DetLines.at(i)[0], DetLines.at(i)[1]), Point(DetLines.at(i)[2], DetLines.at(i)[3]),Scalar(255, 0, 0),2);  //画出边缘图
                     line(PIC_copy, Point(DetLines.at(j)[0], DetLines.at(j)[1]), Point(DetLines.at(j)[2], DetLines.at(j)[3]),Scalar(0, 255, 0),2);  //画出边缘图
                     resize(PIC_copy, xianshi, Size2f(0.5*PIC_copy.cols,0.5*PIC_copy.rows));
                     //imshow("xianshi",xianshi);
                     //waitKey();
                     PIC_copy=PIC_copy1.clone();*/

				// 判断两点是否在线段上,是则不进行合并
                //bool mid_x1 = ((DetLines.at(j)[0]>min(DetLines.at(i)[0],DetLines.at(i)[2])) && (DetLines.at(j)[0]<max(DetLines.at(i)[0],DetLines.at(i)[0])));
                //bool mid_x2 = ((DetLines.at(j)[2]>min(DetLines.at(i)[0],DetLines.at(i)[2])) && (DetLines.at(j)[0]<max(DetLines.at(i)[0],DetLines.at(i)[0])));
                /*bool mid_x1 = ((DetLines.at(j)[0]>min(DetLines.at(i)[0],DetLines.at(i)[2])) && (DetLines.at(j)[0]<max(DetLines.at(i)[0],DetLines.at(i)[2])));
                bool mid_x2 = ((DetLines.at(j)[2]>min(DetLines.at(i)[0],DetLines.at(i)[2])) && (DetLines.at(j)[0]<max(DetLines.at(i)[0],DetLines.at(i)[2])));
                bool mid_y1 = ((DetLines.at(j)[1]>min(DetLines.at(i)[1],DetLines.at(i)[3])) && (DetLines.at(j)[1]<max(DetLines.at(i)[1],DetLines.at(i)[3])));
				bool mid_y2 = ((DetLines.at(j)[3]>min(DetLines.at(i)[1],DetLines.at(i)[3])) && (DetLines.at(j)[3]<max(DetLines.at(i)[1],DetLines.at(i)[3])));
                bool ismid = (mid_x1&&mid_x2) ||(mid_y1&&mid_y2);*/
			    //cout<< "ismid: " << ismid << endl;
				// 选出点到直线距离小的两条直线

                //if(Dis_p2line<5 && !ismid){
                //if(Dis_p2line<0.01*Linedistance(DetLines[i]) && !ismid){
                Vec4f line;
                double xian=600;
                if((Dis_p2line1<0.02*min(max(Linedistance(DetLines[i]),Linedistance(DetLines[j])),xian)) && (Dis_p2line2<0.02*min(max(Linedistance(DetLines[i]),Linedistance(DetLines[j])),xian)))
                {
                    cout << "Linedistance(DetLines[i]): " << Linedistance(DetLines[i]) << endl;
                //if(Dis_p2line<5){
                    //cout<<"chenggongle"<<endl;
					// 生成新的一条直线

					lenth[0] = Pointdistance(DetLines.at(i)[0],DetLines.at(i)[1],DetLines.at(j)[0],DetLines.at(j)[1]);
					lenth[1] = Pointdistance(DetLines.at(i)[0],DetLines.at(i)[1],DetLines.at(j)[2],DetLines.at(j)[3]);
					lenth[2] = Pointdistance(DetLines.at(i)[2],DetLines.at(i)[3],DetLines.at(j)[0],DetLines.at(j)[1]);
                    lenth[3] = Pointdistance(DetLines.at(i)[2],DetLines.at(i)[3],DetLines.at(j)[2],DetLines.at(j)[3]);
                    lenth[4] = Linedistance(DetLines[i]);
                    lenth[5] = Linedistance(DetLines[j]);
                    length_max= max(max(max(lenth[0],lenth[1]),max(lenth[2],lenth[3])),max(lenth[4],lenth[5]));
                    for(int k=0;k<6;k++){
						if(lenth[k]==length_max){
							switch(k){
								case 0:{ line[0]=DetLines.at(i)[0];line[1]=DetLines.at(i)[1];line[2]=DetLines.at(j)[0];line[3]=DetLines.at(j)[1]; 
                                        //cout << "finished in case 0" << endl;
                                break;}
								case 1:{ line[0]=DetLines.at(i)[0];line[1]=DetLines.at(i)[1];line[2]=DetLines.at(j)[2];line[3]=DetLines.at(j)[3]; 
                                        //cout << "finished in case 1" << endl;
                                break;}
								case 2:{ line[0]=DetLines.at(i)[2];line[1]=DetLines.at(i)[3];line[2]=DetLines.at(j)[0];line[3]=DetLines.at(j)[1];
                                        //cout << "finished in case 2" << endl;
                                break;}
								case 3:{ line[0]=DetLines.at(i)[2];line[1]=DetLines.at(i)[3];line[2]=DetLines.at(j)[2];line[3]=DetLines.at(j)[3]; 
                                        //cout << "finished in case 3" << endl;
                                break;}
                            case 4:{ line[0]=DetLines.at(i)[0];line[1]=DetLines.at(i)[1];line[2]=DetLines.at(i)[2];line[3]=DetLines.at(i)[3];
                                    //cout << "finished in case 3" << endl;
                            break;}
                            case 5:{ line[0]=DetLines.at(j)[0];line[1]=DetLines.at(j)[1];line[2]=DetLines.at(j)[2];line[3]=DetLines.at(j)[3];
                                    //cout << "finished in case 3" << endl;
                            break;}
							}
                            break;
                        }
						}
                    Num_index[Uindex] =i; Uindex=Uindex+1;// cout << "i: " << i << "Num_index[Uindex-1]" << Num_index[Uindex-1] <<endl;
                    Num_index[Uindex] =j; Uindex=Uindex+1;// cout << "j: " << j << "Num_index[Uindex-1]" << Num_index[Uindex-1] <<endl;
                   // cout << "Uindex:  " << Uindex << endl;
                    //cout << "DetLines.size()1:" << DetLines.size() << endl;
                    DetLines.push_back(line);
                    // cout << "DetLines.size()2:" << DetLines.size() << endl;
                    // cout<<"i= "<<i<<"  j=  "<<j<<endl;
                    break;//为防止直线越来越多
                }
                 else
                 {
                            //针对平行近似相等直线设置搜索条件
                    if((Dis_p2line1<0.05*max(Linedistance(DetLines[i]),Linedistance(DetLines[j]))) && (Dis_p2line2<0.05*max(Linedistance(DetLines[i]),Linedistance(DetLines[j])))
                           && (Linedistance(DetLines[i]) < 1.2*Linedistance(DetLines[j]) && Linedistance(DetLines[i]) > 0.8*Linedistance(DetLines[j]) ))
                        {
                                 // 生成新的一条直线
                                 lenth[0] = Pointdistance(DetLines.at(i)[0],DetLines.at(i)[1],DetLines.at(j)[0],DetLines.at(j)[1]);
                                 lenth[1] = Pointdistance(DetLines.at(i)[0],DetLines.at(i)[1],DetLines.at(j)[2],DetLines.at(j)[3]);
                                 lenth[2] = Pointdistance(DetLines.at(i)[2],DetLines.at(i)[3],DetLines.at(j)[0],DetLines.at(j)[1]);
                                 lenth[3] = Pointdistance(DetLines.at(i)[2],DetLines.at(i)[3],DetLines.at(j)[2],DetLines.at(j)[3]);
                                 lenth[4] = Linedistance(DetLines[i]);
                                 lenth[5] = Linedistance(DetLines[j]);
                                 length_max= max(max(lenth[0],lenth[1]),max(lenth[2],lenth[3]));
                                 if (length_max<1.2*lenth[4] || length_max<1.2*lenth[5])
                                 {
                                     length_max= max(max(max(lenth[0],lenth[1]),max(lenth[2],lenth[3])),max(lenth[4],lenth[5]));
                                     for(int k=0;k<6;k++){
                                         if(lenth[k]==length_max){
                                             switch(k){
                                                 case 0:{ line[0]=DetLines.at(i)[0];line[1]=DetLines.at(i)[1];line[2]=DetLines.at(j)[0];line[3]=DetLines.at(j)[1];
                                                         //cout << "finished in case 0" << endl;
                                                 break;}
                                                 case 1:{ line[0]=DetLines.at(i)[0];line[1]=DetLines.at(i)[1];line[2]=DetLines.at(j)[2];line[3]=DetLines.at(j)[3];
                                                         //cout << "finished in case 1" << endl;
                                                 break;}
                                                 case 2:{ line[0]=DetLines.at(i)[2];line[1]=DetLines.at(i)[3];line[2]=DetLines.at(j)[0];line[3]=DetLines.at(j)[1];
                                                         //cout << "finished in case 2" << endl;
                                                 break;}
                                                 case 3:{ line[0]=DetLines.at(i)[2];line[1]=DetLines.at(i)[3];line[2]=DetLines.at(j)[2];line[3]=DetLines.at(j)[3];
                                                         //cout << "finished in case 3" << endl;
                                                 break;}
                                             case 4:{ line[0]=DetLines.at(i)[0];line[1]=DetLines.at(i)[1];line[2]=DetLines.at(i)[2];line[3]=DetLines.at(i)[3];
                                                     //cout << "finished in case 3" << endl;
                                             break;}
                                             case 5:{ line[0]=DetLines.at(j)[0];line[1]=DetLines.at(j)[1];line[2]=DetLines.at(j)[2];line[3]=DetLines.at(j)[3];
                                                     //cout << "finished in case 3" << endl;
                                             break;}
                                             }
                                             break;
                                 }
                             }
                                     Num_index[Uindex] =i; Uindex=Uindex+1;// cout << "i: " << i << "Num_index[Uindex-1]" << Num_index[Uindex-1] <<endl;
                                     Num_index[Uindex] =j; Uindex=Uindex+1;// cout << "j: " << j << "Num_index[Uindex-1]" << Num_index[Uindex-1] <<endl;
                                     // cout << "Uindex:  " << Uindex << endl;
                                     //cout << "DetLines.size()1:" << DetLines.size() << endl;
                                     DetLines.push_back(line);
                                     // cout << "DetLines.size()2:" << DetLines.size() << endl;
                                     // cout<<"i= "<<i<<"  j=  "<<j<<endl;
                                     break;//为防止直线越来越多
                        }


                    }

					}

				}
		}
	}
    //cout << "Uindexzong:  " << Uindex << endl;
	bool Iskick;
	// 剔除部分直线，并得到新的直线组
	for(int Mi=0;Mi<DetLines.size();Mi++){
		Iskick=false;
        //cout << "Num_index.size()" << Num_index.size() << endl;
		for(int Mj=0;Mj<Uindex;Mj++){
			if(Mi==Num_index[Mj]){ Iskick=true; break;  }
		}
		if(!Iskick){
			cluster_line.push_back(DetLines[Mi]);
			//cout << "push_back DetLines[Mi]" << endl;
		}
	}
	// 	显示经过聚类的直线
    //Mat cluster_PIC; PIC.copyTo(cluster_PIC);
    //cout <<"cluster_line.size():" << cluster_line.size() << endl;
    /*for (int i = 0; i < cluster_line.size(); i++){
		cv::line(cluster_PIC, Point(cluster_line.at(i)[0], cluster_line.at(i)[1]), Point(cluster_line.at(i)[2], cluster_line.at(i)[3]),Scalar(0, 255, 0),2);  //画出边缘图
    }*/
	return cluster_line;	
}

//判断能否组成矩形
bool rect_ansys(Vec4f line1,Vec4f line2,Vec4f line3,Vec4f line4)
{//可以改进
	vector<double> angle;
	angle.push_back(lineslope(line1));
	angle.push_back(lineslope(line2));
	angle.push_back(lineslope(line3));
	angle.push_back(lineslope(line4));


	//cout << endl <<"enter rect_ansys function:" << endl;
	//cout << "angle" << angle[0] <<"\t" << angle[1] <<"\t" <<angle[2]<<"\t"<<angle[3] <<"\t" <<endl;
	// 存在不存在平行线的直线时则退出
	bool parell=false,vertical;
    //cout<<"xin1lun"<<endl;
	for(int i=0;i<4;i++){
		parell=false;
		vertical=false;
		for(int j=0;j<4;j++){
            float x=angle[i]-angle[j];
            //cout <<"x= "<<x<<endl;
            bool pingxing=false;
            if(abs(angle[i]-angle[j])<0.52 ||abs(angle[i]-angle[j])>2.76){
                pingxing=true;
            }
            //if((i!=j)&&(abs(angle[i]-angle[j])<0.1)){
            if((i!=j)&&pingxing){
				 parell = true;  break;
			}

		}
        //cout<<"yilun"<<endl;
        //判断垂直
		for(int j=0;j<4;j++){
             float y=angle[i]-angle[j];
             //cout <<"y= "<<y<<endl;
            bool chuizhi=false;
            if(abs(angle[i]-angle[j])>1.1 && abs(angle[i]-angle[j])<2){
                chuizhi=true;
            }
            //if((i!=j)&&(abs(angle[i]-angle[j])>1.2)){
            if((i!=j)&&chuizhi){
				 vertical = true;  break;
			}
		}
        //cout<<"yilun"<<endl;
		if(!parell||!vertical)
            {cout << "parell or vertical return false" << endl;  return false; //return false不保留???
			}
	}
    cout<<"退出了吗"<<endl;
	// 求直线交点，当距离直线端点太远则剔除
	Vec4f H1,H2,V1,V2;
	int res_index;
	H1=line1;
	for(int i=1;i<4;i++){
        if(abs(angle[i]-angle[0])<0.52 ||abs(angle[i]-angle[0])>2.76)
			res_index=i+1;

	}
    //cout<<"res_index="<<res_index<<endl;
	switch(res_index){
		case 2:	{
			H2=line2; V1=line3;V2=line4; break;	}
		case 3:	{
			H2=line3; V1=line2;V2=line4; break;	}
		case 4:	{
			H2=line4; V1=line2;V2=line3; break;	}
	}

	Point2f P1,P2,P3,P4;
	P1=getCrossPoint(H1,V1);
	P2=getCrossPoint(H2,V1);
	P3=getCrossPoint(H1,V2);
	P4=getCrossPoint(H2,V2);
	// cout << "point x and y" << P1.x<<"\t" << P1.y<<"\t"<< P2.x<<"\t"<< P2.y<<"\t"<< P3.x<<"\t" <<P3.y<< endl;
	
	// 剔除结果为nan类型
    if(isnan(P1.x)||isnan(P1.y)||isnan(P2.x)||isnan(P2.y)||isnan(P3.x)||isnan(P3.y)||isnan(P1.x)||isnan(P1.y)){
        cout<<"nan1"<<endl;
        return false;}

    if((P1.x<0||P1.y<0)||(P2.x<0||P2.y<0)||(P3.x<0||P3.y<0)||(P4.x<0||P4.y<0)){
        cout<<"xiaoyu0"<<endl;
        return false;}

   /* if((P1.x>img_x||P1.y>img_y)||(P2.x>img_x||P2.y>img_y)||(P3.x>img_x||P3.y>img_y)||(P4.x>img_x||P4.y>img_y)){
        cout<<"dayufanwei"<<endl;
        return false;}*/
	// 计算交点与端点的距离

	double errDistRate1,errDistRate2,errDistRate3,errDistRate4;
	double errDistRateV1,errDistRateV2,errDistRateV3,errDistRateV4;
	errDistRate1=min((double)(Pointdistance(P1.x,P1.y,H1[0],H1[1])),(double)(Pointdistance(P1.x,P1.y,H1[2],H1[3])))/(double)((Linedistance(H1)));
	errDistRate2=min((double)(Pointdistance(P2.x,P2.y,H2[0],H2[1])),(double)(Pointdistance(P2.x,P2.y,H2[2],H2[3])))/(double)((Linedistance(H2)));
	errDistRate3=min((double)(Pointdistance(P3.x,P3.y,H1[0],H1[1])),(double)(Pointdistance(P3.x,P3.y,H1[2],H1[3])))/(double)((Linedistance(H1)));
	errDistRate4=min((double)(Pointdistance(P4.x,P4.y,H2[0],H2[1])),(double)(Pointdistance(P4.x,P4.y,H2[2],H2[3])))/(double)((Linedistance(H2)));

	errDistRateV1=min((double)(Pointdistance(P1.x,P1.y,V1[0],V1[1])),(double)(Pointdistance(P1.x,P1.y,V1[2],V1[3])))/(double)((Linedistance(V1)));
	errDistRateV2=min((double)(Pointdistance(P2.x,P2.y,V1[0],V1[1])),(double)(Pointdistance(P2.x,P2.y,V1[2],V1[3])))/(double)((Linedistance(V1)));
	errDistRateV3=min((double)(Pointdistance(P3.x,P3.y,V2[0],V2[1])),(double)(Pointdistance(P3.x,P3.y,V2[2],V2[3])))/(double)((Linedistance(V2)));
	errDistRateV4=min((double)(Pointdistance(P4.x,P4.y,V2[0],V2[1])),(double)(Pointdistance(P4.x,P4.y,V2[2],V2[3])))/(double)((Linedistance(V2)));


	//cout <<"errDistRate: " << errDistRate1  <<" " <<errDistRate2<<" " << errDistRate3 <<" " << errDistRate4 << endl;
	DrawRect(H1,H2,V1,V2);
    //if((errDistRate1<0.25)&&(errDistRate2<0.25)&&(errDistRate3<0.25)&&(errDistRate4<0.25)&&(errDistRateV1<0.45)&&(errDistRateV2<0.45)&&(errDistRateV3<0.45)&&(errDistRateV4<0.45))
    if((errDistRate1<0.5)&&(errDistRate2<0.5)&&(errDistRate3<0.5)&&(errDistRate4<0.5)&&(errDistRateV1<0.5)&&(errDistRateV2<0.5)&&(errDistRateV3<0.5)&&(errDistRateV4<0.5))
        // if((errDistRate1<0.2)&&(errDistRate2<0.2)&&(errDistRate3<0.2)&&(errDistRate4<0.2))
	{
        //cout<<"hahaah"<<endl;
        //计算平行直线间距，不能小于另外的直线的长度,直线间距离用平行线间距离算也可以
		double dist_H=min(Pointdistance(H1[0],H1[1],H2[0],H2[1]),Pointdistance(H1[0],H1[1],H2[2],H2[3]));
		double dist_V=min(Pointdistance(V1[0],V1[1],V2[0],V2[1]),Pointdistance(V1[0],V1[1],V2[2],V2[3]));
		double length_H=max(Linedistance(H1),Linedistance(H2));
		double length_V=max(Linedistance(V1),Linedistance(V2));
        //if((dist_H<0.8*length_V)||(dist_H>1.2*length_V)||(dist_V<0.8*length_H)||(dist_V>1.2*length_H)){
        if((dist_H<0.5*length_V)||(dist_H>1.5*length_V)||(dist_V<0.5*length_H)||(dist_V>1.5*length_H)){
			return false;
		}
		cout << "errDistRate and length true" << endl; return true;  
	}

	return false;	

}

// double OnlyThreshold = 0.8;

vector<Point2f> ObtainCrossPoint(Vec4f line1,Vec4f line2,Vec4f line3,Vec4f line4){
	
	vector<double> angle;
	angle.push_back(lineslope(line1));
	angle.push_back(lineslope(line2));
	angle.push_back(lineslope(line3));
	angle.push_back(lineslope(line4));

	Vec4f H1,H2,V1,V2;
	int res_index;
	H1=line1;
	for(int i=1;i<4;i++){
        if(abs(angle[i]-angle[0])<0.3 || abs(angle[i]-angle[0])>2.8)
			res_index=i+1;
	}
	
	switch(res_index){
		case 2:	{
			H2=line2; V1=line3;V2=line4; break;	}
		case 3:	{
			H2=line3; V1=line2;V2=line4; break;	}
		case 4:	{
			H2=line4; V1=line2;V2=line3; break;	}
	}


	Point2f P1,P2,P3,P4;
	P1=getCrossPoint(H1,V1);
	P2=getCrossPoint(H2,V1);
	P3=getCrossPoint(H1,V2);
	P4=getCrossPoint(H2,V2);

	vector<Point2f> pointC,pointRes;
	pointC.push_back(P1);
	pointC.push_back(P2);
	pointC.push_back(P3);
	pointC.push_back(P4);

	// 按x方向升序排序
	sort(pointC.begin(), pointC.end(), [](const Point2f &point1, const Point2f &point2){ return point1.x < point2.x;});
    if(abs(pointC[0].y)<abs(pointC[1].y)){
       if(abs(pointC[2].y)<abs(pointC[3].y)){//0  2
                                             //1  3
           pointRes.push_back(pointC[0]);
           pointRes.push_back(pointC[2]);
           pointRes.push_back(pointC[3]);
           pointRes.push_back(pointC[1]);
           //cout<<"c1"<<endl;
       }
       else{//0  3  1  2
           pointRes.push_back(pointC[0]);
           pointRes.push_back(pointC[3]);
           pointRes.push_back(pointC[2]);
           pointRes.push_back(pointC[1]);
           //cout<<"c2"<<endl;
       }
    }
    else{
        if(abs(pointC[2].y)<abs(pointC[3].y)){//1  2  0  3
            pointRes.push_back(pointC[1]);
            pointRes.push_back(pointC[2]);
            pointRes.push_back(pointC[3]);
            pointRes.push_back(pointC[0]);
            //cout<<"c3"<<endl;
        }
        else{//1  3  0  2
            pointRes.push_back(pointC[1]);
            pointRes.push_back(pointC[3]);
            pointRes.push_back(pointC[2]);
            pointRes.push_back(pointC[0]);
           // cout<<"c4"<<endl;
        }
    }
    /*double Dist_P01=Pointdistance(pointC[0].x,pointC[0].y,pointC[1].x,pointC[1].y);
	double Dist_P02=Pointdistance(pointC[0].x,pointC[0].y,pointC[2].x,pointC[2].y);
	Vec4f line_01 = Vec4f(pointC[0].x,pointC[0].y,pointC[1].x,pointC[1].y);
	Vec4f line_02 = Vec4f(pointC[0].x,pointC[0].y,pointC[2].x,pointC[2].y);
    // 选择短边判断左上角点
	if(Dist_P01>Dist_P02){
		if(line_02[1]<line_02[3]){// -*0--*1
		      					  // --*2--*3
            pointRes.push_back(pointC[0]);  //左上角点zenyanggeidian???
			pointRes.push_back(pointC[1]);
			pointRes.push_back(pointC[2]);
			pointRes.push_back(pointC[3]);
		}
		else{// --*2--*3
		     // -*0--*1
			pointRes.push_back(pointC[2]);
			pointRes.push_back(pointC[3]);
			pointRes.push_back(pointC[1]);
			pointRes.push_back(pointC[0]);
		}
	}
	else{
		if(line_01[1]<line_01[3]){//-*0--*2
		      					 // --*1--*3
			pointRes.push_back(pointC[0]);  //左上角点
			pointRes.push_back(pointC[2]);
			pointRes.push_back(pointC[3]);
			pointRes.push_back(pointC[1]);
		}
		else{ // --*1--*3
		      // -*0--*2
			pointRes.push_back(pointC[1]);
			pointRes.push_back(pointC[3]);
			pointRes.push_back(pointC[2]);
			pointRes.push_back(pointC[0]);
		}
    }*/
    //cout<<"aa"<<pointRes[0].x<<"  "<<pointRes[0].y<<endl;
	return pointRes;

	
}

vector<Point2f> RectDetect(Mat ROI_PIC,bool* isDetect)
{
#if 0
	Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
#else  

    Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE, 0.7, 0.6, 2, 27.5, 0, 0.7, 1024);
    //Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD, 0.8, 0.6, 2.0, 22.5, 0, 0.7, 1024);

#endif
    //int flag=0;
	vector<Point2f> corner;  //默认为空，用于未检测到时返回
    vector<Point2f> fourcorner;
	ROI_PIC.copyTo(PIC);
    PIC.copyTo(PIC_copy);
    PIC.copyTo(PIC_copy1);
    PIC.copyTo(PIC_copy2);
    PIC.copyTo(PIC_copy3);
    PIC.copyTo(PIC_copy4);
    //imshow("pic",pic);
	Mat Src;  
    cvtColor(PIC, Src, CV_RGB2GRAY);
    Mat Huibao;
    Src.copyTo(Huibao);
    vector<Vec4f> ScrLines, DetLines,ResLines,SaveLines,MidLines,ChangLines;
	ls->detect(Src, ScrLines);



    vector<double> lineangle,Resangle,Saveangle,Changangle;
	img_x=Src.cols;
	img_y=Src.rows;
	//cout << "img size :" << img_x  << " " << img_y << endl;

    IprovedFDCM fastmodel;
    int len_th = 100;
	double rst[4];
    fastmodel.lineSortCut(ScrLines, fastmodel.order, len_th);//删去过短线段
	cout << "ScrLines.size():" << ScrLines.size()<< endl;

    int N=min(30,int(ScrLines.size()));
    for(int i=0;i<N;i++){//怎样选择最大的20条？？？
		 DetLines.push_back(ScrLines.at(fastmodel.order.at(i).order));
         lineangle.push_back(lineslope(DetLines[i]));//lineslope:线坡度
         //cout << "lineangle[i]:" << lineangle[i] << endl;
         //cout << "Linedistance(DetLines[i]):" << Linedistance(DetLines[i]) << endl;
	}

	// 剔除不存在平行线的直线
	int num = DetLines.size();
	
	bool parell=false;
	for(int i=0;i<num;i++){
		parell=false;
		for(int j=0;j<num;j++){
			float k=lineangle[i]-lineangle[j];
            //cout<< "k="<<k<<endl;
			bool pingxing=false;
            if (abs(lineangle[i]-lineangle[j])<0.52||abs(lineangle[i]-lineangle[j])>2.62){
				pingxing=true;
			}
			//if((i!=j) && abs(lineangle[i]-lineangle[j])<0.7){
				if((i!=j) && pingxing){
				 parell = true;
				 break; }
		}
		if(parell){
			// cout << "**Pareller push_back Ld(DetLines[j])**:  " <<Linedistance(DetLines[i]) << endl;
			SaveLines.push_back(DetLines[i]);
			Saveangle.push_back(lineangle[i]);
            //cout<<"buxuyao"<<endl;
			}
            //cout<<"wancheng"<<endl;
	}

	DetLines.clear(); lineangle.clear();
    DetLines.swap(SaveLines);  lineangle.swap(Saveangle);//swap:使用另一个％vector交换数据。

	SaveLines.clear();   Saveangle.clear(); lineangle.clear();
    cout<<"after tichubupingxing zhixian"<<DetLines.size()<<endl;

    /*for(int i=0;i<DetLines.size();i++){
        cout << "Linedistance(DetLines[i]):" << Linedistance(DetLines[i]) << endl;
                   cout << "lineangle[i]" << lineangle[i] << endl;
               }*/

    /*//逐个显示所绘制的直线
     PIC_copy1=PIC_copy.clone();
     for (int i = 0; i < DetLines.size(); i++)
     {
         line(PIC_copy, Point(DetLines.at(i)[0], DetLines.at(i)[1]), Point(DetLines.at(i)[2], DetLines.at(i)[3]),Scalar(255, 0, 0),2);  //画出边缘图
         resize(PIC_copy, xianshi, Size2f(0.5*PIC_copy.cols,0.5*PIC_copy.rows));
         //imshow("xianshi",xianshi);
         //waitKey();
         PIC_copy=PIC_copy1.clone();
     }*/

    /*//绘制直线及显示
    for (int i = 0; i < DetLines.size(); i++){
        line(PIC_copy, Point(DetLines.at(i)[0], DetLines.at(i)[1]), Point(DetLines.at(i)[2], DetLines.at(i)[3]),Scalar(255, 0, 0),2);  //画出边缘图
        resize(PIC_copy, xianshi, Size2f(0.5*PIC_copy.cols,0.5*PIC_copy.rows));
        //imshow("xianshi",xianshi);
        cout<<"xianshi"<<endl;
        //waitKey();
    }*/
    int cishu=0;
   while(1)
   {
       //对两共线的 两条直线进行聚合，将其变为一条直线
       MidLines=line_cluster(DetLines);
       if (MidLines.size() != DetLines.size())
       {
           DetLines.clear(); 	DetLines.swap(MidLines);
           lineangle.clear();

           for(int i=0;i<DetLines.size();i++){
               lineangle.push_back(lineslope(DetLines[i]));

           }

           // 输出经过聚类后的线条信息
           cout << "after cluster DetLines.size()" << DetLines.size() << endl;

       }
       else
       {
           // 输出经过聚类后的线条信息
           cout << "after cluster DetLines.size()" << DetLines.size() << endl;
           //for(int i=0;i<DetLines.size();i++){
               //cout << "Linedistance(DetLines[i]):" << Linedistance(DetLines[i]) << endl;
               //cout << "lineangle[i]" << lineangle[i] << endl;
           //}
           break;
       }
       /*//绘制直线及显示
       for (int i = 0; i < DetLines.size(); i++)
       {
           if (cishu ==0)
           {
               line(PIC_copy, Point(DetLines.at(i)[0], DetLines.at(i)[1]), Point(DetLines.at(i)[2], DetLines.at(i)[3]),Scalar(255, 0, 0),2);  //画出边缘图
               resize(PIC_copy, xianshi, Size2f(0.5*PIC_copy.cols,0.5*PIC_copy.rows));
               //imshow("xianshi",xianshi);
               //waitKey(1000);
           }
           if (cishu ==1)
           {
               line(PIC_copy1, Point(DetLines.at(i)[0], DetLines.at(i)[1]), Point(DetLines.at(i)[2], DetLines.at(i)[3]),Scalar(0, 255, 0),2);  //画出边缘图
               resize(PIC_copy1, xianshi, Size2f(0.5*PIC_copy1.cols,0.5*PIC_copy1.rows));
               //imshow("xianshi",xianshi);
               //waitKey(1000);
           }
           if (cishu ==2)
           {
               line(PIC_copy2, Point(DetLines.at(i)[0], DetLines.at(i)[1]), Point(DetLines.at(i)[2], DetLines.at(i)[3]),Scalar(0, 0, 255),2);  //画出边缘图
               resize(PIC_copy2, xianshi, Size2f(0.5*PIC_copy2.cols,0.5*PIC_copy2.rows));
               //imshow("xianshi",xianshi);
               //waitKey(1000);
           }
           if (cishu ==3)
           {
               line(PIC_copy3, Point(DetLines.at(i)[0], DetLines.at(i)[1]), Point(DetLines.at(i)[2], DetLines.at(i)[3]),Scalar(200, 200, 200),2);  //画出边缘图
               resize(PIC_copy3, xianshi, Size2f(0.5*PIC_copy3.cols,0.5*PIC_copy3.rows));
               //imshow("xianshi",xianshi);
               //waitKey(1000);
           }
           if (cishu ==4)
           {
               line(PIC_copy4, Point(DetLines.at(i)[0], DetLines.at(i)[1]), Point(DetLines.at(i)[2], DetLines.at(i)[3]),Scalar(100, 100, 100),2);  //画出边缘图
               resize(PIC_copy4, xianshi, Size2f(0.5*PIC_copy4.cols,0.5*PIC_copy4.rows));
               //imshow("xianshi",xianshi);
               //waitKey(1000);
           }
           else
           {
                int a=1;
           }
       }*/

       cishu=cishu+1;
   }

   for(int i=0;i<DetLines.size();i++){
       cout << "Linedistance(DetLines[i]):" << Linedistance(DetLines[i]) << endl;
                  cout << "lineangle[i]" << lineangle[i] << endl;
              }

   //逐个显示所绘制的直线
    PIC_copy1=PIC_copy.clone();
    for (int i = 0; i < DetLines.size(); i++)
    {
        line(PIC_copy, Point(DetLines.at(i)[0], DetLines.at(i)[1]), Point(DetLines.at(i)[2], DetLines.at(i)[3]),Scalar(255, 0, 0),2);  //画出边缘图
        resize(PIC_copy, xianshi, Size2f(0.5*PIC_copy.cols,0.5*PIC_copy.rows));
        //imshow("xianshi",xianshi);
        //waitKey();
        PIC_copy=PIC_copy1.clone();
    }

	
    //剔除长度过短直线
   for(int i=0;i<DetLines.size();i++){

       if(Linedistance(DetLines[i] )> 300)
       {
           // cout << "**Pareller push_back Ld(DetLines[j])**:  " <<Linedistance(DetLines[i]) << endl;
           ChangLines.push_back(DetLines[i]);
           Changangle.push_back(lineangle[i]);
           }
   }

   DetLines.clear(); lineangle.clear();
   DetLines.swap(ChangLines);  lineangle.swap(Changangle);//swap:使用另一个％vector交换数据。

   ChangLines.clear();   Changangle.clear();
   cout<<"after tichubupingxing zhixian"<<DetLines.size()<<endl;

   //绘制直线及显示
   for (int i = 0; i < DetLines.size(); i++)
   {
           line(PIC_copy, Point(DetLines.at(i)[0], DetLines.at(i)[1]), Point(DetLines.at(i)[2], DetLines.at(i)[3]),Scalar(255, 0, 0),2);  //画出边缘图

    }
   resize(PIC_copy, xianshi, Size2f(0.5*PIC_copy.cols,0.5*PIC_copy.rows));
   //imshow("xianshi",xianshi);
   //waitKey();



    /*// 剔除平行直线中不存在长度近似相等的直线
    cout << "DetLines.size() After line_cluster :" << DetLines.size() << endl;
	bool lenEqual=false;
    int num2 = DetLines.size();
	for(int i=0;i<num2;i++){
		lenEqual=false;
		for(int j=0;j<num2;j++){
            float k=lineangle[i]-lineangle[j];
            //cout<< "k="<<k<<endl;
            bool pingxing=false;
            if (abs(lineangle[i]-lineangle[j])<0.3||abs(lineangle[i]-lineangle[j])>3){
                pingxing=true;
            }
            if((i!=j) && pingxing){
            //if((i!=j) && abs(lineangle[i]-lineangle[j])<0.2){//原程序
                //if(abs(Linedistance(DetLines[i])-Linedistance(DetLines[j]))<0.2*Linedistance(DetLines[i])){
                if(abs(Linedistance(DetLines[i])-Linedistance(DetLines[j]))<0.5*Linedistance(DetLines[i])){
					lenEqual=true;  // cout <<"i:" <<i << "j:" << j<< endl; 
					//cout<<"Ld(DetLines[i]):"<<Linedistance(DetLines[i])<< "\t"; 
					//cout <<"Ld(DetLines[j])"<< Linedistance(DetLines[j])<<endl;
				}
			}
		}
        if(lenEqual){
			//cout << "**push_back Ld(DetLines[j])**:  " <<Linedistance(DetLines[i]) << endl;
			SaveLines.push_back(DetLines[i]);
			Saveangle.push_back(lineangle[i]);
		}
	}

	DetLines.clear(); lineangle.clear();
	DetLines.swap(SaveLines);  lineangle.swap(Saveangle);
    cout << "after length  kick DetLines.size()" << DetLines.size() << endl;
    //绘制直线及显示
       for (int i = 0; i < DetLines.size(); i++)
       {
               line(PIC_copy, Point(DetLines.at(i)[0], DetLines.at(i)[1]), Point(DetLines.at(i)[2], DetLines.at(i)[3]),Scalar(255, 0, 0),2);  //画出边缘图

        }
       resize(PIC_copy, xianshi, Size2f(0.5*PIC_copy.cols,0.5*PIC_copy.rows));
       //imshow("xianshi",xianshi);
       //waitKey();
	// for(int i=0;i<DetLines.size();i++){
	// 	//cout << "Linedistance(DetLines[i]):" << Linedistance(DetLines[i]) << endl;
	// 	//cout << "lineangle[i]" << lineangle[i] << endl;
    // }*/

     /*ls->drawSegments(Huibao, DetLines);
     //imshow("Huibao",Huibao);
     //waitKey(100);
     //sprintf(pic_name,"../image/pic_saved/Template_%d.bmp",Ti);
     cv::imwrite("Huibao.bmp",Huibao);
     cout<<"zuizhongDetlines:  "<<DetLines.size()<<endl;*/
	//pause();  //程序在当前位置暂停，查看输出
	//从现有直线中选出4条
    int n0,n1,n2,n3;  	int n,k;
    int m=0;  int a[10];  //临时存放结果的数组
    n=DetLines.size();
    k=4;
	// 小于4条线条时直接返回
    if(n<4)
    {
       //flag=0;
        return fourcorner;
    }
    //cout<<"n k a m "<<n<<endl<<k<<endl<<a<<endl<<m<<endl;
    func_select(n,k,a,m);/*实现排列组合:进行选择*///输出res_select中的每一个元素为C84中的一种
       //cout<<"算完啦"<<endl<<"n k m:"<<n<<endl<<k<<endl<<m<<endl<<"a:"<<endl<<a[0]<<endl<<a[1]<<endl<<a[2]<<endl<<a[3]<<endl<<a[4]<<endl<<a[5]<<endl<<a[6]<<endl<<a[7]<<endl<<a[8]<<endl<<a[9]<<endl<<a[10]<<endl<<"结束啦"<<endl;
	bool IsRect;
     //cout << "res_select.size():" << res_select.size() << endl;
	for(int i=res_select.size()-1;i>=0;i--){
		// 判断是否可组成矩形
		n0=res_select[i][0]-1;n1=res_select[i][1]-1;n2=res_select[i][2]-1;n3=res_select[i][3]-1;
		IsRect=rect_ansys(DetLines[n0],DetLines[n1],DetLines[n2],DetLines[n3]);
		if(IsRect)
			break;
	}
    //cout << "IsRect : " << IsRect << endl;
    //cout << "n:" << n0 << " " << n1 << " " << n2 << " " << n3 << endl;
	ResLines.push_back(DetLines[n0]);
	ResLines.push_back(DetLines[n1]);
	ResLines.push_back(DetLines[n2]);
	ResLines.push_back(DetLines[n3]);
    cout<<endl<<"IsRect:  "<<IsRect<<endl<<endl;
	if(IsRect){
        //cout <<"ResLines.size():" << ResLines.size() << endl;//ResLines:修复线
		for (int i = 0; i < ResLines.size(); i++){
			line(PIC, Point(ResLines.at(i)[0], ResLines.at(i)[1]), Point(ResLines.at(i)[2], ResLines.at(i)[3]),Scalar(0, 255, 0),2);  //画出边缘图
		}
	}

	if(!IsRect){
        //cout <<"DetLines.size():" << DetLines.size() << endl;//DetLines:虚线
		for (int i = 0; i < DetLines.size(); i++){
            line(PIC, Point(DetLines.at(i)[0], DetLines.at(i)[1]), Point(DetLines.at(i)[2], DetLines.at(i)[3]),Scalar(0, 255, 0),2);  //画出边缘图
		}
	}

    //imshow("Rect_Src",PIC);
    imwrite("Rect_Src.bmp", PIC);
    //fourcorner.clear();
    //waitKey(4);
	if(IsRect){
		cout <<"Isrect is true in circle put function" << endl;
        //vector<Point2f> pointCorner;
        fourcorner = ObtainCrossPoint(ResLines.at(0),ResLines.at(1),ResLines.at(2),ResLines.at(3));
        //cout<<"pointCorner[0]:  "<<pointCorner[0].x<<"  "<<pointCorner[0].y<<endl;
        //cout<<"pointCorner[1]:  "<<pointCorner[1].x<<"  "<<pointCorner[1].y<<endl;
        //cout<<"pointCorner[2]:  "<<pointCorner[2].x<<"  "<<pointCorner[2].y<<endl;
        //cout<<"pointCorner[3]:  "<<pointCorner[3].x<<"  "<<pointCorner[3].y<<endl;
        circle(PIC, fourcorner[0], 5, Scalar(225, 0, 225), 3, 8);//线宽为3，颜色为紫色
        cv::putText(PIC, to_string(0) , cv::Point(fourcorner[0].x-34, fourcorner[0].y), CV_FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(0, 0, 255),2);
        //cv::putText(PIC, to_string(0) , cv::Point(pointCorner[0].x+34, pointCorner[0].y+34), CV_FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(0, 0, 255),2);
		
        circle(PIC, fourcorner[1], 5, Scalar(225, 0, 225), 3, 8);
        cv::putText(PIC, to_string(1) , cv::Point(fourcorner[1].x+14, fourcorner[1].y), CV_FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(0, 0, 255),2);
        //cv::putText(PIC, to_string(1) , cv::Point(pointCorner[1].x-34, pointCorner[1].y+34), CV_FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(0, 0, 255),2);

        circle(PIC, fourcorner[2], 5, Scalar(225, 0, 225), 3, 8);
        cv::putText(PIC, to_string(2) , cv::Point(fourcorner[2].x+14, fourcorner[2].y), CV_FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(0, 0, 255),2);
        //cv::putText(PIC, to_string(2) , cv::Point(pointCorner[2].x-34, pointCorner[2].y-34), CV_FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(0, 0, 255),2);
		
        circle(PIC, fourcorner[3], 5, Scalar(225, 0, 225), 3, 8);
        cv::putText(PIC, to_string(3) , cv::Point(fourcorner[3].x-34, fourcorner[3].y), CV_FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(0, 0, 255),2);
        //cv::putText(PIC, to_string(3) , cv::Point(pointCorner[3].x+34, pointCorner[3].y-34), CV_FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(0, 0, 255),2);
		
        imwrite("PIC_circle.bmp",PIC);
        //imshow("circle_Res",PIC);
        //waitKey(200);
		*isDetect = IsRect;
		cout <<endl <<  "value of isDetect：" << *isDetect << endl   << endl  << endl;
        //waitKey(300);
        //flag=1;
        cout<<"fourcorner.size:  "<<fourcorner.size()<<endl;
       /* fourcorner.clear();
        //cout<<"save corner"<<endl;
        for (int i=0;i<4;i++)
        {
            fourcorner.push_back(pointCorner[i]);
        }*/

        return fourcorner;
	}
    *isDetect = IsRect;
	cout << "ISRect in Rect_detect function is  : "  <<  IsRect  << endl ;




	//imwrite("./res/res_PIC.bmp",PIC);
	

	//filename.erase(0,9);
	//cout << "filename_erase:" << filename << endl;
	//string savename = "../image/res/" + filename;
	//imwrite(savename,PIC);


    return fourcorner;
}
