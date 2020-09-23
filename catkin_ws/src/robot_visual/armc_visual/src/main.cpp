#include <iostream>
#include "armc_visual/capture.h"

using namespace std;

int main()
{
    int a=1;
    float tag_T_cam[4][4]={0.0f};
    float twist_1[6]={0.0f};
    float twist_2[6]={0.0f};
    int flag=0;
    if (a == 0)
    {
          cout<<"a0= "<<a<<endl;
        flag=lock_capture_pose(tag_T_cam);
          cout<<"a1= "<<a<<endl;

    }
    else
    {
        flag=handle_capture_pose(twist_1,twist_2);
        cout<<"a2= "<<a<<endl;
    }
    return 0;
}
