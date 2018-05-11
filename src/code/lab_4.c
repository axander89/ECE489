/*************************
 Included libraries
 **************************/
#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"

#define MAX_POINTS 19
/*************************
 Functions
 **************************/
//void forward_kinematics(float theta1motor, float theta2motor, float theta3motor);
void forward_kinematics();
void inverse_kinematics();
void my_trajectory(float t, float *desired_th, float *desired_th_dot, float *desired_th_double_dot);
void calculate_Jacobian();
void calculate_rot_matrix();
/*************************
 Global variables
 **************************/

/* These two offsets are only used in the main file user_CRSRobot.c  
 You just need to create them here and find the correct offset 
 and then these offset will adjust the encoder readings
 */
float offset_Enc2_rad = -0.442;//-0.37; //Shoulder
float offset_Enc3_rad = 0.226;//0.27;  //Elbow

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

//#pragma DATA_SECTION(theta1array, ".my_arrs")
//float theta1array[100];

#pragma DATA_SECTION(base, ".my_arrs")
float base[100];

#pragma DATA_SECTION(shoulder, ".my_arrs")
float shoulder[100];

#pragma DATA_SECTION(elbow, ".my_arrs")
float elbow[100];

long arrayindex = 0;

// Variables to print motor angles
float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

// time for trajectory
float t = 0.0;

// For dh angles
float dhtheta1 = 0;
float dhtheta2 = 0;
float dhtheta3 = 0;

// For motor angles
float mtheta1 = 0;
float mtheta2 = 0;
float mtheta3 = 0;

// Assign these float to the values that would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;


// Link_1:
float ek_1 = 0;
float ek_1_old = 0;
float theta_1_old = 0;
float omega_1_old1 = 0;
float omega_1_old2 = 0;
float omega_1 = 0;
float threshold_1 = 0.1;

// Link_2:
float ek_2 = 0;
float ek_2_old = 0;
float theta_2_old = 0;
float omega_2_old1 = 0;
float omega_2_old2 = 0;
float omega_2 = 0;
float threshold_2 = 0.1;

// Link_3:
float ek_3 = 0;
float ek_3_old = 0;
float theta_3_old = 0;
float omega_3_old1 = 0;
float omega_3_old2 = 0;
float omega_3 = 0;
float threshold_3 = 0.1;


//Variables for friction compensation
float Vpos_1 = 0.2;
float Cpos_1 = 0.3637;
float Vneg_1 = 0.22;
float Cneg_1 = -0.35;
float slope_1 = 3.6;

float Vpos_2 = 0.20;
float Cpos_2 = 0.4759*0.75;
float Vneg_2 = 0.287*0.75;
float Cneg_2 = -0.5031*0.75;
float slope_2 = 3.6;

float Vpos_3 = 0.1922*0.9;
float Cpos_3 = 0.5339*0.9;
float Vneg_3 = 0.2132*0.9;
float Cneg_3 = -0.5190*0.9;
float slope_3 = 3.6;


// variables for Task Space PD control
float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;

float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;

float thetaz = 0;
float thetax = 0;
float thetay = 0;

float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;

float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;

float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;


// x axis
float kp_x = 0.5;
float kd_x = 0.025;
float kp_x_n = 0.5;
float kd_x_n = 0.025;

float x = 0;
float x_d = 0;
float x_d_dot = 0;
float x_old = 0;
float ek_x = 0;
float ek_x_dot = 0;
float x_dot = 0;
float x_dot_old1 = 0;
float x_dot_old2 = 0;

// y axis
float kp_y = 0.5;
float kd_y = 0.025;
float kp_y_n = 0.5;
float kd_y_n = 0.025;

float y = 0;
float y_d = 0;
float y_d_dot = 0;
float y_old = 0;
float ek_y = 0;
float ek_y_dot = 0;
float y_dot = 0;
float y_dot_old1 = 0;
float y_dot_old2 = 0;

// z axis
float kp_z = 0.5;
float kd_z = 0.025;
float kp_z_n = 0.5;
float kd_z_n = 0.025;

float z = 0;
float z_d = 0;
float z_d_dot = 0;
float z_old = 0;
float ek_z = 0;
float ek_z_dot = 0;
float z_dot = 0;
float z_dot_old1 = 0;
float z_dot_old2 = 0;


typedef struct {
    float x;
    float y;
    float z;
    float rotation;
    float kp_x;
    float kd_x;
    float kp_y;
    float kd_y;
    float kp_z;
    float kd_z;
    float speed;
} my_state;

my_state array_to[MAX_POINTS];

int cur_state = 1;

// straight line
float x_a = 0;
float y_a = 0;
float z_a = 0;

float x_b = 0;
float y_b = 0;
float z_b = 0;

float delta_x = 0;
float delta_y = 0;
float delta_z = 0;

//float speed = 5;
float t_total = 0;
float t_start = 0;

int dir_flag = 0;

float x_hole = 0.69;
float y_hole = 13.76;
float z_zig_zag = 8.4;
float fast_speed = 40;
int cur_state_flag = 0;

/*************************
 Functions
 **************************/
/* This function is called every 1 ms:
 Inputs: theta1motor,theta2motor,theta3motor,*tau1,*tau2,*tau3, error
 Output:
 Global variables changing: printtheta1motor, printtheta2motor, printtheta3motor
 Simulink_PlotVar1, Simulink_PlotVar2, Simulink_PlotVar3, Simulink_PlotVar4
 mycount
 */ 
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {
    
    
    
    /*******/
    int i;
    for(i=0; i<MAX_POINTS; i++)
    {
        array_to[i].kp_x = kp_x;
        array_to[i].kd_x = kd_x;
        array_to[i].kp_y = kp_y;
        array_to[i].kd_y = kd_y;
        array_to[i].kp_z = kp_z;
        array_to[i].kd_z = kd_z;
        array_to[i].rotation = 0;
    }
    
    array_to[0].x = 5.47;
    array_to[0].y = 0;
    array_to[0].z = 16.79;
    array_to[0].speed = fast_speed+5;
    
    array_to[1].x = x_hole;
    array_to[1].y = y_hole;
    array_to[1].z = 16.79;
    array_to[1].speed = fast_speed-5;
    
    array_to[2].x = x_hole;
    array_to[2].y = y_hole;
    array_to[2].z = 8.5;
    array_to[2].kp_x = kp_x/20;
    array_to[2].kd_x = kd_x/20;
    array_to[2].kp_y = kp_y/20;
    array_to[2].kd_y = kd_y/20;
    array_to[2].speed = 15; //5;
    
    array_to[3].x = x_hole;
    array_to[3].y = y_hole;
    array_to[3].z = 5;
    array_to[3].kp_x = kp_x/20;
    array_to[3].kd_x = kd_x/20;
    array_to[3].kp_y = kp_y/20;
    array_to[3].kd_y = kd_y/20;
    array_to[3].speed = 15; //5;
    
    array_to[4].x = x_hole;
    array_to[4].y = y_hole;
    array_to[4].z = 8.5;
    array_to[4].speed = 20;
    
    array_to[5].x = 12.32;
    array_to[5].y = 4.08;
    array_to[5].z = 8.5;
    array_to[5].speed = 7;
    
    array_to[6].x = 14.3;
    array_to[6].y = 4.35;
    array_to[6].z = z_zig_zag-0.1;  //
    array_to[6].rotation = 36.87*PI/180;
    array_to[6].kp_x = kp_x/10;
    array_to[6].kd_x = kd_x/10;
    array_to[6].speed = 10;
    
    array_to[7].x = 15.86;
    array_to[7].y = 2.62;
    array_to[7].z = z_zig_zag-0.1;
    array_to[7].rotation = 36.87*PI/180;
    array_to[7].kp_y = kp_y/10;
    array_to[7].kd_y = kd_y/10;
    array_to[7].speed = 7;
    
    array_to[8].x = 15.81;
    array_to[8].y = 2.19;
    array_to[8].z = z_zig_zag-0.1;
    array_to[8].kp_x = kp_x/10;
    array_to[8].kd_x = kd_x/10;
    array_to[8].kp_y = kp_y/10;
    array_to[8].kd_y = kd_y/10;
    array_to[8].speed = 7;
    
    array_to[9].x = 15.18;
    array_to[9].y = 1.95;
    array_to[9].z = z_zig_zag-0.1;
    array_to[9].rotation = -15*PI/180;
    array_to[9].kp_y = kp_y/10;
    array_to[9].kd_y = kd_y/10;
    array_to[9].speed = 10;
    
    array_to[10].x = 12.98;
    array_to[10].y = 2.32;
    array_to[10].z = z_zig_zag-0.1;
    array_to[10].rotation = 36.87*PI/180;
    array_to[10].kp_y = kp_y/10;
    array_to[10].kd_y = kd_y/10;
    array_to[10].speed = 7;
    
    array_to[11].x = 12.4;
    array_to[11].y = 1.83;
    array_to[11].z = z_zig_zag-0.1;
    array_to[11].rotation = 0;
    array_to[11].kp_x = kp_x/10;
    array_to[11].kd_x = kd_x/10;
    array_to[11].kp_y = kp_y/10;
    array_to[11].kd_y = kd_y/10;
    array_to[11].speed = 7;
    
    array_to[12].x = 12.43;
    array_to[12].y = 1.51;
    array_to[12].z = z_zig_zag-0.1;
    array_to[12].rotation = 36.87*PI/180;
    array_to[12].kp_x = kp_x/10;
    array_to[12].kd_x = kd_x/10;
    array_to[12].speed = 10;
    
    array_to[13].x = 15.21;
    array_to[13].y = -1.49;
    array_to[13].z = z_zig_zag-0.1;
    array_to[13].speed = fast_speed;
    
    array_to[14].x = 15.21;
    array_to[14].y = -1.49;
    array_to[14].z = 14.5;
    array_to[14].speed = 25;
    
    array_to[15].x = 14.54;
    array_to[15].y = -5.05;
    array_to[15].z = 14.5;
    array_to[15].kp_z = kp_z*2/3;
    array_to[15].kd_z = kd_z*2/3;
    array_to[15].speed = 2.5;
    
    array_to[16].x = 14.54;
    array_to[16].y = -5.05;
    array_to[16].z = 13.44;
    array_to[16].kp_z = kp_z*2/3;
    array_to[16].kd_z = kd_z*2/3;
    array_to[16].speed = 10;
    
    array_to[17].x = 14.54;
    array_to[17].y = -5.05;
    array_to[17].z = 14.5;
    array_to[17].speed = fast_speed+10;
    
    array_to[18].x = 5.47;
    array_to[18].y = 0;
    array_to[18].z = 16.79;
    array_to[18].speed = fast_speed+10;
    /******/
    
    
    
    x_a = array_to[cur_state-1].x;
    y_a = array_to[cur_state-1].y;
    z_a = array_to[cur_state-1].z;
    x_b = array_to[cur_state].x;
    y_b = array_to[cur_state].y;
    z_b = array_to[cur_state].z;
    thetaz = array_to[cur_state-1].rotation;
    kp_x_n = array_to[cur_state-1].kp_x;
    kd_x_n = array_to[cur_state-1].kd_x;
    kp_y_n = array_to[cur_state-1].kp_y;
    kd_y_n = array_to[cur_state-1].kd_y;
    kp_z_n = array_to[cur_state-1].kp_z;
    kd_z_n = array_to[cur_state-1].kd_z;
    
    cosq1 = cos(theta1motor);
    sinq1 = sin(theta1motor);
    cosq2 = cos(theta2motor);
    sinq2 = sin(theta2motor);
    cosq3 = cos(theta3motor);
    sinq3 = sin(theta3motor);
    calculate_Jacobian();
    
    calculate_rot_matrix();
    
    delta_x = x_b - x_a;
    delta_y = y_b - y_a;
    delta_z = z_b - z_a;
    t_total = (sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z))/array_to[cur_state-1].speed;
    
    if(cur_state_flag==1)
    {
        x_d = x_b;
        y_d = y_b;
        z_d = z_b;
    }
    else if(cur_state_flag==2)
    {
        if(t>0.35)
        {
            t = 0;
            cur_state_flag = 0;
            cur_state++;
        }
        x_d = x_b;
        y_d = y_b;
        z_d = z_b;
    }
    else
    {
        if(t < t_total)
        {
            x_d = (delta_x*t/t_total) + x_a;
            y_d = (delta_y*t/t_total) + y_a;
            z_d = (delta_z*t/t_total) + z_a;
        }
        else
        {
            x_d = x_b;
            y_d = y_b;
            z_d = z_b;
            t = 0;
            if(cur_state+1 < MAX_POINTS)
            {
                if(cur_state==16)
                cur_state_flag = 2;
                else
                cur_state++;
            }
            else
            cur_state_flag = 1;
        }
    }
    
    x_d_dot = 0.0;
    y_d_dot = 0.0;
    z_d_dot = 0.0;
    
    forward_kinematics();
    
    ek_x = x_d - x;
    ek_y = y_d - y;
    ek_z = z_d - z;
    
    // calculate position derivatives
    x_dot = (x - x_old) / 0.001;
    x_dot = (x_dot + x_dot_old1 + x_dot_old2) / 3.0;
    x_dot_old2 = x_dot_old1;
    x_dot_old1 = x_dot;
    
    y_dot = (y - y_old) / 0.001;
    y_dot = (y_dot + y_dot_old1 + y_dot_old2) / 3.0;
    y_dot_old2 = y_dot_old1;
    y_dot_old1 = y_dot;
    
    z_dot = (z - z_old) / 0.001;
    z_dot = (z_dot + z_dot_old1 + z_dot_old2) / 3.0;
    z_dot_old2 = z_dot_old1;
    z_dot_old1 = z_dot;
    
    // saving old coords
    x_old = x;
    y_old = y;
    z_old = z;
    
    ek_x_dot = x_d_dot - x_dot;
    ek_y_dot = y_d_dot - y_dot;
    ek_z_dot = z_d_dot - z_dot;
    
    float Fx_n = kp_x_n*(RT11*ek_x + RT12*ek_y + RT13*ek_z) + kd_x_n*(RT11*ek_x_dot + RT12*ek_y_dot + RT13*ek_z_dot);
    float Fy_n = kp_y_n*(RT21*ek_x + RT22*ek_y + RT23*ek_z) + kd_y_n*(RT21*ek_x_dot + RT22*ek_y_dot + RT23*ek_z_dot);
    float Fz_n = kp_z_n*(RT31*ek_x + RT32*ek_y + RT33*ek_z) + kd_z_n*(RT31*ek_x_dot + RT32*ek_y_dot + RT33*ek_z_dot);
    
    float Fx = R11*Fx_n + R12*Fy_n + R13*Fz_n;
    float Fy = R21*Fx_n + R22*Fy_n + R23*Fz_n;
    float Fz = R31*Fx_n + R32*Fy_n + R33*Fz_n;
    
    float temp_tau1 = JT_11*Fx + JT_12*Fy + JT_13*Fz;
    float temp_tau2 = JT_21*Fx + JT_22*Fy + JT_23*Fz;
    float temp_tau3 = JT_31*Fx + JT_32*Fy + JT_33*Fz;
    
    /******/
    
    // calculate velocity
    omega_1 = (theta1motor - theta_1_old) / 0.001;
    omega_1 = (omega_1 + omega_1_old1 + omega_1_old2) / 3.0;
    omega_1_old2 = omega_1_old1;
    omega_1_old1 = omega_1;
    
    omega_2 = (theta2motor - theta_2_old) / 0.001;
    omega_2 = (omega_2 + omega_2_old1 + omega_2_old2) / 3.0;
    omega_2_old2 = omega_2_old1;
    omega_2_old1 = omega_2;
    
    omega_3 = (theta3motor - theta_3_old) / 0.001;
    omega_3 = (omega_3 + omega_3_old1 + omega_3_old2) / 3.0;
    omega_3_old2 = omega_3_old1;
    omega_3_old1 = omega_3;
    
    
    // defining old thetas
    theta_1_old = theta1motor;
    theta_2_old = theta2motor;
    theta_3_old = theta3motor;
    /******/
    
    float u_fric1;
    if(omega_1>0.1)
    {
        u_fric1 = Vpos_1*omega_1 + Cpos_1;
    }
    else if(omega_1<-0.1)
    {
        u_fric1 = Vneg_1*omega_1 + Cneg_1;
    }
    else
    {
        u_fric1 = slope_1*omega_1;
    }
    
    float u_fric2;
    if(omega_2>0.05)
    {
        u_fric2 = Vpos_2*omega_2 + Cpos_2;
    }
    else if(omega_2<-0.05)
    {
        u_fric2 = Vneg_2*omega_2 + Cneg_2;
    }
    else
    {
        u_fric2 = slope_2*omega_2;
    }
    
    float u_fric3;
    if(omega_3>0.05)
    {
        u_fric3 = Vpos_3*omega_3 + Cpos_3;
    }
    else if(omega_3<-0.05)
    {
        u_fric3 = Vneg_3*omega_3 + Cneg_3;
    }
    else
    {
        u_fric3 = slope_3*omega_3;
    }
    
    // Init. varaibles:
    *tau1 = temp_tau1 + u_fric1; //base
    *tau2 = temp_tau2 + u_fric2; //shoulder
    *tau3 = temp_tau3 + u_fric3; //elbow
    
    //Motor torque limitation(Max: 5 Min: -5)
    // save past states
    if ((mycount%50)==0) {
        
        base[arrayindex] = theta1motor;         //base
        shoulder[arrayindex] = theta2motor;     //shoulder
        elbow[arrayindex] = theta3motor;        //elbow
        
        if (arrayindex >= 100) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }
        
    }
    
    // Print variables:
    if ((mycount%500)==0) {
        if (whattoprint > 0.5) {
            serial_printf(&SerialA, "I love robotics\n\r");
        } else {
            // Assign variables to be printed:
            printtheta1motor = theta1motor;
            printtheta2motor = theta2motor;
            printtheta3motor = theta3motor;
            
            // compute forward kinematics:
            //forward_kinematics(theta1motor, theta2motor, theta3motor);
            
            // compute inverse kinematics:            
            //            inverse_kinematics();
            
            SWI_post(&SWI_printf); //Using a SWI to fix SPI issue from sending too many floats.
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        //        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }
    
    /* Uncomment this section to blink LED on Emergency Stop Box
     if((mycount%400)==0){
     GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
     }
     
     if(((mycount+100)%400)==0){
     GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
     }
     */
    
    // Assign data to be sent to Simulink
    Simulink_PlotVar1 = x;
    Simulink_PlotVar2 = y;
    Simulink_PlotVar3 = z;
    Simulink_PlotVar4 = 0;
    
    // Increment mycount variable
    mycount++;
    
    // increment time for trajectory
    t = t + 0.001;
    //    if(t>8){
    //        t = 0.0;
    //    }
}

/* Function to print to Tera terminal:
 Inputs:
 Output:
 Global variables changing:
 */ 
void printing(void){
    serial_printf(&SerialA, "Encoder readings     : th_1: %.2f, th_2: %.2f, th_3: %.2f \n\r", printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);
    serial_printf(&SerialA, "Fwd. Kin             : X: %.2f, Y: %.2f, Z: %.2f \n\r", x, y, z);
    //serial_printf(&SerialA, "DH angles            : DH_th1: %.2f, DH_th2: %.2f, DH_th3: %.2f  \n\r", dhtheta1, dhtheta2, dhtheta3);
    //serial_printf(&SerialA, "Inv. kin motor angles: m_th1: %.2f, m_th2: %.2f, m_th3: %.2f \n\r\n\r", mtheta1, mtheta2, mtheta3);
}


/* Function to create the step by step theta for a desired trajectory
 Inputs: t: time; desired_th:  for zero degree polynomial. . .
 Output:
 Global variables changing:
 */
void my_trajectory(float t, float *desired_th, float *desired_th_dot, float *desired_th_double_dot)
{
    //    if (t>2)
    //    {
    //        *desired_th = 0;
    //        *desired_th_dot = 0;
    //        *desired_th_double_dot = 0;
    //    }
    //    else if (t>=0 && t<=1)
    //    {
    //        *desired_th = coefs_1i + coefs_2i*t + coefs_3i*t*t + coefs_4i*t*t*t;
    //        *desired_th_dot = coefs_2i + 2*coefs_3i*t + 3*coefs_4i*t*t;
    //        *desired_th_double_dot = 2*coefs_3i + 6*coefs_4i*t;
    //    }
    //    else if (t>1 && t<=2)
    //    {
    //        *desired_th = coefs_1d + coefs_2d*t + coefs_3d*t*t + coefs_4d*t*t*t;
    //        *desired_th_dot = coefs_2d + 2*coefs_3d*t + 3*coefs_4d*t*t;
    //        *desired_th_double_dot = 2*coefs_3d + 6*coefs_4d*t;
    //    }
    
    float mystep;
    if(t>=0 && t<4)
    mystep = 0.25;
    else if(t>=4 && t<8)
    mystep = 0.75;
    else
    mystep = 0;
    //implement_discrete_tf(&trajectory, mystep, desired_th, desired_th_dot, desired_th_double_dot);
}



void calculate_Jacobian()
{
    JT_11 = -10*sinq1*(cosq3 + sinq2);
    JT_12 = 10*cosq1*(cosq3 + sinq2);
    JT_13 = 0;
    JT_21 = 10*cosq1*(cosq2 - sinq3);
    JT_22 = 10*sinq1*(cosq2 - sinq3);
    JT_23 = -10*(cosq3 + sinq2);
    JT_31 = -10*cosq1*sinq3;
    JT_32 = -10*sinq1*sinq3;
    JT_33 = -10*cosq3;
}

void calculate_rot_matrix()
{
    // Rotation zxy and its Transpose
    cosz = cos(thetaz);
    sinz = sin(thetaz);
    cosx = cos(thetax);
    sinx = sin(thetax);
    cosy = cos(thetay);
    siny = sin(thetay);
    RT11 = R11 = cosz*cosy-sinz*sinx*siny;
    RT21 = R12 = -sinz*cosx;
    RT31 = R13 = cosz*siny+sinz*sinx*cosy;
    RT12 = R21 = sinz*cosy+cosz*sinx*siny;
    RT22 = R22 = cosz*cosx;
    RT32 = R23 = sinz*siny-cosz*sinx*cosy;
    RT13 = R31 = -cosx*siny;
    RT23 = R32 = sinx;
    RT33 = R33 = cosx*cosy;
}




/* Function to compute forward kinematics:
 Inputs: theta1motor, theta2motor, theta3motor
 Output:
 Global variables changing: x,y,z
 */
//void forward_kinematics(float theta1motor, float theta2motor, float theta3motor)
void forward_kinematics()
{
    //    x = 10*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    //    y = 10*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    //    z = 10*(1 + cos(theta2motor) - sin(theta3motor));
    x = 10*cosq1*(cosq3 + sinq2);
    y = 10*sinq1*(cosq3 + sinq2);
    z = 10*(1 + cosq2 - sinq3);
    
}

/* Function to compute inverse kinematics:
 Inputs:
 Output:
 Global variables changing: dhtheta1, dhtheta2, dhtheta3, mtheta1, mtheta2, mtheta3
 */
void inverse_kinematics()
{
    // Compute geometric variables L, alpha and beta
    float L = sqrt(x*x + y*y + (z-10)*(z-10));
    float alpha = acos(L/(2*10));
    float beta = atan2(z-10, sqrt(x*x + y*y));
    
    // compute dh angles:
    dhtheta1 = atan2(y, x)*180/PI;
    dhtheta2 = -(alpha+beta)*180/PI;
    dhtheta3 = 2*alpha*180/PI;
    
    // Compute motor angles:
    mtheta1 = dhtheta1;
    mtheta2 = dhtheta2 + 90;
    mtheta3 = dhtheta3 - 90 + mtheta2;
}
