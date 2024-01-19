SDK_DEMO_PATH ?= .
BL_SDK_BASE ?= $(SDK_DEMO_PATH)/../..

export BL_SDK_BASE

CHIP ?= bl616
BOARD ?= bl616dk
CROSS_COMPILE ?= riscv64-unknown-elf-

# add custom cmake definition
#cmake_definition+=-Dxxx=sss

include $(BL_SDK_BASE)/project.build




# /*
#  Draws a 3d rotating cube on the freetronics OLED screen.
#  Original code was found at http://forum.freetronics.com/viewtopic.php?f=37&t=5495
#  Thanks to Adafruit at http://www.adafruit.com for the great display and sensor libraries
#  */
# #include <SPI.h>
# #include <Adafruit_SSD1306.h>
# #include <Adafruit_GFX.h>
# #include <Wire.h>

# //MPU 
# const int MPU=0x68;  // I2C address of the MPU-6050
# int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

# //Display
# #define OLED_RESET 4
# Adafruit_SSD1306 display(OLED_RESET);

# // OLED I2C bus address
# #define OLED_address  0x3c 

# float xx,xy,xz;
# float yx,yy,yz;
# float zx,zy,zz;

# float fact;

# int Xan,Yan;

# int Xoff;
# int Yoff;
# int Zoff;

# struct Point3d
# {
#   int x;
#   int y;
#   int z;
# };

# struct Point2d
# {
#   int x;
#   int y;
# };



# int LinestoRender; // 要渲染的线。
# int OldLinestoRender; // 线条以进行渲染，以防其发生变化。这样可以确保旧的线条都被删除。

# struct Line3d
# {
#   Point3d p0;
#   Point3d p1;
# };

# struct Line2d
# {
#   Point2d p0;
#   Point2d p1;
# };

# Line3d Lines[12];  //要渲染的线数
# Line2d Render[12];
# Line2d ORender[12];

# /***********************************************************************************************************************************/
#//设置三维变换的全局变量。通过“流程”发送的任何点都将使用这些数字进行转换。
#//只有当Xan或Yan被更改时才需要调用。
# void SetVars(void)
# {
#   float Xan2,Yan2,Zan2;
#   float s1,s2,s3,c1,c2,c3;
  
#   Xan2 = Xan / fact; // 将度数转换为弧度。
#   Yan2 = Yan / fact;
  
#   // Zan假定为零
    
#   s1 = sin(Yan2);
#   s2 = sin(Xan2);
  
#   c1 = cos(Yan2);
#   c2 = cos(Xan2);

#   xx = c1;
#   xy = 0; 
#   xz = -s1;

#   yx = (s1 * s2);
#   yy = c2;
#   yz = (c1 * s2);

#   zx = (s1 * c2);
#   zy = -s2;
#   zz = (c1 * c2);
# }


# /***********************************************************************************************************************************/
#//处理x1，y1，z1，返回由SetVars（）中设置的变量转换后的rx1，ry1
#//这里的浮点值相当重。
#//使用了一堆全局变量。可以用结构重写，但不值得这么做。
# void ProcessLine(struct Line2d *ret,struct Line3d vec)
# {
#   float zvt1;
#   int xv1,yv1,zv1;

#   float zvt2;
#   int xv2,yv2,zv2;
  
#   int rx1,ry1;
#   int rx2,ry2;
 
#   int x1;
#   int y1;
#   int z1;

#   int x2;
#   int y2;
#   int z2;
  
#   int Ok;
  
#   x1=vec.p0.x;
#   y1=vec.p0.y;
#   z1=vec.p0.z;

#   x2=vec.p1.x;
#   y2=vec.p1.y;
#   z2=vec.p1.z;

#   Ok=0; // 默认为“不确定”

#   xv1 = (x1 * xx) + (y1 * xy) + (z1 * xz);
#   yv1 = (x1 * yx) + (y1 * yy) + (z1 * yz);
#   zv1 = (x1 * zx) + (y1 * zy) + (z1 * zz);

#   zvt1 = zv1 - Zoff;


#   if( zvt1 < -5){
#     rx1 = 256 * (xv1 / zvt1) + Xoff;
#     ry1 = 256 * (yv1 / zvt1) + Yoff;
#     Ok=1; // ok we are alright for point 1.
#   }
  
  
#   xv2 = (x2 * xx) + (y2 * xy) + (z2 * xz);
#   yv2 = (x2 * yx) + (y2 * yy) + (z2 * yz);
#   zv2 = (x2 * zx) + (y2 * zy) + (z2 * zz);

#   zvt2 = zv2 - Zoff;


#   if( zvt2 < -5){
#     rx2 = 256 * (xv2 / zvt2) + Xoff;
#     ry2 = 256 * (yv2 / zvt2) + Yoff;
#   } else
#   {
#     Ok=0;
#   }
  
#   if(Ok==1){
#   ret->p0.x=rx1;
#   ret->p0.y=ry1;

#   ret->p1.x=rx2;
#   ret->p1.y=ry2;
#   }
#  // 这里的if是对越界的检查。这里需要更多的代码来“保护”那些不正常的行，这样它们就不会被绘制并导致屏幕垃圾。
 
# }



# /***********************************************************************************************************************************/
# void setup() {
#  Wire.begin();
#   display.begin(SSD1306_SWITCHCAPVCC, 0x3c);  // initialize with the I2C addr 0x3D (for the 128x64)
#   display.clearDisplay();   // 清除屏幕和缓冲区


# Wire.begin();

#   fact = 180 / 3.14159265358979323846264338327950; // conversion from degrees to radians.
  
#   Xoff = 90; // 将3d转换空间的中心定位到OLED屏幕的中心。这通常是screen_x_size/2。
#   Yoff = 10; // 屏幕_y_size/2
#   Zoff = 750;   //立方体尺寸，较大数量=较小立方体

# // 线段来绘制立方体。基本上p0到p1。p1到p2。p2到p3，依此类推。

# // 正面。

#   Lines[0].p0.x=-20;
#   Lines[0].p0.y=-20;
#   Lines[0].p0.z=20;
#   Lines[0].p1.x=20;
#   Lines[0].p1.y=-20;
#   Lines[0].p1.z=20;

#   Lines[1].p0.x=20;
#   Lines[1].p0.y=-20;
#   Lines[1].p0.z=20;
#   Lines[1].p1.x=20;
#   Lines[1].p1.y=20;
#   Lines[1].p1.z=20;

#   Lines[2].p0.x=20;
#   Lines[2].p0.y=20;
#   Lines[2].p0.z=20;
#   Lines[2].p1.x=-20;
#   Lines[2].p1.y=20;
#   Lines[2].p1.z=20;

#   Lines[3].p0.x=-20;
#   Lines[3].p0.y=20;
#   Lines[3].p0.z=20;
#   Lines[3].p1.x=-20;
#   Lines[3].p1.y=-20;
#   Lines[3].p1.z=20;


# //背面。

#   Lines[4].p0.x=-20;
#   Lines[4].p0.y=-20;
#   Lines[4].p0.z=-20;
#   Lines[4].p1.x=20;
#   Lines[4].p1.y=-20;
#   Lines[4].p1.z=-20;

#   Lines[5].p0.x=20;
#   Lines[5].p0.y=-20;
#   Lines[5].p0.z=-20;
#   Lines[5].p1.x=20;
#   Lines[5].p1.y=20;
#   Lines[5].p1.z=-20;

#   Lines[6].p0.x=20;
#   Lines[6].p0.y=20;
#   Lines[6].p0.z=-20;
#   Lines[6].p1.x=-20;
#   Lines[6].p1.y=20;
#   Lines[6].p1.z=-20;

#   Lines[7].p0.x=-20;
#   Lines[7].p0.y=20;
#   Lines[7].p0.z=-20;
#   Lines[7].p1.x=-20;
#   Lines[7].p1.y=-20;
#   Lines[7].p1.z=-20;

# // 现在是4条边缘线。

#   Lines[8].p0.x=-20;
#   Lines[8].p0.y=-20;
#   Lines[8].p0.z=20;
#   Lines[8].p1.x=-20;
#   Lines[8].p1.y=-20;
#   Lines[8].p1.z=-20;

#   Lines[9].p0.x=20;
#   Lines[9].p0.y=-20;
#   Lines[9].p0.z=20;
#   Lines[9].p1.x=20;
#   Lines[9].p1.y=-20;
#   Lines[9].p1.z=-20;

#   Lines[10].p0.x=-20;
#   Lines[10].p0.y=20;
#   Lines[10].p0.z=20;
#   Lines[10].p1.x=-20;
#   Lines[10].p1.y=20;
#   Lines[10].p1.z=-20;

#   Lines[11].p0.x=20;
#   Lines[11].p0.y=20;
#   Lines[11].p0.z=20;
#   Lines[11].p1.x=20;
#   Lines[11].p1.y=20;
#   Lines[11].p1.z=-20;

#   LinestoRender=12;
#   OldLinestoRender=LinestoRender;
 
#   // Initialize MPU
#   Wire.beginTransmission(MPU);
#   Wire.write(0x6B);  // PWR_MGMT_1 register
#   Wire.write(0);     // 设置为零（唤醒MPU-6050）
#   Wire.endTransmission(true);
  
# }
# /***********************************************************************************************************************************/
# void RenderImage( void)
# {
#//在擦除旧的线之后渲染所有的线。
#//这里是与OLED实际对接的唯一代码。因此，如果您使用不同的lib，这里就是更改它的地方。
#  for (int i=0; i<OldLinestoRender; i++ )
#   {
#    display.drawLine(ORender[i].p0.x,ORender[i].p0.y,ORender[i].p1.x,ORender[i].p1.y, BLACK); // erase the old lines.
#   }

    
#   for (int i=0; i<LinestoRender; i++ )
#   {
#    display.drawLine(Render[i].p0.x,Render[i].p0.y,Render[i].p1.x,Render[i].p1.y, WHITE);
#   }
#   OldLinestoRender=LinestoRender;
  
  
#   Wire.beginTransmission(MPU);
#   Wire.write(0x3B);  // 从寄存器0x3B开始（ACCEL_XOUT_H）
#   Wire.endTransmission(true);
#   Wire.requestFrom(MPU,14,true);  // 请求总共14个寄存器
#   AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
#   AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
#   AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
#   Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
#   GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
#   GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
#   GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
#   // // text display tests
#   // display.setTextSize(1);
#   // display.setTextColor(WHITE);
#   // display.setCursor(0,0);
#   // //Display ACC
#   // display.print("AcX: ");
#   // display.println(AcX);
#   // display.print("AcY: ");
#   // display.println(AcY);
#   // //Display gyro
#   // display.print("GyX: ");
#   // display.println(GyX);
#   // display.print("GyY: ");
#   // display.println(GyY);
#   // //delay(10);

#   // display.print("AcX: ");
#   // display.println(AcX);
#   // display.print("AcY: ");
#   // display.println(AcY);
#   // //Display gyro
#   // display.print("GyX: ");
#   // display.println(GyX);
#   // display.print("GyY: ");
#   // display.println(GyY);
# }


# /***********************************************************************************************************************************/

# void loop() {
#   display.display();
#   display.clearDisplay();   // 清除屏幕和缓冲区
 
#  // PIX=GREEN; // 所有画的线的颜色都将是绿色的，直到改变为止。
  
#   //For cube rotation
#   int xOut=0;
#   int yOut=0;
  
#   xOut = map(AcX,-17000,17000,-50,50);
#     yOut = map(AcY,-17000,17000,-50,50);
  
#  Xan+=xOut;
#  Yan+=yOut;
  

#   Yan=Yan % 360;
#   Xan=Xan % 360; // 防止溢出。
  


#   SetVars(); //设置全局变量以进行转换。

#   for(int i=0; i<LinestoRender ; i++)
#   {
#     ORender[i]=Render[i]; // 存储旧线段，以便稍后删除。
#     ProcessLine(&Render[i],Lines[i]); // 将三维线段转换为二维线段。
#   }  
  
#   RenderImage(); // 去画吧！!
  
  
 
# }