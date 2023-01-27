#include <M5StickC.h>
#include <driver/i2s.h>

#define READ_LEN (2*128)
#define GAIN_FACTOR 3
#define MIC_Unit 33

long unsigned int a2;

//---------------------------------------------------------------------------//
//              0,1, 2, 3 ,4 ,5 ,6 , 7  ,8  ,9  ,10 ,11 ,12
int  in[128]; //C,C#,D ,D#,E ,F ,F#, G  ,G# ,A  ,A# ,B  ,C
byte NoteV[13]={8,23,40,57,76,96,116,138,162,187,213,241,255};
float f_peaks[8]; // top 8 frequencies peaks in descending order
String chord_out;
//---------------------------------------------------------------------------//

//-----------------------------Chord Detection Function----------------------------------------------//
// Documentation on Chord_detection:https://www.instructables.com/member/abhilash_patel/instructables/
// Code Written By: Abhilash Patel
// Contact: abhilashpatel121@gmail.com
// this code written for arduino Nano board (should also  work for UNO) or better board
// this code won't work for any board having RAM less than 2kb,
// More accurate detection can be carried out on more powerful borad by increasing sample size



//-----------------------------FFT Function----------------------------------------------//
// Documentation on EasyFFT:https://www.instructables.com/member/abhilash_patel/instructables/
// EasyFFT code optimised for 128 sample size to reduce mamory consumtion

float FFT(byte N,float Frequency)
{
byte data[8]={1,2,4,8,16,32,64,128};
int a,c1,f,o,x;
a=N;  
                                 
      for(int i=0;i<8;i++)                 //calculating the levels
         { if(data[i]<=a){o=i;} }
      o=7;
byte in_ps[data[o]]={};     //input for sequencing
float out_r[data[o]]={};   //real part of transform
float out_im[data[o]]={};  //imaginory part of transform
           
x=0;  
      for(int b=0;b<o;b++)                     // bit reversal
         {
          c1=data[b];
          f=data[o]/(c1+c1);
                for(int j=0;j<c1;j++)
                    { 
                     x=x+1;
                     in_ps[x]=in_ps[j]+f;
                    }
         }
 
      for(int i=0;i<data[o];i++)            // update input array as per bit reverse order
         {
          if(in_ps[i]<a)
          {out_r[i]=in[in_ps[i]];}
          if(in_ps[i]>a)
          {out_r[i]=in[in_ps[i]-a];}      
         }


int i10,i11,n1;
float e,c,s,tr,ti;

    for(int i=0;i<o;i++)                                    //fft
    {
     i10=data[i];              // overall values of sine cosine  
     i11=data[o]/data[i+1];    // loop with similar sine cosine
     e=6.283/data[i+1];
     e=0-e;
     n1=0;

          for(int j=0;j<i10;j++)
          {
          c=cos(e*j); 
          s=sin(e*j); 
          n1=j;
          
                for(int k1=0;k1<i11;k1++)
                 {
                 tr=c*out_r[i10+n1]-s*out_im[i10+n1];
                 ti=s*out_r[i10+n1]+c*out_im[i10+n1];
          
                 out_r[n1+i10]=out_r[n1]-tr;
                 out_r[n1]=out_r[n1]+tr;
          
                 out_im[n1+i10]=out_im[n1]-ti;
                 out_im[n1]=out_im[n1]+ti;          
          
                 n1=n1+i10+i10;
                  }       
             }
     }

/*
for(int i=0;i<data[o];i++)
{
Serial.print(out_r[i]);
Serial.print("\t");                                     // uncomment to print RAW o/p    
Serial.print(out_im[i]); Serial.println("i");      
}
*/

//---> here onward out_r contains amplitude and our_in conntains frequency (Hz)
    for(int i=0;i<data[o-1];i++)               // getting amplitude from compex number
        {
         out_r[i]=sqrt((out_r[i]*out_r[i])+(out_im[i]*out_im[i])); // to  increase the speed delete sqrt
         out_im[i]=(i*Frequency)/data[o];
         /*
         Serial.print(out_im[i],2); Serial.print("Hz");
         Serial.print("\t");                            // uncomment to print freuency bin    
         Serial.println(out_r[i]); 
         */
        }

x=0;       // peak detection
   for(int i=1;i<data[o-1]-1;i++)
      {
      if(out_r[i]>out_r[i-1] && out_r[i]>out_r[i+1]) 
      {in_ps[x]=i;    //in_ps array used for storage of peak number
      x=x+1;}    
      }

s=0;
c=0;
    for(int i=0;i<x;i++)             // re arraange as per magnitude
    {
        for(int j=c;j<x;j++)
        {
            if(out_r[in_ps[i]]<out_r[in_ps[j]]) 
                {s=in_ps[i];
                in_ps[i]=in_ps[j];
                in_ps[j]=s;}
        }
    c=c+1;
    }
    
    for(int i=0;i<8;i++)     // updating f_peak array (global variable)with descending order
    {
     // f_peaks[i]=out_im[in_ps[i]];Serial.println(f_peaks[i]);
     f_peaks[i]=(out_im[in_ps[i]-1]*out_r[in_ps[i]-1]+out_im[in_ps[i]]*out_r[in_ps[i]]+out_im[in_ps[i]+1]*out_r[in_ps[i]+1])
     /(out_r[in_ps[i]-1]+out_r[in_ps[i]]+out_r[in_ps[i]+1]);
     // Serial.println(f_peaks[i]);
    } 
}
    
//------------------------------------------------------------------------------------//





int Chord_det()
{ 
  long unsigned int a1,b;
  float a;
  float sum1=0,sum2=0;
  float sampling;
  a1=micros();

        for(int i=0;i<128;i++)
          {
            //a=analogRead(MIC_Unit)-500; 
            a = map(analogRead(33),1800, 4095, -512, 512);    //rough zero shift
            //utilising time between two sample for windowing & amplitude calculation
            sum1=sum1+a;              //to average value
            sum2=sum2+a*a;            // to RMS value
            a=a*(sin(i*3.14/128)*sin(i*3.14/128));   // Hann window
            in[i]=4*a;                // scaling for float to int conversion
            delayMicroseconds(195);   // based on operation frequency range
          }
      b=micros(); 
      sum1=sum1/128;               //average amplitude
      sum2=sqrt(sum2/128);         //RMS amplitude
      sampling= 128000000/(b-a1);  // real time sampling frequency

  //for very low or no amplitude, this code wont start
  //it takes very small aplitude of sound to initiate for value sum2-sum1>3, 
  //change sum2-sum1 threshold based on requirement
  if(sum2-sum1>7){   
       FFT(128,sampling);        //EasyFFT based optimised  FFT code
               
              for(int i=0;i<12;i++){
                in[i]=0;
                } 
 int j=0,k=0;//below loop will convert frequency value to note 
       for(int i=0; i<8;i++)
           {   
           Serial.print(i); 
           Serial.print(":"); 
           Serial.print(f_peaks[i]);   
           Serial.print(" "); 

           if(f_peaks[i]>1040){f_peaks[i]=0;}
           if(f_peaks[i]>=65.4   && f_peaks[i]<=130.8) {f_peaks[i]=255*((f_peaks[i]/65.4)-1);}//C2~C3
           if(f_peaks[i]>=130.8  && f_peaks[i]<=261.6) {f_peaks[i]=255*((f_peaks[i]/130.8)-1);}//C3~C4 A3=220:173.89
           if(f_peaks[i]>=261.6  && f_peaks[i]<=523.25){f_peaks[i]=255*((f_peaks[i]/261.6)-1);}//C4~C5 A4=440:173.89
           if(f_peaks[i]>=523.25 && f_peaks[i]<=1046)  {f_peaks[i]=255*((f_peaks[i]/523.25)-1);}//C5~C6
           if(f_peaks[i]>=1046 && f_peaks[i]<=2093)  {f_peaks[i]=255*((f_peaks[i]/1046)-1);}//C6~C7
           if(f_peaks[i]>255){f_peaks[i]=254;}
           
           Serial.print(" "); 
           Serial.print(f_peaks[i]);
           Serial.print(" "); 

           j=1;k=0;
         while(j==1)
              {
              if(f_peaks[i]<=NoteV[k]){
                f_peaks[i]=k;//Am: f_peaks[i]= 9
                j=0;
                }
              k++;  // a note with max peaks (harmonic) with amplitude priority is selected
              if(k>15){j=0;}
              }

              if(f_peaks[i]==12){
                f_peaks[i]=0;
                }
            
              
              f_peaks[i]++;

              Serial.print(f_peaks[i]);// Am:9 4 0  i=0:root
               Serial.print("(f_peaks[i])");
               Serial.print("\n"); 

           }

           float fir =  f_peaks[0]+1;
           float sec = f_peaks[0]+2;
           float thi = f_peaks[0]+3;//Am third = 12
           float fou = f_peaks[0]+4;
           float fif = f_peaks[0]+5;
           float six = f_peaks[0]+6;//Am fifth = 4
           float sev = f_peaks[0]+7;//Am seventh = 7
           float eig = f_peaks[0]+8;
           float nin = f_peaks[0]+9;
           float ten = f_peaks[0]+10;
           float ele = f_peaks[0]+11;           
           
           if(fir>=12){
             fir = fir - 12;//m3
           }

            if(sec>=12){
             sec = sec - 12;//p5
           }

            if(thi>=12){
             thi = thi - 12; //m7           
            }

            if(fou>=12){
             fou = fou - 12;//m3
           }

           if(fif>=12){
             fif = fif - 12;//m3
           }

           if(six>=12){
             six = six - 12;//m3
           }

           if(sev>=12){
             sev = sev - 12;//m3
           }

           if(eig>=12){
             eig = eig - 12;//m3
           }

           if(nin>=12){
             nin = nin - 12;//m3
           }

           if(ten>=12){
             ten = ten - 12;//m3
           }

           if(ele>=12){
             ele = ele - 12;//m3
           }

          int a=0, b=0, c=0;
             
              for(int i=0;i<8;i++)
             {
            //  if(NoteV[thi]==f_peaks[i] && f_peaks[sev]==f_peaks[i] && f_peaks[ten]==f_peaks[i]){//m7の基本形から✓
            //         Serial.print("yes");
            //  } 

             Serial.print(" "); 
           Serial.print(f_peaks[i]);
           Serial.print(" ");
            Serial.print(thi);
            Serial.print(" ");
            Serial.print(eig); 
              Serial.print(" ");
           
   
                if(f_peaks[i]==thi){///a=2個面の音の検出
                  a=3;
                }

                if(f_peaks[i]==fou){
                  a=4;
                }

                if(f_peaks[i]==sec){
                  a=2;
                }

                if(f_peaks[i]==fir){
                  a=1;
                }


                 if(f_peaks[i]==sev){///b=3個面の音の検出
                  b=7;
                }

                if(f_peaks[i]==fif){
                  b=5;
                }

                if(f_peaks[i]==six){
                  b=6;
                }


                if(f_peaks[i]==ten){///c=4個面の音の検出
                  c=10;
                }

                if(f_peaks[i]==nin){
                  c=9;
                }

                if(f_peaks[i]==ele){
                  c=11;
                }

                if(f_peaks[i]==eig){///c=4個面の音の検出
                  c=8;
                }

             } 
           Serial.print(" "); 
           Serial.print(a+b*c);
           Serial.print(" ");               

            if (b*c+a==73 || b*c+a==67 || b*c+a==43 || b*c+a==47){//chord detect
              chord_out ="m7";           
            }

            if (b*c+a==81 || b*c+a==59 || b*c+a==49 || b*c+a==41){
              chord_out ="M7";           
            }

            if (b*c+a==74 || b*c+a==51 || b*c+a==48 || b*c+a==57){
              chord_out ="7";           
            }

            if (b*c+a==63 || b*c+a==66 || b*c+a==58 || b*c+a==42){
              chord_out ="m7(b5)";           
            } 

            int root;

            if (b*c+a==73 || b*c+a==81 || b*c+a==74 || b*c+a==63){//基本形
              root = fir;       
            }

            if (b*c+a==67 || b*c+a==66){//1
              root = nin;       
            }
            
            if (b*c+a==59 || b*c+a==51){//1
              root = eig;       
            }

            if (b*c+a==43 || b*c+a==49 || b*c+a==48){//2
              root = fif;       
            }

            if (b*c+a==58){//2
              root = six;       
            }

            if (b*c+a==47 || b*c+a==57 || b*c+a==42){//3
              root = sec;       
            }
            
            if (b*c+a==41){//3
              root = fir;       
            }

          //   Serial.print(root);
          //  Serial.print("(root)");
          //  Serial.print(third);
          //  Serial.print("(third)");
          //  Serial.print(seventh);
          //  Serial.print("(seventh)");
             Serial.print(root);          
            Serial.print('\n');
                 
  return  root;
  }
}


int get_mode(const int* array){
    size_t size = 5;
    assert(array != NULL);
    assert(size > 0);
   
    int mode;              // これまでに調べた中での最頻値
    int count_max = 0;     // これまでに調べた登場回数の中で最大のもの
    
    for (size_t i = 0; i < size; ++i) {

        // array[i] の値の登場回数を調べる
        int count = 1;
        for (size_t j = i + 1; j < size; ++j) {
            if (array[i] == array[j]) {
                ++count;
            }
        }

        // これまでの最大の登場回数よりも多かったら、更新する
        if (count_max <= count) {
            count_max = count;
            mode = array[i];
        }
    }
    return mode;
}


// Here "chord" variable has value of detected chord,
// 0-11 defines all majot chord from C,C#,D,D#,.. B
//12-23 defines all minor chord from Cm,C#m,Dm,D#m,.. Bm

void Chord_output(int c){

   a2=micros();
      M5.Lcd.setCursor(10, 10);
    if (c == 0) {
      M5.Lcd.print('C');
      M5.Lcd.print(' ');
      M5.Lcd.println(chord_out);
      Serial.print('C');
      Serial.println(chord_out);
    }
    if (c == 1) {
      M5.Lcd.print('C');
      M5.Lcd.print('#');
      M5.Lcd.println(chord_out);
      Serial.print('C');
      Serial.print('#');
      Serial.println(chord_out);
    }
    if (c == 2) {
      M5.Lcd.print('D');
      M5.Lcd.print(' ');
      M5.Lcd.println(chord_out);
      Serial.print('D');
      Serial.println(chord_out);
    }
    if (c == 3) {
      M5.Lcd.print('D');
      M5.Lcd.print('#');
      M5.Lcd.println(chord_out);
      Serial.print('D');
      Serial.print('#');
      Serial.println(chord_out);
    }
    if (c == 4) {
      M5.Lcd.print('E');
      M5.Lcd.print(' ');
      M5.Lcd.println(chord_out);
      Serial.print('E');
      Serial.println(chord_out);
    }
    if (c == 5) {
      M5.Lcd.print('F');
      M5.Lcd.print(' ');
      M5.Lcd.println(chord_out);
      Serial.print('F');
      Serial.println(chord_out);
    }
    if (c == 6) {
      M5.Lcd.print('F');
      M5.Lcd.print('#');
      M5.Lcd.println(chord_out);
      Serial.print('F');
      Serial.print('#');
      Serial.println(chord_out);
    }
    if (c == 7) {
      M5.Lcd.print('G');
      M5.Lcd.print(' ');
      M5.Lcd.println(chord_out);
      Serial.print('G');
      Serial.println(chord_out);
    }
    if (c == 8) {
      M5.Lcd.print('G');
      M5.Lcd.print('#');
      M5.Lcd.println(chord_out);
      Serial.print('G');
      Serial.print('#');
      Serial.println(chord_out);
    }
    if (c == 9) {
      M5.Lcd.print('A');
      M5.Lcd.print(' ');
      M5.Lcd.println(chord_out);
      Serial.print('A');
      Serial.println(chord_out);
    }
    if (c == 10) {
      M5.Lcd.print('A');
      M5.Lcd.print('#');
      M5.Lcd.println(chord_out);
      Serial.print('A');
      Serial.print('#');
      Serial.println(chord_out);
    }
    if (c == 11) {
      M5.Lcd.print('B');
      
      M5.Lcd.println(chord_out);
      Serial.print('B');
      Serial.println(chord_out);
    }
     Serial.println("\n");
}


void setup() {
M5.begin();
M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setTextColor(BLACK, WHITE);
  M5.Lcd.setTextSize(2);
}


int array[5];
int i;

void loop() { 
 	int c;
 	int k = Chord_det();
  i++;
	i%=5;
  array[i] = k;
	c = get_mode(array);
	Chord_output(c);
}
