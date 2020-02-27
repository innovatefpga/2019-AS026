/*
iOwlT: Sound Geolocalization System main Coordenadas

links HPS to FPGA and applies the multilateration algorithm

*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "hps_0.h"

#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#include <algorithm>
#include <fftw3.h>
#include <fstream>
#include <Eigen/Dense>
#include <iostream>

#define v 343
#define Fs 16000
#define L 8000 

#define xm1 0
#define xm2 1.135
#define xm3 0.35073
#define xm4 0.35073
#define xm5 -0.91823
#define xm6 -0.91823

#define ym1 0
#define ym2 0
#define ym3 1.07945
#define ym4 -1.07945
#define ym5 -0.66714
#define ym6 0.66714

#define zm1 0
#define zm2 -0.555
#define zm3 -0.555
#define zm4 -0.555
#define zm5 -0.555
#define zm6 -0.555

//#include "multilateration.cpp"
#include "NeuralNetwork.h"
#include "MFCC.cc"

using namespace Eigen;
using namespace std;

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )


//Microphone addresses in FPGA-created Memory
#define MIC1_ADDR 0x2
#define MIC2_ADDR 0x1F42
#define MIC3_ADDR 0x3E82
#define MIC4_ADDR 0x5DC2
#define MIC5_ADDR 0x7D02
#define MIC6_ADDR 0x9C42

double Tau (int16_t x[L], int16_t y[L]){
  double a[2*L-1]={0},b[2*L-1]={0};
  double c[2*L-1];
  double t,mx=0,my=0;
  int i=0,max_index,sx=0,sy=0;
  int maxx=x[0],maxy=y[0];
  for(i=0;i<L;i++){
    sx+=x[i];
    if (x[i]>maxx){
        maxx=x[i];
    }
    if (y[i]>maxy){
        maxy=y[i];
    }
    sy+=y[i];
  }
  mx = (sx/L)*1.0;
  my = (sy/L)*1.0;
  for(i=0;i<L;i++){
    a[i]=(x[i]-mx)/(maxx-mx);
    b[i]=(y[L-i-1]-my)/(maxy-my);
  }
  
  
  fftw_complex *A,*B,*C; /* Output */
  fftw_plan p1,p2,p3; /* Plan */
  
  A = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*(L*2-1));
  B = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*(L*2-1));
  C = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*(L*2-1));
  p1 = fftw_plan_dft_r2c_1d(2*L-1, a, A, FFTW_ESTIMATE);
  fftw_execute(p1);
  p2 = fftw_plan_dft_r2c_1d(2*L-1, b, B, FFTW_ESTIMATE);
  fftw_execute(p2);
  
  for(i=0;i<2*L-1;i++){
    C[i][0] = (A[i][0]*B[i][0]-A[i][1]*B[i][1])/(2*L-1);
    C[i][1] = (A[i][0]*B[i][1]+A[i][1]*B[i][0])/(2*L-1);
  }
  p3 = fftw_plan_dft_c2r_1d(2*L-1,C, c, FFTW_BACKWARD);
  fftw_execute(p3);
  
  fftw_destroy_plan(p1);
  fftw_free(A);
  fftw_free(B);
  fftw_free(C);
  fftw_destroy_plan(p2);
  fftw_destroy_plan(p3);
  
  max_index = max_element(c,c+2*L-1) - c;
  t = 1.0*(max_index-L+1)/Fs;
  return t;
}

void get_cord(double t[5],double* coord){
  int i,j;
  double R1,Ri;
  double T[4];
  int max_corr;
  //Coordenadas dos microfones
  MatrixXd mic(5,3);
  max_corr = max_element(t,t+5)-t;
 mic(0,0) = xm1;
 mic(0,1) = ym1;
 mic(0,2) = zm1;
 if(max_corr==0){
    mic(1,0) = xm3;
    mic(1,1) = ym3;
    mic(1,2) = zm3;
    mic(2,0) = xm4;
    mic(2,1) = ym4;
    mic(2,2) = zm4;
    mic(3,0) = xm5;
    mic(3,1) = ym5;
    mic(3,2) = zm5;
    mic(4,0) = xm6;
    mic(4,1) = ym6;
    mic(4,2) = zm6;
    T[0] = t[1];
    T[1] = t[2];
    T[2] = t[3];
    T[3] = t[4];
cout<<"mic2 desprezado"<<endl;

  }
else if(max_corr==1){
    mic(1,0) = xm2;
    mic(1,1) = ym2;
    mic(1,2) = zm2;
    mic(2,0) = xm4;
    mic(2,1) = ym4;
    mic(2,2) = zm4;
    mic(3,0) = xm5;
    mic(3,1) = ym5;
    mic(3,2) = zm5;
    mic(4,0) = xm6;
    mic(4,1) = ym6;
    mic(4,2) = zm6;
    T[0] = t[0];
    T[1] = t[2];
    T[2] = t[3];
    T[3] = t[4];
cout<<"mic3 desprezado"<<endl;

  }
  else if (max_corr==2){
    mic(1,0) = xm2;
    mic(1,1) = ym2;
    mic(1,2) = zm2;
    mic(2,0) = xm3;
    mic(2,1) = ym3;
    mic(2,2) = zm3;
    mic(3,0) = xm5;
    mic(3,1) = ym5;
    mic(3,2) = zm5;
    mic(4,0) = xm6;
    mic(4,1) = ym6;
    mic(4,2) = zm6;
    T[0] = t[0];
    T[1] = t[1];
    T[2] = t[3];
    T[3] = t[4];
cout<<"mic4 desprezado"<<endl;

}
  else if (max_corr==3){
    mic(1,0) = xm2;
    mic(1,1) = ym2;
    mic(1,2) = zm2;
    mic(2,0) = xm3;
    mic(2,1) = ym3;
    mic(2,2) = zm3;
    mic(3,0) = xm4;
    mic(3,1) = ym4;
    mic(3,2) = zm4;
    mic(4,0) = xm6;
    mic(4,1) = ym6;
    mic(4,2) = zm6;
    T[0] = t[0];
    T[1] = t[1];
    T[2] = t[2];
    T[3] = t[4];
cout<<"mic5 desprezado"<<endl;

}
    else{
      mic(1,0) = xm2;
      mic(1,1) = ym2;
      mic(1,2) = zm2;
      mic(2,0) = xm3;
      mic(2,1) = ym3;
      mic(2,2) = zm3;
      mic(3,0) = xm4;
      mic(3,1) = ym4;
      mic(3,2) = zm4;
      mic(4,0) = xm5;
      mic(4,1) = ym5;
      mic(4,2) = zm5;
      T[0] = t[0];
      T[1] = t[1];
      T[2] = t[2];
      T[3] = t[3];
     cout<<"mic6 desprezado"<<endl;
}
  //Diferença dos tempos

  
               
  VectorXd Tau(4),Solution(3);
  for (i=0;i<4;i++)
    Tau(i)=T[i];
  
 cout << Tau<<endl; 
  //Ax+B = 0
  MatrixXd A(3,3);
  VectorXd B(3);
  for(i=2;i<5;++i){
    for(j=0;j<3;++j){
      A(i-2,j) = 2*mic(i,j)/(v*Tau(i-1))-2*mic(1,j)/(v*Tau(0));
    }
  }
  //x1^2+y1^2+z1^2
  R1 = pow(mic(1,0),2)+pow(mic(1,1),2)+pow(mic(1,2),2);
  for (i=2;i<5;++i){
    Ri = pow(mic(i,0),2)+pow(mic(i,1),2)+pow(mic(i,2),2);
    B(i-2) = -v*Tau(i-1)+v*Tau(0)+Ri/(v*Tau(i-1))- R1/(v*Tau(0));
  }
  
  //mais lento e mais preciso 
  /*   cout << "The least-squares solution is:\n"
        << A.bdcSvd(ComputeThinU | ComputeThinV).solve(b) << endl;
  */
  //Solution << A.bdcSvd(ComputeThinU | ComputeThinV).solve(B);  

  //mais rapido e menos preciso
  
  Solution << (A.transpose() * A).ldlt().solve(A.transpose()*B);

  cout << Solution << endl;

  VectorXd::Map(coord,Solution.rows()) = Solution;
}

void multilateration(int16_t mic1[L], int16_t mic2[L], int16_t mic3[L], int16_t mic4[L], 
int16_t mic5[L],int16_t mic6[L], double *coord){
  double t[5];
  int i;
  
  t[0] = Tau(mic2,mic1);
  t[1] = Tau(mic3,mic1);
  t[2] = Tau(mic4,mic1);
  t[3] = Tau(mic5,mic1);
  t[4] = Tau(mic6,mic1);
  
  
  get_cord(t,coord);
}


int main() {
 
    void *virtual_base;
    int fd;
    void *h2p_lw_button_addr;
    void *h2p_lw_ram_addr;
    void *h2p_lw_ram_base;

    
    //Defining pointer for register
    if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
        printf( "ERROR: could not open \"/dev/mem\"...\n" );
        return( 1 );
    }
    virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );
    if( virtual_base == MAP_FAILED ) {
        printf( "ERROR: mmap() failed...\n" );
        close( fd );
        return( 1 );
    }
    h2p_lw_ram_base=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ONCHIP_MEMORY2_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_button_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + BUTTON_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

    
    int i;
    double coord[3]; //Variable that will store x, y, z coords after multilateration
    char vetor[6]; 
    
    //Arrays that will receive microphone data
    int16_t mic1[L];
    int16_t mic2[L];
    int16_t mic3[L];
    int16_t mic4[L];
    int16_t mic5[L];
    int16_t mic6[L];
    
    int index;
    int buffer_pointer;//Store the position in memory that represent the begin of the microphone data
    int impulsive_sound=0;//1 if impulse sound, else 0
    
	vector<double> v0;
    
    int samplingRate=16000;
    int numCepstra=13;
    int winLength=25;
    int frameShift=10;
    int numFilters=26;
    int lowFreq=50;
    int highFreq=16000/2;
    
    bool result;
    
    MLP_NeuralNetwork nn = MLP_NeuralNetwork();

    MFCC mfccComputer (samplingRate, numCepstra, winLength, frameShift, numFilters, lowFreq,highFreq,mic1);
    
    h2p_lw_ram_addr = h2p_lw_ram_base;

    *(int16_t *)(h2p_lw_ram_addr+2*0xBB82)=0x0;

    while(1)
    {
        //Read position that indicates if some impulsive sound has occurred
        impulsive_sound = *(uint16_t *)h2p_lw_ram_addr;
	
	//cout << *(int16_t *)(h2p_lw_ram_addr+2*(MIC3_ADDR)) <<endl;        

        if(impulsive_sound){
            printf("Impulsive sound detection.\n");
            buffer_pointer = *(int16_t *)(h2p_lw_ram_addr + 2);
            printf("%d\n", buffer_pointer);
            

            for(i=0;i<L;++i){
                index = ( *(short *)(h2p_lw_ram_addr + 2) +i)%L; 
                //printf("%d\n", index); 

                mic1[i]=*(int16_t *)(h2p_lw_ram_addr+2*(MIC1_ADDR+index));
                mic2[i]=*(int16_t *)(h2p_lw_ram_addr+2*(MIC2_ADDR+index));
                mic3[i]=*(int16_t *)(h2p_lw_ram_addr+2*(MIC3_ADDR+index));
                mic4[i]=*(int16_t *)(h2p_lw_ram_addr+2*(MIC4_ADDR+index));
                mic5[i]=*(int16_t *)(h2p_lw_ram_addr+2*(MIC5_ADDR+index));
                mic6[i]=*(int16_t *)(h2p_lw_ram_addr+2*(MIC6_ADDR+index));
            }
          	
		    //cout << mic1[0] << " "<<mic1[100]<<endl;
    
            v0 = mfccComputer.get_mfcc_coeffs();
    
            //cout<<v0[0]<<" "<<v0[671]<<" "<<v0.size()<<endl;
            result=nn.classify(v0);
            cout<<"Resultado da classificao: "<<(int)result<<endl;

            //send the result of Neural Network to FPGA
            if(result==0)
                //Negative Result to desired sound
                *(uint16_t *)(h2p_lw_ram_addr+2*0xBB82)=0x1;
            else{
                //Desired sound recognized

                multilateration(mic1,mic2,mic3,mic4,mic5,mic6,coord);//calling multilateration algorithm

                printf("x:%f,y:%f,z:%f\n",coord[0],coord[1],coord[2]);

                sprintf(vetor, "%06f", coord[0]);
                *(char *)(h2p_lw_ram_addr+2*0xBB83+1)=vetor[0];
                *(char *)(h2p_lw_ram_addr+2*0xBB83)=vetor[1];
                *(char *)(h2p_lw_ram_addr+2*0xBB84+1)=vetor[2];
                *(char *)(h2p_lw_ram_addr+2*0xBB84)=vetor[3]; 
                *(char *)(h2p_lw_ram_addr+2*0xBB85+1)=vetor[4];
                *(char *)(h2p_lw_ram_addr+2*0xBB85)=vetor[5];

                sprintf(vetor, "%06f", coord[1]);
                *(char *)(h2p_lw_ram_addr+2*0xBB86+1)=vetor[0];
                *(char *)(h2p_lw_ram_addr+2*0xBB86)=vetor[1]; 
                *(char *)(h2p_lw_ram_addr+2*0xBB87+1)=vetor[2];
                *(char *)(h2p_lw_ram_addr+2*0xBB87)=vetor[3];
                *(char *)(h2p_lw_ram_addr+2*0xBB88+1)=vetor[4];
                *(char *)(h2p_lw_ram_addr+2*0xBB88)=vetor[5];

                sprintf(vetor, "%06f", coord[2]);
                *(char *)(h2p_lw_ram_addr+2*0xBB89+1)=vetor[0];
                *(char *)(h2p_lw_ram_addr+2*0xBB89)=vetor[1]; 
                *(char *)(h2p_lw_ram_addr+2*0xBB8A+1)=vetor[2];
                *(char *)(h2p_lw_ram_addr+2*0xBB8A)=vetor[3];
                *(char *)(h2p_lw_ram_addr+2*0xBB8B+1)=vetor[4];
                *(char *)(h2p_lw_ram_addr+2*0xBB8B)=vetor[5];

                *(uint16_t *)(h2p_lw_ram_addr+2*0xBB82)=0x2;
            }
            
            *(int16_t *)(h2p_lw_ram_addr+2*0xBB8C)=800; //send new threshold to FPGA
            *(uint16_t *)(h2p_lw_ram_addr)=0x0; //finish arm process
        }
        else if(*(uint32_t*)h2p_lw_button_addr != 3){
            //ends program execution if any button is pressed
            break;
        }
    }

    if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
        printf( "ERROR: munmap() failed...\n" );
        close( fd );
        return( 1 );
    }
 
    close( fd );

    return 0;
}

