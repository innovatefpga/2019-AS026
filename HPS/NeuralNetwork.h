/*
iOwlT: Sound Geolocalization System Neural Network

Neural Network for recognizing gunshots
*/

#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class MLP_NeuralNetwork {
public:
    bool classify(vector<double> in);
    MLP_NeuralNetwork();  // constructor
    
private:
    MatrixXd w0 = MatrixXd(672,200);
    VectorXd b0 = VectorXd(200);
    MatrixXd w1 = MatrixXd(200,10);
    VectorXd b1 = VectorXd(10);
    MatrixXd w2 = MatrixXd(10,1);
    VectorXd b2 = VectorXd(1);
    
    double sigmoid(double x);
};

double MLP_NeuralNetwork::sigmoid(double x){
//sigmoid function
return 1/(1+exp(-x));
}

MLP_NeuralNetwork::MLP_NeuralNetwork(){
//Read All the Neural Network parameters from files
double w;
int aux;
vector<double> v0;

//MatrixXd x(672,1);
ifstream data;
data.open("w0.txt");

if (!data) {
    cerr << "Unable to open file w0.txt";
    exit(1);   // call system to stop
}


while (data >> w) {
    v0.push_back(w);
}

aux=0;
for (int i=0; i<672; ++i){
    for(int j=0; j<200; ++j){
        w0(i,j)=v0[aux];
        ++aux;
    }
}
v0.clear();

data.close();
data.open("b0.txt");

if (!data) {
    cerr << "Unable to open file b0.txt";
    exit(1);   // call system to stop
}

while (data >> w) {
    v0.push_back(w);
}

b0 = VectorXd::Map(v0.data(), v0.size());
v0.clear();

data.close();
data.open("w1.txt");

if (!data) {
    cerr << "Unable to open file w1.txt";
    exit(1);   // call system to stop
}

while (data >> w) {
    v0.push_back(w);
}

aux=0;
for (int i=0; i<200; ++i){
    for(int j=0; j<10; ++j){
        w1(i,j)=v0[aux];
        ++aux;
    }
}
v0.clear();

data.close();
data.open("b1.txt");

if (!data) {
    cerr << "Unable to open file b1.txt";
    exit(1);   // call system to stop
}

while (data >> w) {
    v0.push_back(w);
}

b1 = VectorXd::Map(v0.data(), v0.size());
v0.clear();
    
data.close();
data.open("w2.txt");

if (!data) {
    cerr << "Unable to open file w2.txt";
    exit(1);   // call system to stop
}

while (data >> w) {
    v0.push_back(w);
}

aux=0;
for (int i=0; i<10; ++i){
    for(int j=0; j<1; ++j){
        w2(i,j)=v0[aux];
        ++aux;
    }
}
v0.clear();

data.close();
data.open("b2.txt");

if (!data) {
    cerr << "Unable to open file b2.txt";
    exit(1);   // call system to stop
}

while (data >> w) {
    v0.push_back(w);
}

b2(0)=v0[0];
v0.clear();

data.close();
}

bool MLP_NeuralNetwork::classify(vector<double> in){
double result;
VectorXd z0(200);
MatrixXd z1(10,1);
MatrixXd z2(1,1);

VectorXd x = VectorXd::Map(in.data(), in.size());

z0=w0.transpose()*x+b0;
z1=w1.transpose()*z0+b1;
z2=w2.transpose()*z1+b2;
result = sigmoid(z2(0));

return (result>=0.5)?1:0;
}

#endif
