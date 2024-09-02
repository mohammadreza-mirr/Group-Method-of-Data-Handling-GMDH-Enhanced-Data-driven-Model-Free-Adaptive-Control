clc;
close all;
clear all;
%%     3 tank parameters
Sa=0.0154;
Sn=5*10^-5;
az1=0.22;
az2=0.27;
az3=0.28;
az10=0.40;
%az10=0.32;
Qmax=1*10^-4;
Hmaxh1=0.6505;
Hmaxh2=0.6505;
steps=24999;
%%  controller initialization
KP11 = 0.00512; KI11 = 0.00063; KD11 = 0.00721;
KP12 = 0.00131; KI12 = 0.00005; KD12 = 0.00130;
KP21 = 0.00267; KI21 = 0.00027; KD21 = 0.00242;
KP22 = 0.00985; KI22 = 0.00101; KD22 = 0.00642;
e1(:,1:3)=0.05;
e2(:,1:3)=0.1;
%%   System initialization
h1(:,1:4)=0;
h2(:,1:4)=0;
h3(:,1:4)=0;
u(:,1)=[0;0];
u(:,2)=[0;0];
u(:,3)=[0;0];
yf1(:,4)=0;
yf1(:,3)=0;
yf1(:,2)=0;
yf1(:,1)=0;
yf2(:,4)=0;
yf2(:,3)=0;
yf2(:,2)=0;
yf2(:,1)=0;
uf1(:,4)=0;
uf1(:,3)=0;
uf1(:,2)=0;
uf1(:,1)=0;
uf2(:,4)=0;
uf2(:,3)=0;
uf2(:,2)=0;
uf2(:,1)=0;
Z1(:,4)=0;
Z2(:,4)=0;

for k=4:steps
    %%  Reference trajectory
    if k<5000
        Ref1(:,k)=0.4;
    end
    if 5000<=k
        Ref1(:,k)=0.65;
    end
    
     if 25000<=k
        Ref1(:,k)=0.5;
     end
     
      if 30000<=k
        Ref1(:,k)=0.4;
     end
    
    %%  R2
    if k<10000
        Ref2(:,k)=0.3;
    end
    if 10000<=k
        Ref2(:,k)=0.6;
    end
    if 15000<=k
        Ref2(:,k)=0.3;
    end
    
     if 35000<=k
        Ref2(:,k)=0.5;
    end
    
    R(:,k+1)=[Ref1(:,k);Ref2(:,k)];
    
    %%   error signal
    e1(:,k)=Z1(:,k)-Ref1(:,k);
    e2(:,k)=Z2(:,k)-Ref2(:,k);
    
    %%    PID Controller
    
    u(:,k)=u(:,k-1)+[KP11,KI11,KD11,KP12,KI12,KD12;
        KP21,KI21,KD21,KP22,KI22,KD22]*[e1(:,k)-e1(:,k-1);
        e1(:,k);
        e1(:,k)-2*e1(:,k-1)-e1(:,k-2);
        e2(:,k)-e2(:,k-1);
        (e2(:,k));
        (e2(:,k)-2*e2(:,k-1))-e2(:,k-2)];
    %% low pass input filter
    uf1(:,k)=1.672*uf1(:,k-1)-0.719*uf1(:,k-2)+0.02483*u(1,k)-0.0222*u(1,k-1);
    uf2(:,k)=1.672*uf2(:,k-1)-0.719*uf2(:,k-2)+0.02483*u(2,k)-0.0222*u(2,k-1);
    u(1,k)=uf1(:,k);
    u(2,k)=uf2(:,k);
    %%
    if u(1,k)>Qmax
        u(1,k)=Qmax;
    end
    if u(2,k)>Qmax
        u(2,k)=Qmax;
    end
    
    if u(1,k)<0
        u(1,k)=0;
    end
    if u(2,k)<0
        u(2,k)=0;
    end
    
    h1(:,k+1)=h1(:,k)+(1/Sa)*(u(1,k)-(az1*Sn*sign(h1(:,k)-h3(:,k))*sqrt(2*9.8*abs(h1(:,k)-h3(:,k)))))-az10*Sn*sign(h1(:,k))*(sqrt(2*9.8*abs(h1(:,k))));
    h2(:,k+1)=h2(:,k)+(1/Sa)*(u(2,k)+(az3*Sn*sign(h3(:,k)-h2(:,k))*sqrt(2*9.8*abs(h3(:,k)-h2(:,k))))-az2*Sn*sign(h2(:,k))*(sqrt(2*9.8*abs(h2(:,k)))));
    h3(:,k+1)=h3(:,k)+(1/Sa)*(az1*Sn*sign(h1(:,k)-h3(:,k))*sqrt(2*9.8*abs(h1(:,k)-h3(:,k)))-(az3*Sn*sign(h3(:,k)-h2(:,k))*sqrt(2*9.8*abs(h3(:,k)-h2(:,k)))));
    
    if h1(:,k+1)<0
        h1(:,k+1)=0;
    end
    if h2(:,k+1)<0
        h2(:,k+1)=0;
    end
    
    
    %%     low pass output  filter
    yf1(:,k+1)=1.672*yf1(:,k)-0.719*yf1(:,k-1)+0.02483*h1(:,k+1)+0.0222*h1(:,k);
    yf2(:,k+1)=1.672*yf2(:,k)-0.719*yf2(:,k-1)+0.02483*h2(:,k+1)+0.0222*h2(:,k);
    h1(:,k+1)=yf1(:,k+1);
    h2(:,k+1)=yf2(:,k+1);
    %% Data Quantization
    Z1(:,k+1)=h1(:,k+1);
    Z2(:,k+1)=h2(:,k+1);                                                   % Sensor data for feedback
    % Quantization  h1
    Z1(:,k+1)=h1(:,k+1);
    if h1(:,k+1)>=Hmaxh1
    tetaqantizer = 0.99;
    basicZ0 =0.7;
    resolution = 6;
    ysend = h1(:,k+1);
    [Z1(:,k+1)] = quantizer(ysend,tetaqantizer, basicZ0,resolution); 
  
    end
    % Quantization h1
    Z2(:,k+1)=h2(:,k+1);
    if h2(:,k+1)>=Hmaxh2
    tetaqantizer = 0.99;
    basicZ0 =150;
    resolution = 46;
    ysend = h2(:,k+1);
    [Z2(:,k+1)] = quantizer(ysend,tetaqantizer, basicZ0,resolution);    
    end
    
    inputsaturationlevel(:,k)=Qmax;
  
end
%% Plot Results for PID
figure (1)
plot(h1,'blue','LineWidth',1.5);

grid on
figure (2)
plot(h2,'blue','LineWidth',1.5);
grid on
figure (3)
plot(h3,'blue','LineWidth',1.5);
grid on


figure (4)
plot(u(1,:),'blue','LineWidth',0.5);
grid on
hold on
plot(inputsaturationlevel(1,:),'--Red','LineWidth',1.5);

figure (5)
plot(u(2,:),'blue','LineWidth',0.5);
grid on
hold on
plot(inputsaturationlevel(1,:),'--Red','LineWidth',1.5);
clear all;



%%            GMDH-MFAC for 3 tanks with
%     3 tank parameters
Sa=0.0154;
Sn=5*10^-5;
az1=0.22;
az2=0.27;
az3=0.28;
az10=0.40;
%az10=0.32;
Qmax=1*10^-4;
Hmaxh1=0.6505;
Hmaxh2=0.6505;
steps=24999;


F11=10;
F22=10;
F(:,:,1)=[F11,0;0,F22];
F(:,:,2)=[F11,0;0,F22];
F(:,:,3)=[F11,0.1;0.1,F22];

deltZ(:,4)=[0;0];
deltau(:,3)=[0;0];
Z(:,4)=[0;0];
eta=1;           
ro=0.5;           
mu=0.0001;            
landa=1;         
%%   System initialization
h1(:,1:4)=0;
h2(:,1:4)=0;
h3(:,1:4)=0;
u(:,1)=[0;0];
u(:,2)=[0;0];
u(:,3)=[0;0];
%% filter parameters
yf11(:,4)=0;
yf11(:,3)=0;
yf11(:,2)=0;
yf11(:,1)=0;
yf22(:,4)=0;
yf22(:,3)=0;
yf22(:,2)=0;
yf22(:,1)=0;
uf1(:,4)=0;
uf1(:,3)=0;
uf1(:,2)=0;
uf1(:,1)=0;
uf2(:,4)=0;
uf2(:,3)=0;
uf2(:,2)=0;
uf2(:,1)=0;
Z1(:,4)=0;
Z2(:,4)=0;
swch=0;
ZGMDH(1,1)=0;
deltZGMDH(1,1)=0;
GMDHt=0;
for k=4:steps
 %% R1   
      if k<5000
        Ref1(:,k)=0.4;
    end
    if 5000<=k
        Ref1(:,k)=0.65;
    end
    
     if 25000<=k
        Ref1(:,k)=0.5;
     end
     
    
    %%  R2
    if k<10000
        Ref2(:,k)=0.3;
    end
    if 10000<=k
        Ref2(:,k)=0.6;
    end
    if 15000<=k
        Ref2(:,k)=0.3;
    end
    
    R(:,k+1)=[Ref1(:,k);Ref2(:,k)];
    %%   MFAC controller
    
    F(:,:,k)=F(:,:,k-1)+ (eta*(deltZ(:,k)-F(:,:,k-1)'*deltau(:,k-1))*deltau(:,k-1)')/(mu+norm(deltau(:,k-1))^2);
    if (F(2,2,k)<=0 | F(1,1,k)<=0 | F(2,1,k)<=0 | F(1,2,k)<=0)
        F(:,:,k)=F(:,:,3);
    end
       u(:,k)=u(:,k-1)+(ro*F(:,:,k)'*(R(:,k+1)-Z(:,k)))/(landa+norm(F(:,:,k))^2);
      
      
    %% Low pass input filter
    uf1(:,k)=1.672*uf1(:,k-1)-0.719*uf1(:,k-2)+0.02483*u(1,k)-0.0222*u(1,k-1);
    uf2(:,k)=1.672*uf2(:,k-1)-0.719*uf2(:,k-2)+0.02483*u(2,k)-0.0222*u(2,k-1);
    u(1,k)=uf1(:,k);
    u(2,k)=uf2(:,k);
    
    if u(1,k)>Qmax
        u(1,k)=Qmax;
    end
    if u(2,k)>Qmax
        u(2,k)=Qmax;
    end
    
    if u(1,k)<=0
        u(1,k)=0;
    end
    if u(2,k)<=0
        u(2,k)=0;
    end
    
    h1(:,k+1)=h1(:,k)+(1/Sa)*(u(1,k)-(az1*Sn*sign(h1(:,k)-h3(:,k))*sqrt(2*9.8*abs(h1(:,k)-h3(:,k)))))-az10*Sn*sign(h1(:,k))*(sqrt(2*9.8*abs(h1(:,k))));
    h2(:,k+1)=h2(:,k)+(1/Sa)*(u(2,k)+(az3*Sn*sign(h3(:,k)-h2(:,k))*sqrt(2*9.8*abs(h3(:,k)-h2(:,k))))-az2*Sn*sign(h2(:,k))*(sqrt(2*9.8*abs(h2(:,k)))));
    h3(:,k+1)=h3(:,k)+(1/Sa)*(az1*Sn*sign(h1(:,k)-h3(:,k))*sqrt(2*9.8*abs(h1(:,k)-h3(:,k)))-(az3*Sn*sign(h3(:,k)-h2(:,k))*sqrt(2*9.8*abs(h3(:,k)-h2(:,k)))));
       
    if h1(:,k+1)<0
        h1(:,k+1)=0;
    end
    if h2(:,k+1)<0
        h2(:,k+1)=0;
    end
      if h3(:,k+1)<0
        h3(:,k+1)=0;
    end
    %% filter
    yf11(:,k+1)=1.672*yf11(:,k)-0.719*yf11(:,k-1)+0.02483*h1(:,k+1)+0.0222*h1(:,k);
    yf22(:,k+1)=1.672*yf22(:,k)-0.719*yf22(:,k-1)+0.02483*h2(:,k+1)+0.0222*h2(:,k);
    h1(:,k+1)=yf11(:,k+1);
    h2(:,k+1)=yf22(:,k+1);
    
    %%
    Z(1,k+1)=h1(:,k+1);
    Z(2,k+1)=h2(:,k+1);
    
    if   (Z(1,k+1)-Z(1,k))==0 && abs(Z(1,k+1)- Ref1(:,k))~=0  && k>9000  && k<16000
       
       
%% GMDH NN         
      if GMDHt==0
        kGMDH=k-1;
        GMDHt=1;
      end
    Inputs=[u(1,1:kGMDH-2),u(1,k-1:k);h3(1,1:kGMDH-2),h3(1,k-1:k);Z(1,1:kGMDH-2),Z(1,k-1:k)];
   
    Targets=[Z(1,2:kGMDH-1),Z(1,k:k+1)];
        

       nData = size(Inputs,2);
        % Train Data
        nTrainData = nData-2;
        TrainInd = 1:nTrainData;
        TrainInputs = Inputs(:,TrainInd);
        TrainTargets = Targets(:,TrainInd);
        
        % Test Data
        nTestData = nData - nTrainData;
        TestInd = nTrainData+1:nData;
        TestInputs = Inputs(:,TestInd);
        TestTargets = Targets(:,TestInd);
        
        
        %% Create and Train GMDH Network
        
        params.MaxLayerNeurons = 15;                                        % Maximum Number of Neurons in a Layer 12
        params.MaxLayers =5;                                                % Maximum Number of Layers 5
        params.alpha = 0.9;                                                 % Selection Pressure 0.9
        params.pTrain = 0.7;                                                % Train Ratio
        [gmdh] = GMDH(params, TrainInputs, TrainTargets);
        
        
        %% Evaluate GMDH Network
        Outputs = ApplyGMDH(gmdh, Inputs);
        TrainOutputs = Outputs(:,TrainInd);
        TestOutputs = Outputs(:,TestInd);
        TestOutputs;
        Z(1,k+1)=TestOutputs(1,2);   
    end
    deltau(:,k)=u(:,k)-u(:,k-1);
    deltZ(:,k+1)=Z(:,k+1)-Z(:,k);
    %% parameter saving to plot
    f11(k,:)=F(1,1,k);
    f21(k,:)=F(2,1,k);
    f12(k,:)=F(1,2,k);
    f22(k,:)=F(2,2,k);
end

%% Plot Results for GMDH-MFAC
figure (1)
hold on
plot(h1,'black','LineWidth',2);
grid on

figure (2)
hold on
plot(h2,'black','LineWidth',2);
grid on

figure (3)
hold on
plot(h3,'black','LineWidth',2);
grid on


figure (4)
hold on
plot(u(1,:),'black','LineWidth',2);
grid on

figure (5)
hold on
plot(u(2,:),'black','LineWidth',2);
grid on

figure(10);
PlotResults(TrainTargets, TrainOutputs, 'Train Data');
figure(20);
PlotResults(TestTargets, TestOutputs, 'Test Data');
figure(30);
PlotResults(Targets, Outputs, 'All Data');
figure (40);
plotregression(TrainTargets, TrainOutputs, 'Train Data', ...
    TestTargets, TestOutputs, 'TestData', ...
    Targets, Outputs, 'All Data');

figure (50)
subplot 221
plot(f11,'black','LineWidth',2);
grid on
subplot 222
plot(f12,'black','LineWidth',2);
grid on
subplot 223
plot(f21,'black','LineWidth',2);
grid on
subplot 224
plot(f22,'black','LineWidth',2);
grid on
clear all

%
%%           MFAC controller 3 tanks
F11=10;
F22=10;
F(:,:,1)=[F11,0;0,F22];
F(:,:,2)=[F11,0;0,F22];
F(:,:,3)=[F11,0.1;0.1,F22];

deltZ(:,4)=[0;0];
deltau(:,3)=[0;0];
Z(:,4)=[0;0];
eta=1;           
ro=0.5;            
mu=0.0001;            
landa=1;        
%%     3 tank parameters
Sa=0.0154;
Sn=5*10^-5;
az1=0.22;
az2=0.27;
az3=0.28;
az10=0.40;
%az10=0.32;
Qmax=1*10^-4;
Hmaxh1=0.6505;
Hmaxh2=0.6505;
steps=24999;
%%   System initialization
h1(:,1:4)=0;
h2(:,1:4)=0;
h3(:,1:4)=0;
u(:,1)=[0;0];
u(:,2)=[0;0];
u(:,3)=[0;0];
% filter parameters
yf11(:,4)=0;
yf11(:,3)=0;
yf11(:,2)=0;
yf11(:,1)=0;
yf22(:,4)=0;
yf22(:,3)=0;
yf22(:,2)=0;
yf22(:,1)=0;
uf1(:,4)=0;
uf1(:,3)=0;
uf1(:,2)=0;
uf1(:,1)=0;
uf2(:,4)=0;
uf2(:,3)=0;
uf2(:,2)=0;
uf2(:,1)=0;
Z1(:,4)=0;
Z2(:,4)=0;
for k=4:steps
%% R1
     if k<5000
        Ref1(:,k)=0.4;
    end
    if 5000<=k
        Ref1(:,k)=0.65;
    end
    
     if 25000<=k
        Ref1(:,k)=0.5;
     end
%%  R2
    if k<10000
        Ref2(:,k)=0.3;
    end
    if 10000<=k
        Ref2(:,k)=0.6;
    end
    if 15000<=k
        Ref2(:,k)=0.3;
    end
    
    R(:,k+1)=[Ref1(:,k);Ref2(:,k)];
%%   MFAC controller
    F(:,:,k)=F(:,:,k-1)+ (eta*(deltZ(:,k)-F(:,:,k-1)'*deltau(:,k-1))*deltau(:,k-1)')/(mu+norm(deltau(:,k-1))^2);
    if (F(2,2,k)<=0 | F(1,1,k)<=0 | F(2,1,k)<=0 | F(1,2,k)<=0)
        F(:,:,k)=F(:,:,3);
    end
    u(:,k)=u(:,k-1)+(ro*F(:,:,k)'*(R(:,k+1)-Z(:,k)))/(landa+norm(F(:,:,k))^2);
    
    %% Low pass input filter
    uf1(:,k)=1.672*uf1(:,k-1)-0.719*uf1(:,k-2)+0.02483*u(1,k)-0.0222*u(1,k-1);
    uf2(:,k)=1.672*uf2(:,k-1)-0.719*uf2(:,k-2)+0.02483*u(2,k)-0.0222*u(2,k-1);
    u(1,k)=uf1(:,k);
    u(2,k)=uf2(:,k);
    %%
    if u(1,k)>Qmax
        u(1,k)=Qmax;
    end
    if u(2,k)>Qmax
        u(2,k)=Qmax;
    end
    
    if u(1,k)<=0
        u(1,k)=0;
    end
    if u(2,k)<=0
        u(2,k)=0;
    end
    
    
    h1(:,k+1)=h1(:,k)+(1/Sa)*(u(1,k)-(az1*Sn*sign(h1(:,k)-h3(:,k))*sqrt(2*9.8*abs(h1(:,k)-h3(:,k)))))-az10*Sn*sign(h1(:,k))*(sqrt(2*9.8*abs(h1(:,k))));
    h2(:,k+1)=h2(:,k)+(1/Sa)*(u(2,k)+(az3*Sn*sign(h3(:,k)-h2(:,k))*sqrt(2*9.8*abs(h3(:,k)-h2(:,k))))-az2*Sn*sign(h2(:,k))*(sqrt(2*9.8*abs(h2(:,k)))));
    h3(:,k+1)=h3(:,k)+(1/Sa)*(az1*Sn*sign(h1(:,k)-h3(:,k))*sqrt(2*9.8*abs(h1(:,k)-h3(:,k)))-(az3*Sn*sign(h3(:,k)-h2(:,k))*sqrt(2*9.8*abs(h3(:,k)-h2(:,k)))));
    
    
    if h1(:,k+1)<0
        h1(:,k+1)=0;
    end
    if h2(:,k+1)<0
        h2(:,k+1)=0;
    end
     if h3(:,k+1)<0
        h4(:,k+1)=0;
    end
    %% filter
    yf11(:,k+1)=1.672*yf11(:,k)-0.719*yf11(:,k-1)+0.02483*h1(:,k+1)+0.0222*h1(:,k);
    yf22(:,k+1)=1.672*yf22(:,k)-0.719*yf22(:,k-1)+0.02483*h2(:,k+1)+0.0222*h2(:,k);
    h1(:,k+1)=yf11(:,k+1);
    h2(:,k+1)=yf22(:,k+1);
    
    %% Data Quantization
    Z(1,k+1)=h1(:,k+1);
    if h1(:,k+1)>=Hmaxh1  
     % Quantization  h1
    Z(:,k+1)=[h1(:,k+1);h2(:,k+1)];
    tetaqantizer = 0.99;
    basicZ0 =0.7;
    resolution = 6;
    ysend = h1(:,k+1);
    [Z(1,k+1)] = quantizer(ysend,tetaqantizer, basicZ0,resolution); 
    
    end
    
    % Quantization h2
    Z(2,k+1)=h2(:,k+1);
    if h2(:,k+1)>=Hmaxh2
    tetaqantizer = 0.99;
    basicZ0 =150;
    resolution = 46;
    ysend = h2(:,k+1);
    [Z(2,k+1)] = quantizer(ysend,tetaqantizer, basicZ0,resolution); 

    end
      %% sensor data for feedback
    deltau(:,k)=u(:,k)-u(:,k-1);
    deltZ(:,k+1)=Z(:,k+1)-Z(:,k);
    %% parameter saving to plot
    f11(k,:)=F(1,1,k);
    f21(k,:)=F(2,1,k);
    f12(k,:)=F(1,2,k);
    f22(k,:)=F(2,2,k);
end
%% Plot Results for QMFAC
figure (1)
hold on
plot(h1,'green','LineWidth',1.5);
hold on
plot(Ref1,':r','LineWidth',1);
grid on
title 'LEVEL Tank 1 '
legend 'PID' 'GMDH-MFAC' 'QMFAC' 'Reference 1'
ylabel 'Liquid level of T 1 (m)'
figure (2)
hold on
plot(h2,'green','LineWidth',1.5);
hold on
plot(Ref2,':r','LineWidth',1);
legend 'PID'  'GMDH-MFAC' 'QMFAC' 'Refrence 2'
grid on
title 'LEVEL Tank 2 '
ylabel 'Liquid level of T 2 (m)'
figure (3)
hold on
plot(h3,'green','LineWidth',1.5);
grid on
title 'LEVEL Tank 3 '
legend 'PID' 'GMDH-MFAC' 'QMFAC'
ylabel 'Liquid level of T3 (m)'
figure (4)
hold on
plot(u(1,:),'green','LineWidth',0.5);
grid on
title 'Control Input Pump 1 '
legend 'PID' 'Input Saturation Level 1' 'GMDH-MFAC' 'QMFAC'
ylabel 'Control input Q1(k) (m3/s)'
figure (5)
hold on
plot(u(2,:),'green','LineWidth',0.5);
ylabel 'Control input Q2(k) (m3/s)'
grid on
title 'Control Input Pump 2 '
legend 'PID' 'Input Saturation Level 2' 'GMDH-MFAC' 'QMFAC'
resolution = 6;
tetaqantizer = 0.99;
for i= 1:resolution
   
    zi(i)=  0.4*(tetaqantizer^i);
    quanlevel=zi(i)*ones(1,steps);
    figure (78)
    hold on
    grid on
    plot(quanlevel,'--blue','LineWidth',1) 
  
    zi(i)=  0.7*(tetaqantizer^i);
    quanlevel=zi(i)*ones(1,steps);
    figure (78)
    hold on
    grid on
    plot(quanlevel,'--blue','LineWidth',1) 
     title 'Quantization Levels for tank 1'
end
for i= 1:resolution
   
    zi(i)=  0.3*(tetaqantizer^i);
    quanlevel=zi(i)*ones(1,steps);
    figure (79)
    hold on
    grid on
    plot(quanlevel,'--m','LineWidth',1) 
  
    zi(i)=  0.6*(tetaqantizer^i);
    quanlevel=zi(i)*ones(1,steps);
    figure (79)
    hold on
    grid on
    plot(quanlevel,'--m','LineWidth',1) 
    title 'Quantization Levels for tank 2'
end


