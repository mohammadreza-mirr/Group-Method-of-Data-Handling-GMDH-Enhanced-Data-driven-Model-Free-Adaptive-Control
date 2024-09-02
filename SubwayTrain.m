clc;
clear;
close all;

%% Quantized MFAC
%parameters
landa1=0.002;
landa2=0.001;
ro2=0.003;
ro1=0.968;
f(:,1)=0.137;
deltau(:,1)=1.342;
deltazz(:,2)=0.8;
v(:,2)=0.8;
zz(:,2)=0.8;
u(:,1)=1.342;
errorm1(:,2)=0;
zi (:,1)=0;
for k=2:4000
    % Reference Calculation 
 if k>=2 && k<=1000
        r=70; 
    end
    if k>1000 && k<=2000
        r=73;
    end
    if k>2000 && k<=2500
        r=65;
    end
    if k>2500 && k<=2900
        r=69;
    end
    if k>2900 
        r=60; 
    end

    Ref(:,k)=r;
    %% Path Slope calculation
     theta(:,k)=randn*0.0001;
  if k==400
        theta(:,k) = 4;
    end
    if k>400 &&  k<1000  
       theta(:,k) = 4+rand*0.01;   
    end
    if k == 2400 
         theta(:,k) = -4;
    end
    if k>2400 &&  k<3000  
        theta(:,k)= -4-rand*0.01; 
    end
    if k>3500 
       theta(:,k) = 0;  
    end
 
    %% controller
    f(:,k)=f(:,k-1)+((ro2*deltau(:,k-1))/(landa2+deltau(:,k-1)^2))*(deltazz(:,k)-f(:,k-1)*deltau(:,k-1));
    if  f(:,k)<=0.0001   %% reset
        f(:,k)= f(:,1);
    end
    u(:,k)=u(:,k-1)+((ro1*f(:,k))/(landa1+abs(f(:,k)^2)))*(r-zz(:,k));
    %% Subway Train Model
    M=500;
    G=9.8;
    PI=3.14;
    wr(:,k)=M*G*sin(theta(:,k));
    wc(:,k)=3.5*M*G*PI*10^-5;
    wt(:,k)=1.3*M*G*10^-4;
    L=2391;
    yymax=78;
    umax=0.46;
    wbar=1;
    Mt=M;
    fa(:,k)=wr(:,k)+wc(:,k)+wt(:,k);
    fb(:,k)=4500+150*v(:,k)+v(:,k)^2;
     if r==0 && k>3505
         fb(:,k)=0+150*v(:,k)+v(:,k)^2;
    end
    v(:,k+1)=wbar*v(:,k)+(wbar/Mt)*(u(:,k)-fa(:,k)-fb(:,k));  
    if v(:,k+1)<0
    v(:,k+1)=0;
    end
%%  Quantization
    
    tetaqantizer = 0.99;
    basicZ0 =83;
    resolution = 46;
    ysend = v(:,k+1);
    [zz(:,k+1)] = quantizer(ysend,tetaqantizer, basicZ0,resolution); 
%% Event Triggeres Scenario
    if (Ref(:,k)~= Ref(:,k-1))
        zz(:,k+1)=v(:,k+1);
    end
%% Error Calculation  
    deltau(:,k)=u(:,k)-u(:,k-1);
    deltav(:,k+1)=v(:,k+1)-v(:,k);
    deltazz(:,k+1)= zz(:,k+1)-zz(:,k);  
    errorm1(:,1:50)=0;
    erroriaet1(:,1:50)=0;
    if k>=50
    errorm1(:,k+1)=abs(v(:,k+1)-r)+errorm1(:,k);
    erroriaet1(:,2)=0;
    erroriaet1(:,k+1)=(k+1)*abs(v(:,k+1)-r)+erroriaet1(:,k);
    end
end
%% Plot Results
figure (1)
hold on
grid on
plot(v,'black','LineWidth',2)
figure (2)
hold on
grid on
plot(u(1,:),'--black','LineWidth',2)
hold on
clc;
%%   CFDL with GMDH
 % parameters
landa1=0.002;
landa2=0.001;
ro2=0.003;
ro1=0.968;
f(:,1)=0.137;
deltau(:,1)=1.342;             
v(:,2)=0.8;                  
u(:,1)=1.342;
error(:,2)=0;
for k=2:4000
%% Reference Calculation
    if k>=2 && k<=1000
        r=70; 
    end
    if k>1000 && k<=2000
        r=73;
    end
    if k>2000 && k<=2500
        r=65;
    end
    if k>2500 && k<=2900
        r=69;
    end
    if k>2900
        r=60; 
    end

    Ref(:,k)=r;
%% controller
    f(:,k)=f(:,k-1)+((ro2*deltau(:,k-1))/(landa2+deltau(:,k-1)^2))*(deltazz(:,k)-f(:,k-1)*deltau(:,k-1));
    if  f(:,k)<=0.0001   %% reset
        f(:,k)= f(:,1);
    end
    u(:,k)=u(:,k-1)+((ro1*f(:,k))/(landa1+abs(f(:,k)^2)))*(r-zz(:,k));
%% Path Slope Calculation
    theta(:,k)=randn*0.0001;
    if k==400
        theta(:,k) = 4;
    end
    if k>400 &&  k<1000  
       theta(:,k) = 4+rand*0.01;   
    end
    if k == 2400 
         theta(:,k) = -4;
    end
    if k>2400 &&  k<3000  
        theta(:,k)= -4-rand*0.01; 
    end
    if k>3500 
       theta(:,k) = 0;  
    end

    
%% GMDH Network
    if   ( k>=5 )
       Inputs1=[u(1,1:k-1);zz(1,1:k-1);theta(1,1:k-1);f(1,1:k-1)];
       Targets1=[f(1,2:k)]; 
       nData1 = size(Inputs1,2);
       % Train Data
       nTrainData1 = nData1-2;
       TrainInd1 = 1:nTrainData1;
       TrainInputs1 = Inputs1(:,TrainInd1);
       TrainTargets1 = Targets1(:,TrainInd1);
       % Test Data
       nTestData1 = nData1 - nTrainData1;
       TestInd1 = nTrainData1+1:nData1;
       TestInputs1 = Inputs1(:,TestInd1);
       TestTargets1 = Targets1(:,TestInd1);
        
%% Create and Train GMDH Network
       params.MaxLayerNeurons =5;                                          % Maximum Number of Neurons in a Layer 5
       params.MaxLayers = 6;                                               % Maximum Number of Layers 6
       params.alpha = 0.95;                                                % Selection Pressure 0.95
       params.pTrain = 0.7;                                                % Train Ratio 0.9
       [gmdh] = GMDH(params, TrainInputs1, TrainTargets1);
%% Calculation of fi
        Inputs1=[u(1,k) ;zz(1,k);theta(1,k);f(1,k)]; 
        Outputs1 = ApplyGMDH(gmdh, Inputs1);  
        TestOutputs1 = Outputs1;                                           % fi calculated
%% Seconf GMDH for y Calculation
       Inputs=[u(1,1:k-1);zz(1,1:k-1);theta(1,1:k-1)];
       Targets=[zz(1,2:k)];
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
       % Create and Train GMDH Network
       params.MaxLayerNeurons = 5;                                         % Maximum Number of Neurons in a Layer 
       params.MaxLayers = 6;                                               % Maximum Number of Layers 
       params.alpha = 0.95;                                                % Selection Pressure 
       params.pTrain = 0.7;                                                % Train Ratio
       [gmdh] = GMDH(params, TrainInputs, TrainTargets);
       % Calculation of y
        Inputs=[u(1,k) ;zz(1,k);theta(1,k)];
        Outputs = ApplyGMDH(gmdh, Inputs);
        TestOutputs = Outputs;

%% GMDH-MFAC Control signal calculation
        u(:,k+1)=u(:,k)+((ro1*TestOutputs1)/(landa1+abs(TestOutputs1^2)))*(r-TestOutputs);
     end
    %% Subway Train Model
    M = 500;
    G = 9.8; 
    PI=3.14; 
    wr(:,k)=M*G*sin(theta(:,k)); 
    wc(:,k)=3.5*M*G*PI*10^-5; 
    wt(:,k)=1.3*M*G*10^-4; 
    L=2391;   
    yymax=78;   
    umax=0.46;  
    wbar=1;  
    Mt=M;  
    fa(:,k)=+wc(:,k)+wt(:,k);  
    fb(:,k)=4500+150*v(:,k)+v(:,k)^2; 
    if r==0 && k>3505     
        fa(:,k)=0;
        fb(:,k)=0+150*v(:,k)+v(:,k)^2;   
    end 
    v(:,k+1)=wbar*v(:,k)+(wbar/Mt)*(u(:,k)-fa(:,k)-fb(:,k));                               
    if v(:,k+1)<0        
        v(:,k+1)=0;       
    end
%% Data Quantization
   
    tetaqantizer = 0.99;
    basicZ0 =83;
    resolution = 46;
    ysend = v(:,k+1);
    [zz(:,k+1)] = quantizer(ysend,tetaqantizer, basicZ0,resolution); 
%% Event-Triggered Scenario
%     if Ref(:,k)~= Ref(:,k-1)
%         zz(:,k+1)=v(:,k+1);
%     end
%% Error Calculation  
     deltau(:,k)=u(:,k)-u(:,k-1);
     deltav(:,k+1)=v(:,k+1)-v(:,k);
     deltazz(:,k+1)= zz(:,k+1)-zz(:,k);
     errorm(:,2)=0;   
     errorm(:,50)=0;
     erroriaet(:,50)=0;
     if k>=50
     errorm(:,k+1)=abs(v(:,k+1)-r)+errorm(:,k);    
     erroriaet(:,2)=0;
     erroriaet(:,k+1)=(k+1)*abs(v(:,k+1)-r)+erroriaet(:,k);
     end
end
%% Plot Results Results
hold on
figure (1)
hold on
grid on
plot(v,'b','LineWidth',2)    
hold on
plot(Ref(1,:),'r','LineWidth',2)
bar(-1*theta(1,:),'black','LineWidth',2)
title 'OutPut Tracking'
ylabel 'Speed'
xlabel 'Time (Sec)'
legend 'QMFAC' 'EDD-QMFAC' 'reference' 'Slope' 
figure (2)
hold on
grid on
plot(u(1,:),'b','LineWidth',2)
hold on
ylabel 'Amplitude '
xlabel 'Time (Sec)'
title 'Control Signal'
legend ' QMFAC'  'EDD-QMFAC'
resolution = 46;
tetaqantizer = 0.99;
steps=4000;
for i= 1:resolution
   
    zi(i)=  70*(tetaqantizer^i);
    quanlevel=zi(i)*ones(1,steps);
    figure (78)
    hold on
    grid on
    plot(quanlevel,'--blue','LineWidth',1) 
   
    zi(i)=  60*(tetaqantizer^i);
    quanlevel=zi(i)*ones(1,steps);
    figure (78)
    hold on
    grid on
    plot(quanlevel,'--blue','LineWidth',1) 
    title 'Quantization Levels'
end