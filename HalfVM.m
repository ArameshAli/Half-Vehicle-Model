
% =========================================================================
% Ali Aramesh
% 3rd Assignment : Half Vehicle Model
% 10/05/2020 
% =========================================================================
%%
clc; close all; clear all;

f = 1.6;                          % Frequency[Hz]
Z = 0.1;                          % Damping coefficient is 0.1 for 10 percent damping 
kw = 50000;                       % Tire stiffness [N/m]

mw = 25;                          % Un-sprung mass [kg]
mc = 225;                         % Sprung mass [kg]
m = mw + mc;                      % Quarter vehicle mass(sum of sorung and unsprung for a 1/4 vehicle)[kg]

wn = 2*pi*f;                      % Angular frequency [rad/s]
Keq = m*(wn^2);                   % Equivalent stiffness [N/m]

ks = (kw*Keq)/(kw-Keq);           % Spring stiffness [N/m]
cc = 2*sqrt(m*ks);                % Critical damping when Zeta=1 [Ns/m]
At = 1e-3;                       % Initial displacement 
t_simul = 30;
counter = 1; 
acceleration_engine = readtable('input1(t) 21.txt');


%%

for Z=0.1:0.1:0.9
    c = Z*cc;                      % Damping coefficient [N s/m]
    sim('HOV.slx',t_simul);
    f1= 1/At;
    [txy,f] = tfestimate(acceleration_input_Front,acceleration_output_Front,[],[],[],f1);   % Returns a vector of frequencies. 
    ACC(:,counter) = abs(txy);     % Storing acceleration amplification response
    FREQ(:,counter) = f;           
    figure
    semilogx(f,abs(txy))           %  Logarithmic scales for the x-axis
    figure
    plot(tout,acceleration_output)
    figure
    plot(tout,acceleration_engine)
    matrix = [tout acceleration_output];
    filename = '/Users/aliaramesh/Documents/JMDSAE/2018-2019/UDeusto/ModuleMaterials/VibroacousticComfortInElectricVehicles/New/th_weighted.m';
    run(filename);
    counter=counter+1;
end
figure
hold on
semilogx(FREQ(:,1:(counter-1)),ACC(:,1:(counter-1)));
xlabel('frequency');
ylabel('transmissibility');
legend('Z=0.1','Z=0.2','Z=0.3','Z=0.4','Z=0.5','Z=0.6','Z=0.7','Z=0.8','Z=0.9');
hold off
%%





