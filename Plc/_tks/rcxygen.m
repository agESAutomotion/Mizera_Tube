%rcxygen
%input record format
%xc yc xm ym

clear all
trail=input('file trail -> ','s');
nam_ext = strcat('rcxygen',trail,'.dat');
eval(['load ',nam_ext]);
nam = strcat('rcxygen',trail);
samp = eval(nam);

xc=samp(:,1);
yc=samp(:,2);
xm=samp(:,3);
ym=samp(:,4);

%close all

figure;
axis equal;hold on;
plot(xc,yc,'r.-');
plot(xm,ym,'b.-');
grid; title(strcat(nam,' rcxygen rb=cm'));
