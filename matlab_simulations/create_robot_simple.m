clear variables
close all
clc 

%% definizione link
L(1) = Link('revolute','d',0,'a',0,'alpha',-pi/2);
L(2) = Link('revolute','d',0,'a',0,'alpha',+pi/2);
L(3) = Link('revolute','d',0.27,'a',0.085,'alpha',pi/2);
L(4) = Link('revolute','d',0,'a',0.06,'alpha',pi/2);
L(5) = Link('revolute','d',0.27,'a',0,'alpha',-pi/2);
L(6) = Link('revolute','d',0,'a',0,'alpha',pi/2);
L(7) = Link('revolute','d',0,'a',0,'alpha',0);

%% definizione robot
R = SerialLink(L,'name','SIA 5F','fast','1');

R.base = [eye(3) [0;0.0;0]; 0 0 0 1];
R.tool = [eye(3) [0;0.0;0]; 0 0 0 1];


%% plot
q = [0 0 0 pi/2 0 0 0];
R.plot(q)
R.model3d='YASKAWA/SIA5F';
R.plot3d(q,'workspace', [-0.8 0.8 -0.8 0.8 0 1.2],...
                       'STLunits','m',...
                       'view',[-45 15],...
                       'color',{'SlateGray1','SlateGray4','SlateGray1','SlateGray1',...
                                'SlateGray4','SlateGray4','SlateGray1'});
