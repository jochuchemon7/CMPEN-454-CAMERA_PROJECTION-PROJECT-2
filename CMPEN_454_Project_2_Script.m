%{
Description: This script would perform forward and backward projection
staring from a set of 3D points and getting 2D points, then those 2D points
back to 3D and finaly calculate the SSD from the original and
reconstructed 3D points
%}


format long

profile on -history

load('Subject4-Session3-Take4_mocapJoints.mat');
load('vue2CalibInfo.mat');
load('vue4CalibInfo.mat');

dimMocap = size(mocapJoints);
dimVue2 = size(vue2);
dimVue4 = size(vue4);

frame_1 = mocapJoints(1,:,:);


%Reading the 3D data
mocapFnum = 420; %frame number 420
x = mocapJoints(mocapFnum,:,1); %array of 12 X coordinates
y = mocapJoints(mocapFnum,:,2); % Y Coordinates
z = mocapJoints(mocapFnum,:,3); % Z Coordinates
conf = mocapJoints(mocapFnum,:,4); %Confidence values

%Intristic: orientation & position
%extristic(Rotation & translation): foclen,  

%Projecting a frame number mocapFnum
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
filenamevue2mp4 = 'Subject4-Session3-24form-Full-Take4-Vue2.mp4';
filenamevue4mp4 = 'Subject4-Session3-24form-Full-Take4-Vue4.mp4';

vue2video = VideoReader(filenamevue2mp4);
vue4video = VideoReader(filenamevue4mp4);

vue4video.CurrentTime = (mocapFnum - 1) * (50/100)/ vue2video.FrameRate;
vue2video.CurrentTime = (mocapFnum - 1) * (50/100)/vue2video.FrameRate;

vid2Frame = readFrame(vue2video);
vid4Frame = readFrame(vue4video);

image(vid2Frame);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%Main()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Projected2DPoints Main()

worldCoord3DPoints = [x;y;z;ones(1,12)];


resultCam1 = project3DTo2D(vue2, worldCoord3DPoints);
resultCam2 = project3DTo2D(vue4, worldCoord3DPoints);
disp("2D projection from 3D camera 1: ");
disp(resultCam1);
disp("2D projection from 3D camera 2: ");
disp(resultCam2)


%2D to 3D Main()
solution = reconstruct3DFrom2D(vue2,resultCam1, vue4, resultCam2);
disp("Reconstruction of 3D points from 2D: ");
disp(solution);

%Find Epipolar
answer = findEpipolarLines(worldCoord3DPoints,vue2,resultCam1, vue4, resultCam2);

%Error Measured
measuredError = measureError(worldCoord3DPoints(1:3,1:12),solution);
disp("Measured error by SSD: ");
disp(measuredError)

p = profile("info");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%3D -> 2D function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function projected2DPoints = project3DTo2D(cam, worldCoord3DPoints)

%Solve for perspectiveProjection matrix
perspectiveProjection = cam.Kmat * cam.Pmat * worldCoord3DPoints;

%Prepare 2D points vector
projected2DPointsX = [0];
projected2DPointsY = [0];

for i = 1:12
    projected2DPointsX(i) = perspectiveProjection(1,i) ./ perspectiveProjection(3,i); 
    projected2DPointsY(i) = perspectiveProjection(2,i) ./ perspectiveProjection(3,i); 
end

%Combine points into a matrix
projected2DPoints = [projected2DPointsX; projected2DPointsY; ones(1,12)];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%2D -> 3D funtion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function recovered3DPoints = reconstruct3DFrom2D(cam1, cam1PixelCoords, cam2, cam2PixelCoords)

    %Calculate the X,Y & Z points from the t part of the equation for camera 1      
    X1 = cam1.Rmat(1,1)*cam1.position(1) + cam1.Rmat(1,2)*cam1.position(2) + cam1.Rmat(1,3)*cam1.position(3);
    Y1 = cam1.Rmat(2,1)*cam1.position(1) + cam1.Rmat(2,2)*cam1.position(2) + cam1.Rmat(2,3)*cam1.position(3);
    Z1 = cam1.Rmat(3,1)*cam1.position(1) + cam1.Rmat(3,2)*cam1.position(2) + cam1.Rmat(3,3)*cam1.position(3);
    
    %Calculate the X,Y & Z points from the t part of the equation for camera 2    
    X2 = cam2.Rmat(1,1)*cam2.position(1) + cam2.Rmat(1,2)*cam2.position(2) + cam2.Rmat(1,3)*cam2.position(3);
    Y2 = cam2.Rmat(2,1)*cam2.position(1) + cam2.Rmat(2,2)*cam2.position(2) + cam2.Rmat(2,3)*cam2.position(3);    
    Z2 = cam2.Rmat(3,1)*cam2.position(1) + cam2.Rmat(3,2)*cam2.position(2) + cam2.Rmat(3,3)*cam2.position(3);
    
    T1 = [X1;Y1;Z1];
    T2 = [X2;Y2;Z2];

    recovered3DPoints = [0;0;0];
    
    for i = 1:12
        
        %Calculate first part of the equation
        camLoc1 = -transpose(cam1.Rmat) * T1;
        camLoc2 = -transpose(cam2.Rmat) * T2;
        
        %Calculate second part of the equation
        VectorPoint1 = transpose(cam1.Rmat) * inv(cam1.Kmat) * cam1PixelCoords(1:3,i);
        VectorPoint2 = transpose(cam2.Rmat) * inv(cam2.Kmat) * cam2PixelCoords(1:3,i);
               
        %Add both parts of the equation
        Pw1 = camLoc1 + VectorPoint1;
        Pw2 = camLoc2 + VectorPoint2;
        
        %Calculate the U-vectors
        U1 = VectorPoint1 ./ norm(VectorPoint1);
        U2 = VectorPoint2 ./ norm(VectorPoint2);
        U3 =  cross(U1, U2) / norm(cross(U1, U2));

        %Solve for a, b, and d
        A = [U1(1) -U2(1) U3(1);...
            U1(2) -U2(2) U3(2);...
            U1(3) -U2(3) U3(3)];

        %Solve and b values 
        b = [camLoc2(1)-camLoc1(1); camLoc2(2)-camLoc1(2); camLoc2(3)-camLoc1(3)];
        x = A\b;        
        
        %Set a and b
        a = x(1);
        b = x(2);
        
        %After solving for a,b,d solve for p1,p2 and p
        p1 = camLoc1 + a*U1;
        p2 = camLoc2 + b*U2;        
        p = (p1 + p2) / 2;
    
        %Assing the values to the matrix
        recovered3DPoints(1:3,i) = [-1*p(1);-1*p(2);-p(3)];
              
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%findEpipolarLines function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [EpipolarLines1, EpipolarLines2] = findEpipolarLines(worldCoord3DPoints, cam1, cam1PixelCoords, cam2, cam2PixelCoords)
   
    EpipolarLines1 = 0;
    EpipolarLines2 = 0;

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%3DReconstructionError function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function recoveredError = measureError(original3DPoints, recovered3DPoints)
    %For storage
    recoveredError = [0];
    
    for i=1:12
        
        %Get original and recovered values into one
        original = [original3DPoints(1,i); original3DPoints(2,i); original3DPoints(3,i)];
        recovered = [recovered3DPoints(1,i); recovered3DPoints(2,i); recovered3DPoints(3,i)];
              
        %SSD calculation
        recoveredError(i) = sqrt((original(1)-recovered(1)).^2 + (original(2)-recovered(2)).^2 + (original(3)-recovered(3)).^2);    
        
    end
    
    %Get average
    recoveredError = sum(recoveredError) / 12;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




