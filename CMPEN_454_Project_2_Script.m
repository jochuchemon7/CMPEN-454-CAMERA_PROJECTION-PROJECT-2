format long

load('Subject4-Session3-Take4_mocapJoints.mat');
load('vue2CalibInfo.mat');
load('vue4CalibInfo.mat');

dimMocap = size(mocapJoints);
dimVue2 = size(vue2);
dimVue4 = size(vue4);

frame_1 = mocapJoints(1,:,:);


%Reading the 3D data
mocapFnum = 700; %frame number 1000
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

%2D to 3D Main()
solution = reconstruct3DFrom2D(vue2,resultCam1, vue4, resultCam2);

%Find Epipolar
answer = findEpipolarLines(worldCoord3DPoints,vue2,resultCam1, vue4, resultCam2);

%Error Measured
measuredError = measureError(worldCoord3DPoints(1:3,1:12),solution);

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

    % set up variable to get viewing ray for cam1           
    X1 = cam1.Rmat(1,1)*cam1.position(1) + cam1.Rmat(1,2)*cam1.position(2) + cam1.Rmat(1,3)*cam1.position(3);
    Y1 = cam1.Rmat(2,1)*cam1.position(1) + cam1.Rmat(2,2)*cam1.position(2) + cam1.Rmat(2,3)*cam1.position(3);
    Z1 = cam1.Rmat(3,1)*cam1.position(1) + cam1.Rmat(3,2)*cam1.position(2) + cam1.Rmat(3,3)*cam1.position(3);
    
    % set up variables to get viewing ray for cam2    
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



%measuredError -> 1.260349926799237e-12



%1.722950000000001   1.597610000000000
%1.641460000000000   1.784930000000000
%1.423840000000001   1.206590000000001

%1.377779999999999   1.473100000000001
%1.910050000000000   1.335040000000000
%1.093350000000000   1.412510000000001


%1.307140000000000   1.107310000000000
%1.434570000000000   1.615490000000001
%1.209600000000001   1.097290000000001


%1.672040000000000   1.748740000000000
%1.546180000000001   1.582000000000000
%0.903168000000001   0.478087000000000

%1.882740000000001   1.562860000000000
%1.541980000000001   1.417399999999999
%0.062396800000000   0.893882000000001

%1.529570000000000   1.559800000000002
%1.390839999999999   1.305280000000001
%0.496933000000000   0.048518100000000


%%%findEpipolarLines function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [EpipolarLines1, EpipolarLines2] = findEpipolarLines(worldCoord3DPoints, cam1, cam1PixelCoords, cam2, cam2PixelCoords)
    filenamevue2mp4 = 'Subject4-Session3-24form-Full-Take4-Vue2.mp4';

    x1 = [0];
    y1 = [0];
    x2 = [0];
    y2 = [0];

    for i = 1:12
        x1(i) = cam1PixelCoords(1, i)
    end

    for i = 1:12
        y1(i) = cam1PixelCoords(2, i)
    end

    for i = 1:12
        x2(i) = cam2PixelCoords(1, i)
    end

    for i = 1:12
        y2(i) = cam2PixelCoords(2, i)
    end

         
    recovered3DPoints = reconstruct3DFrom2D(cam1, cam1PixelCoords, cam2, cam2PixelCoords)
        
    EpipolarLines1 = 0;
    EpipolarLines2 = 0;
    
    
    %do Hartley preconditioning     
    savx1 = x1; savy1 = y1; savx2 = x2; savy2 = y2;
    mux = mean(x1);
    disp(mux)
    muy = mean(y1);
    disp(muy)
    disp(std(x1))
    stdxy = (std(x1)+std(y1))/2;
    disp(stdxy)
    T1 = [1 0 -mux; 0 1 -muy; 0 0 stdxy]/stdxy;
    nx1 = (x1-mux)/stdxy;
    ny1 = (y1-muy)/stdxy;
    mux = mean(x2);
    muy = mean(y2);
    stdxy = (std(x2)+std(y2))/2;
    T2 = [1 0 -mux; 0 1 -muy; 0 0 stdxy]/stdxy;
    nx2 = (x2-mux)/stdxy;
    ny2 = (y2-muy)/stdxy;
        
     %=   A = [];
     %   for i=1:12;
            %disp(cam1PixelCoords(i))
            %disp(cam1PixelCoords(i,2))
     %       A(i,:) = [cam1PixelCoords(1,i)*cam2PixelCoords(1,i) cam1PixelCoords(1,i)*cam2PixelCoords(2,i) cam1PixelCoords(1,i) cam1PixelCoords(2,i)*cam2PixelCoords(1,i) cam1PixelCoords(2,i)*cam2PixelCoords(2,i) cam1PixelCoords(2,i) cam2PixelCoords(1,i) cam2PixelCoords(2,i) cam2PixelCoords(3,i)];
     %   end
     
     A = [];
     disp(nx1)
    for i=1:length(nx1)
        A(i,:) = [nx1(i)*nx2(i) nx1(i)*ny2(i) nx1(i) ny1(i)*nx2(i) ny1(i)*ny2(i) ny1(i) nx2(i) ny2(i) 1];
    end
    
        [u,d] = eigs(A' * A,1,'SM');
        F = reshape(u,3,3);

    %make F rank 2
    oldF = F;
    [U,D,V] = svd(F);
    D(3,3) = 0;
    F = U * D * V';

    F = T2' * F * T1;

     
  disp("hello")
    disp(F)

    % plot points2D2 and points2D4 onto frame
    vue2Video = VideoReader(filenamevue2mp4);
    vue2Video.CurrentTime = (700-1)*(50/100)/vue2Video.FrameRate;
    vid2Frame = readFrame(vue2Video);

    colors =  'bgrcmykbgrcmykbgrcmykbgrcmykbgrcmykbgrcmykbgrcmyk';

    figure(700);
    set(gcf, 'Position',  [100, 100, 1000, 400])



    [nr,nc,nb] = size(vid2Frame);

    L = F * [x1; y1; ones(size(x1))];
    subplot(1,2,1)
             title('vue2');
             image(vid2Frame);

    figure(700); clf; imagesc(vid2Frame); axis image;

    for i=1:length(L);
        a = L(1,i); b = L(2,i); c=L(3,i);
        if (abs(a) > (abs(b)))
           ylo=0; yhi=nr; 
           xlo = (-b * ylo - c) / a;
           xhi = (-b * yhi - c) / a;
           hold on
           h=plot([xlo; xhi],[ylo; yhi]);
           set(h,'Color',colors(i),'LineWidth',2);
           hold off
           drawnow;
        else
           xlo=0; xhi=nc; 
           ylo = (-a * xlo - c) / b;
           yhi = (-a * xhi - c) / b;
           hold on
           h=plot([xlo; xhi],[ylo; yhi],'b');
           set(h,'Color',colors(i),'LineWidth',2);
           hold off
           drawnow;
        end
    end


end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%3DReconstructionError function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function recoveredError = measureError(original3DPoints, recovered3DPoints)
    
    recoveredError = [0];
    
    for i=1:12
        
        original = [original3DPoints(1,i); original3DPoints(2,i); original3DPoints(3,i)];
        recovered = [recovered3DPoints(1,i); recovered3DPoints(2,i); recovered3DPoints(3,i)];
              
        recoveredError(i) = sqrt((original(1)-recovered(1)).^2 + (original(2)-recovered(2)).^2 + (original(3)-recovered(3)).^2);    
        
    end
    
    recoveredError = sum(recoveredError) / 12;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




