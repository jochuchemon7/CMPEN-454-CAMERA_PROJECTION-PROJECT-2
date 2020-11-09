

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