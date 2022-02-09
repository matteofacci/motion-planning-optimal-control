function [thetaFinal,R] = computeTheta(x0,y0,x1,y1)

A = [x0, y0, 0];
B = [x1, y1, 0];

u = B-A;
x_axis = [1, 0, 0];

if x1>=x0
    if y1 >= y0
        thetaFinal = pi+atan2(norm(cross(u,x_axis)),dot(u,x_axis));
    else
        thetaFinal = pi-atan2(norm(cross(u,x_axis)),dot(u,x_axis));
    end
else
    if y1 >= y0
        thetaFinal = pi+atan2(norm(cross(u,x_axis)),dot(u,x_axis));
    else
        thetaFinal = pi-atan2(norm(cross(u,x_axis)),dot(u,x_axis));
    end
end

end