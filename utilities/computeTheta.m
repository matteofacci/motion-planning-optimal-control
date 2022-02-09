function [thetaFinal,R] = computeTheta(x1,y1,x0,y0)

A = [x1, y1, 0];
B = [x0, y0, 0];

u = B-A;
x_axis = [1, 0, 0];

if x0>=x1
    if y0 >= y1
        thetaFinal = pi+atan2(norm(cross(u,x_axis)),dot(u,x_axis));
    else
        thetaFinal = pi-atan2(norm(cross(u,x_axis)),dot(u,x_axis));
    end
else
    if y0 >= y1
        thetaFinal = pi+atan2(norm(cross(u,x_axis)),dot(u,x_axis));
    else
        thetaFinal = pi-atan2(norm(cross(u,x_axis)),dot(u,x_axis));
    end
end

end