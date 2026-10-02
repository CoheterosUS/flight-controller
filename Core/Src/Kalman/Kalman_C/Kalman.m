function [P, x]  = Kalman(IMU_available, GPS_available, ...
    MAG_available, BAR_available, accel_IMU, omega_IMU, z_GPS, z_MAG, z_BAR, R_GPS, ...
    R_MAG, R_BAR, Q, DCM_bIMU, DCM_MAGb, B_e, h, g, P, x)

skew = @(x)[0 -x(3)  x(2); x(3) 0 -x(1);-x(2) x(1) 0];

DCM_be = @(q) [q(1)^2+q(2)^2-q(3)^2-q(4)^2,...
             2*(q(2)*q(3)+q(1)*q(4)),...
             2*(q(2)*q(4)-q(1)*q(3));
             2*(q(2)*q(3)-q(1)*q(4)),...
             q(1)^2-q(2)^2+q(3)^2-q(4)^2,...
             2*(q(3)*q(4)+q(1)*q(2));
             2*(q(2)*q(4)+q(1)*q(3)),...
             2*(q(3)*q(4)-q(1)*q(2)),...
             q(1)^2-q(2)^2-q(3)^2+q(4)^2];

DCM_eb = @(q) DCM_be(q)';


if GPS_available

    H = zeros(6, 9);
    H(1:3,1:3) = eye(3);
    H(4:6,4:6) = eye(3);
    [delta_x, P] = kalman_update(z_GPS, H, P, R_GPS, x(1:6));
    [x] = kalman_delta_sum(x, delta_x);

end

if MAG_available

    H = zeros(3,9);
    H(1:3,7:9) = DCM_MAGb*skew(DCM_be(x(7:10))*B_e);
    [delta_x, P] = kalman_update(z_MAG, H, P, R_MAG, DCM_MAGb * DCM_be(x(7:10))*B_e);
    [x] = kalman_delta_sum(x, delta_x);

end

if BAR_available
    p0 = 101325;
    T0 = 288.15;
    alpha = 0.0065;
    R = 287.05;
    H = zeros(1,9);
    H(1,3) = p0 * (g(3)/(R*T0)) * (1 + alpha*x(3)/T0)^(g(3)/(R*alpha)-1);
    [delta_x, P] = kalman_update(z_BAR, H, P, R_BAR, p0 * (1 + alpha*x(3)/T0)^(g(3)/(R*alpha)));
    [x] = kalman_delta_sum(x, delta_x);

end

if IMU_available

    omega_IMU = omega_IMU*pi/180;

    accel_b = DCM_bIMU*accel_IMU;
    omega_b = DCM_bIMU*omega_IMU;

    r_t = x(1:3);
    v_t = x(4:6);
    q = x(7:10);

    A = zeros(9,9);
    A(1:3,4:6) = eye(3);
    A(4:6,7:9) = -DCM_eb(q) * skew(accel_b);
    A(7:9,7:9) = -skew(omega_b);   % -[omega x]
    P = kalman_propagate(P, A, Q, h);
    
    v_dot = DCM_eb(q) * accel_b + g;
    v_t = v_t + h*v_dot;
    r_t = r_t + h*v_t;

    
    normOmega_b = norm(omega_b);

    if normOmega_b > 1e-10

        delta_q = [cos(normOmega_b*h/2); sin(normOmega_b*h/2)*omega_b/normOmega_b];

    else

        delta_q = [1; omega_b*h/2];

    end

    q = quatmultiply(q', delta_q')';
    q = q/norm(q);

    x(1:3) = r_t;
    x(4:6) = v_t;
    x(7:10) = q;

end

P = (P + P')/2;