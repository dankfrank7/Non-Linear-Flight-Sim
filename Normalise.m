function quat_normed = Normalise(quat) 
q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);
mag = sqrt(q0.^2 + q1.^2 +q2.^2 +q3.^2);
quat_normed = quat/mag;
