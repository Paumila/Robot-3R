function jac = jacobian(j1,j2,j3)

jac = [j1(2) j2(2) j3(2); -j1(1) -j2(1) -j3(1); 1 1 1];

end