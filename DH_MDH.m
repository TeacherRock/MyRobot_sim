function Output = DH_MDH(type)
    switch type
        case 'DH'
            % DH
            theta_DH = [     0,   pi/2,      0,       0,      0,       0];
            d_DH =     [  34.5,      0,      0,    29.5,      0,    10.2];
            a_DH =     [     8,     27,      9,       0,      0,       0];
            alpha_DH = [  pi/2,      0,   pi/2,   -pi/2,   pi/2,       0];
            Output = [theta_DH', d_DH', a_DH', alpha_DH'];
        case 'MDH'
            % MDH
            theta_MDH = [ 0,      0,   pi/2,      0,       0,      0,       0];
            d_MDH =     [ 0,   34.5,      0,      0,    29.5,      0,    10.2];
            a_MDH =     [ 0,      8,     27,      9,       0,      0,       0];
            alpha_MDH = [ 0,   pi/2,      0,   pi/2,   -pi/2,  pi/2,        0];
            Output =       [theta_MDH', d_MDH',  b_MDH', alpha_MDH'];
    end
    
end
