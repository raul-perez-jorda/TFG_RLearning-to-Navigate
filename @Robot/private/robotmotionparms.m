function mps = robotmotionparms()
% Get in MPS a struct with the robot motion parameters.

	mps = struct();
	
    mps.differentmotors=0; % DIFFERENTMOTORS indicates whether to use different motors for both wheels
                        % (if it is 1) or not (0). Notice that even using the same motors, the
                        % odometric pose obtained only from encoder values will differ from reality
                        % due to the encoder resolution of 1 degree, especially in turns
    mps.deadzonemotors=10;
    
    mps.vftestr=35; % rad/seg max wheel turning speed 
    mps.xtestr=0.5;  
    mps.txtestr=100e-3; % time in reaching XTESTR% of max wheel
    mps.ctestr=100; % power given in test
    if (mps.differentmotors==1)
        mps.vftestl=30; 
        mps.xtestl=0.5;  
        mps.txtestl=80; 
        mps.ctestl=100; 
    else
        mps.vftestl=mps.vftestr;
        mps.xtestl=mps.xtestr;
        mps.txtestl=mps.txtestr;
        mps.ctestl=mps.ctestr;
    end

    % other parameters of the real robot
    mps.resolenc=1; % increment of encoder value per turned degree

end
