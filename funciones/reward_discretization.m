function [reward, estoy_siguiendo_pared] = reward_discretization(desired_action,collision,sars, triplete_der)

    reward = 0;
    estoy_siguiendo_pared = false;
    if(desired_action==1)
        reward = (reward + 5);
    end

    if(collision==true)
        reward = reward - 100;
    end

    if(~isempty(sars))
        last_action = sars( length(sars(:,2)), 2);
        if (last_action==2 && desired_action==3) || (last_action==3 && desired_action==2)
            reward = (reward - 0.2);
        end
    end

    if(triplete_der.alpha > deg2rad(-20) && triplete_der.alpha < deg2rad(20) && ...
            triplete_der.d > 0.5 && triplete_der.d < 1 && ...
            desired_action == 1)
        reward = (reward + triplete_der.qual/100 * 40);
        estoy_siguiendo_pared = true;
    end
end

