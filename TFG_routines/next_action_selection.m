function [next_action] = next_action_selection(epsilon, politica_actual, current_state, QLearningOpt)
%Given the inputs it follows an epsilon-greedy policy for selecting the
%next action to take
    random_number = rand(1);
    if(random_number<=epsilon) % movimiento aleatorio
        next_action = randi([1, QLearningOpt.total_actions],1);
    else % la accion con mayor valor
        next_action = politica_actual(current_state); % resto y sumo uno porque los arrays empiezan en 1
    end
end



