epsilon = [];
num_episodes = 1:1:350;

for i = num_episodes
epsilon = [epsilon; get_epsilon("constante",num_episodes(i),1/i)];
end

plot(epsilon)