function [q, q_dot] = IK_calib(q_0, link_lengths, p_global)
[T, T1, T2, T3, T4, ~, ~, ~, cur_pos] = FK(q_0, link_lengths);
count = 1;

while norm(p_global(1:3) - cur_pos(1:3)) > 1e-02
    %[q, q_dot] = Damped_LS(q_0, link_lengths, p_global);
    [q, q_dot] = TaskAugmentation(q_0, link_lengths, p_global);
    [T, T1, T2, T3, T4, ~, ~, ~, cur_pos] =  FK(q, link_lengths);
    
    q_0 = q;
    count = count + 1; 
end
end
