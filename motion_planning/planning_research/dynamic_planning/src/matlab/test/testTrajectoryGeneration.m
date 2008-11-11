function [x, y, theta] = testTrajectoryGeneration(x, y, theta, num_thetas, params, fig_num)

figure(fig_num);hold on;
plot(x, y, '.b');
for t=0:params.delta_t:params.max_t  
    if(params.w_hat==0)
        x = x + params.v_hat*params.delta_t*cos(theta);
        y = y + params.v_hat*params.delta_t*sin(theta);
    else
        x = x + (-params.v_hat/params.w_hat)*sin(theta) + (params.v_hat/params.w_hat)*sin(theta + params.w_hat*params.delta_t);
        y = y + (params.v_hat/params.w_hat)*cos(theta) - (params.v_hat/params.w_hat)*cos(theta + params.w_hat*params.delta_t);
        theta = theta + params.w_hat*params.delta_t;
    end
    plot(x, y, '.b');
end

