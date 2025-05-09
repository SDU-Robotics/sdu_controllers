clc
clear
close all

%%
pid = pid_controller;
pid.Kp = 20;
pid.Kd = 5;
pid.Ki = 20;
pid.N = 0;
pid.num_states = 1;
pid.sample_time = 1e-4;

%%
tvec = 0:pid.sample_time:20;

q_d = sin(0.1 * tvec);
dq_d = q_d * 0;
u_ff = q_d * 0;

ddq = q_d * 0;
dq = ddq;
q = ddq;

%%
tic
for i = 1:length(tvec) - 1
    % i/length(tvec)

    % tic
    y = pid.step(q_d(i), dq_d(i), u_ff(i), q(i), dq(i));
    % toc

    ddq(i) = 10 * q(i) - 5 * dq(i) + y;
    dq(i + 1) = dq(i) + pid.sample_time * ddq(i);
    q(i + 1) = q(i) + pid.sample_time * dq(i);
end
toc

%%
figure;
plot(tvec, q)
hold on
plot(tvec, q_d)