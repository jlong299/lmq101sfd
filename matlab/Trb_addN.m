% Add noise to data
% Data source file : din_ori_4b.dat 
%   data description : 4bit soft info data as Turbo decoder input, 
%   e.g. 9,7,9,...
% Output file :  din_N_4b.dat
%   description :  input + noise 4bit.  e.g. 3,15,0,12,...  

clear all;
din_ori_4b = load('din_ori_4b.dat');
din_ori = 8 - din_ori_4b;

SNR = 0;
%din_N = awgn( din_ori, SNR);

% Noise 
N_gen = randn(length(din_ori),1);
%mean(N_gen.^2);  %variance

% add noise
din_N = din_ori + N_gen/(10^(SNR/20));   
%mean( ((10^(SNR/20))*N_gen).^2 );

% change to 4 bits
din_N_x4 = din_N * 4;
din_N_x4_round = round( din_N_x4 );
% signed : -7 ~ 7
din_N_4b_s = zeros(length(din_N_x4_round),1);
for i = 1:length(din_N_x4_round)
    if din_N_x4_round(i) > 7 
        din_N_4b_s(i) = 7;
    elseif din_N_x4_round(i) < -7
        din_N_4b_s(i) = -7;
    else
        din_N_4b_s(i) = din_N_x4_round(i);
    end
end

% unsigned :  9 ~ 7
din_N_4b = din_N_4b_s;
for i = 1 : length(din_N_4b_s)
    if (din_N_4b_s(i) < 0 )
        din_N_4b(i) = din_N_4b_s(i) + 16;
    end
end
    
outf = fopen('din_N_4b.dat','w');
for j = 1 : length(din_N_4b)
    fprintf(outf , '%d\n' , din_N_4b(j));
end
fclose(outf);
        