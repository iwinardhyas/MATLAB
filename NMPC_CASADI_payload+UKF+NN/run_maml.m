net = initialize_maml_nn();
lr_inner = 0.005;
lr_outer = 0.001;

% Asumsi net = create_maml_network() dan lr_inner/lr_outer sudah didefinisikan
N_sim = 1000; % Contoh jumlah langkah simulasi

% Inisialisasi array untuk menyimpan error residual
history_residual_error = zeros(1, N_sim);
time_steps = 1:N_sim; % Untuk plot sumbu X

for t = 1:N_sim
    [x_input, y_true] = generate_dummy_nn_data(); % Ambil data dummy
    disp(x_input);
    disp(y_true');

    % Panggil fungsi update dan dapatkan loss_inner
    [net, current_error] = update_maml_online(net, x_input, y_true, lr_inner, lr_outer);
    disp(net);
    
    % Simpan error residual untuk iterasi ini
    history_residual_error(t) = current_error;
end

%% Plot Hasil Error Residual
figure;
plot(time_steps, history_residual_error, 'b', 'LineWidth', 1.5);
title('Error Residual NN (MSE) Selama Pembelajaran Online');
xlabel('Langkah Waktu Simulasi');
ylabel('Mean Squared Error (MSE)');
grid on;
legend('MSE Residual');

function [x_input_dummy, y_true_dummy] = generate_dummy_nn_data()
% generate_dummy_nn_data: Fungsi untuk menghasilkan data dummy input dan output NN.
%
% Output:
%   x_input_dummy: Vektor dummy input NN (ukuran 17x1).
%                  Mewakili [13 states + 4 kontrol input].
%   y_true_dummy: Vektor dummy target residual (ukuran 13x1).
%                 Mewakili 13 elemen dinamika residual.

    % Dummy x_input:
    % 13 elemen pertama untuk state drone (posisi, orientasi, kecepatan, massa estimasi)
    % 4 elemen berikutnya untuk input kontrol (gaya dorong motor)
    x_input_dummy = rand(1,17); % Menggunakan nilai acak antara 0 dan 1
    
    % Dummy y_true:
    % 13 elemen untuk dinamika residual
    y_true_dummy = rand(13, 1) * 0.1; % Menggunakan nilai acak yang lebih kecil
                                     % karena residual biasanya kecil
                                     % dikalikan 0.1 untuk membuatnya lebih realistis kecil
end