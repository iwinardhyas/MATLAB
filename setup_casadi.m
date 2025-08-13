function setup_casadi()
    % === STEP 1: Bersihkan environment ===
    disp('--- Resetting MATLAB path and clearing environment ---');
    restoredefaultpath;      % Kembalikan path ke default
    rehash toolboxcache;     % Refresh toolbox cache
    clearvars;               % Hapus semua variabel di workspace
    clc;                     % Bersihkan Command Window

    % === STEP 2: Tentukan path CasADi ===
    casadi_path = 'C:\Users\DELL\Documents\MATLAB\casadi-3.7.0-windows64-matlab2018b';

    if ~isfolder(casadi_path)
        error('Path CasADi tidak ditemukan: %s', casadi_path);
    end

    % === STEP 3: Tambahkan path CasADi ===
    addpath(casadi_path);
    import casadi.*;
    disp(['CasADi path ditambahkan: ', casadi_path]);

    % === STEP 4: Cek apakah casadiMEX tersedia ===
    mex_path = which('casadiMEX');
    if isempty(mex_path)
        error('casadiMEX tidak ditemukan. Pastikan file .mexw64 ada di folder CasADi.');
    else
        disp(['casadiMEX ditemukan di: ', mex_path]);
    end

    % === STEP 5: Tes CasADi dengan ekspresi sederhana ===
    try
        x = MX.sym('x',2,1);
        f = x'*x;
        disp('Tes CasADi berhasil:');
        disp(f);
    catch ME
        error('Gagal menjalankan tes CasADi: %s', ME.message);
    end

    % === STEP 6: Simpan path agar tidak perlu setup ulang ===
    savepath;
    disp('Setup CasADi selesai. Path sudah disimpan.');
end
