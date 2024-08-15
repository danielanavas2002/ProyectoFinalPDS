%% Diseño de filtros FIR
clear; clc

% Parámetros
Fs = 1000;
Ts = 1/Fs;

%% Definición de Parametros
% Clase - Pasabajas (1), Pasaaltas (2), Pasabandas (3), Eliminabandas (4)
clase = 1;

% Orden (1 a 4)
Orden = 4;  

% Se selecciona el tipo de ventana a emplear
wsel = 1; 


% Frecuencias de Corte
Fc = 400; % Hz - Pasaalta / Pasabanda
Fc_Low = 100; % Hz - Pasabanda / Eliminabanda
Fc_High = 300; % Hz

wc = 2*pi*Fc*(1/Fs);
wcband = [2*pi*Fc_Low*(1/Fs) 2*pi*Fc_High*(1/Fs)];

% Crear el vector que contiene Fc_Low y Fc_High
Fc_Band = [Fc_Low, Fc_High];

if(wsel == 1)
    % w = ones(size(1:(N+1)))'; % Ventana rectangular
    w = rectwin(Orden+1);     % Ventana rectangular
elseif(wsel == 2)
    w = bartlett(Orden+1);    % Ventana de Bartlett
elseif(wsel == 3)
    w = hann(Orden+1);        % Ventana de Hann
elseif(wsel == 4)
    w = hamming(Orden+1);     % Ventana de Hamming
elseif(wsel == 5)
    w = blackman(Orden+1);    % Ventana de Blackman
elseif(wsel == 6) 
    w = kaiser(Orden+1, 2.5); % Ventana de Kaiser
end

%% Obtener coeficientes
if clase == 1
    b = fir1(Orden, wc/pi, 'low', w, 'noscale');  
elseif clase == 2
    b = fir1(Orden, wc/pi, 'high', w, 'noscale');  
elseif clase == 3
    b = fir1(Orden, wcband/pi, 'bandpass', w, 'noscale');  
elseif clase == 4
    b = fir1(Orden, wcband/pi, 'stop', w, 'noscale');  
end

b

%% Graficas 
close all

% Calcular la respuesta en frecuencia
[h, w] = freqz(b, 1, 1024); % 1024 puntos de frecuencia



% Convertir la frecuencia a Hz
f = w * (Fs / (2*pi));

% Graficar magnitud no dB vs Hz
figure;
plot(f, abs(h));
title('Respuesta en Frecuencia (Magnitud no dB vs Hz)');
xlabel('Frecuencia (Hz)');
ylabel('Magnitud');
grid on;

% Graficar magnitud no dB vs rad/muestra
figure;
plot(w, abs(h));
title('Respuesta en Frecuencia (Magnitud no dB vs rad/muestra)');
xlabel('Frecuencia (rad/muestra)');
ylabel('Magnitud');
grid on;

% Convertir la magnitud a dB
H_dB = 20 * log10(abs(h));

% Graficar magnitud dB vs Hz
figure;
plot(f, H_dB);
title('Respuesta en Frecuencia (Magnitud dB vs Hz)');
xlabel('Frecuencia (Hz)');
ylabel('Magnitud (dB)');
grid on;

% Graficar magnitud dB vs rad/muestra
figure;
plot(w, H_dB);
title('Respuesta en Frecuencia (Magnitud dB vs rad/muestra)');
xlabel('Frecuencia (rad/muestra)');
ylabel('Magnitud (dB)');
grid on;

