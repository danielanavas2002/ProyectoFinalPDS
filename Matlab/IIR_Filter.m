%% Diseño de filtros IIR
clear; clc

% Parámetros
Fs = 1000;
Ts = 1/Fs;

%% Definición de Parametros
% Subtipos - Butterworth (1), Chebyshev I (2), Chebyshev II (3), Eliptico (4)
subtipo = 1;

% Clase - Pasabajas (1), Pasaaltas (2), Pasabandas (3), Eliminabandas (4)
clase = 2;

% Orden (1 a 5)
Orden = 4;  

% Frecuencias de Corte
Fc = 200; % Hz - Pasaalta / Pasabajas
Fc_Low = 100; % Hz - Pasabanda / Eliminabanda
Fc_High = 200; % Hz

% Crear el vector que contiene Fc_Low y Fc_High
Fc_Band = [Fc_Low, Fc_High];

% Definir Ws
Ws = Fc/(Fs/2);
Ws_Band = [Fc_Low/(Fs/2), Fc_High/(Fs/2)];
%% Obtener coeficientes
if subtipo == 1
    if clase == 1
        [b,a] = butter(Orden, 2*Fc*Ts, 'low');
    elseif clase == 2
        [b,a] = butter(Orden, 2*Fc*Ts, 'high');
    elseif clase == 3
        [b,a] = butter(Orden/2, 2*Fc_Band*Ts,'bandpass');
    elseif clase == 4
        [b,a] = butter(Orden/2, 2*Fc_Band*Ts,'stop');
    end
elseif subtipo == 2
    if clase == 1
        [b,a] = cheby1(Orden, 0.5, 2*Fc*Ts,'low');
    elseif clase == 2
        [b,a] = cheby1(Orden, 0.5, 2*Fc*Ts,'high');
    elseif clase == 3
        [b,a] = cheby1(Orden/2, 0.5, 2*Fc_Band*Ts,'bandpass');
    elseif clase == 4
        [b,a] = cheby1(Orden/2, 0.5, 2*Fc_Band*Ts,'stop');
    end
elseif subtipo == 3
    if clase == 1
        [b,a] = cheby2(Orden, 0.5, 2*Fc*Ts,'low');
    elseif clase == 2
        [b,a] = cheby2(Orden, 0.5, 2*Fc*Ts,'high');
    elseif clase == 3
        [b,a] = cheby2(Orden/2, 50, 2*Fc_Band*Ts,'bandpass');
    elseif clase == 4
        [b,a] = cheby2(Orden/2, 0.5, 2*Fc_Band*Ts,'stop');
    end
end

bT = b
aT = a

%% Graficas 
close all

% Respuesta en frecuencia
[h, w] = freqz([1 -sqrt(2) 1], [1 1.272792206 0.81]);
f_lpf = w*Fs/(2*pi);  % vector de frecuencia, en Hz

% Gráfica de respuesta en frecuencia del filtro Butterworth
figure(1); clf
subplot(2,1,1);
plot(w, abs(h));
xlim([w(1), w(end)]);
xlabel('f (Hz)');
ylabel('Magnitud');
grid on;
subplot(2,1,2);
plot(w, angle(h));
xlim([w(1), w(end)]);
xlabel('f (Hz)');
ylabel('Fase (grados)');
grid on;
sgtitle('Respuesta en frec. de filtro IIR');

%% Filtrado

data = load('onda_sinusoidal.mat');

x = data.x;
Fs = data.Fs;


filtrada = filter(b,a,x);
t = (0:length(x)-1) / Fs; % Vector de tiempo

figure;
plot(t, x, 'b', t, filtrada, 'r');
xlabel('Tiempo (s)');
ylabel('Amplitud');
title('Señal Original vs Señal Filtrada');
legend('Original', 'Filtrada');
grid on;


%% 
% Calcular la Transformada de Fourier de la señal original y la señal filtrada
X = fft(x);
Filtrada = fft(filtrada);

% Calcular el vector de frecuencia
frecuencia = Fs*(0:length(x)-1)/length(x);

% Gráfica utilizando plot
figure;
subplot(2,1,1);
plot(frecuencia, abs(X), 'b');
xlabel('Frecuencia (Hz)');
ylabel('Magnitud');
title('Espectro de la Señal Original');
grid on;

subplot(2,1,2);
plot(frecuencia, abs(Filtrada), 'r');
xlabel('Frecuencia (Hz)');
ylabel('Magnitud');
title('Espectro de la Señal Filtrada');
grid on;

% Gráfica utilizando stem
figure;
subplot(2,1,1);
stem(frecuencia, abs(X), 'b');
xlabel('Frecuencia (Hz)');
ylabel('Magnitud');
title('Espectro de la Señal Original');
grid on;

subplot(2,1,2);
stem(frecuencia, abs(Filtrada), 'r');
xlabel('Frecuencia (Hz)');
ylabel('Magnitud');
title('Espectro de la Señal Filtrada');
grid on;
%%
% Especificaciones
w1 = 0.25 * pi;
r = 0.95;  % Control de ancho de banda

% Coeficientes del filtro notch
b_notch = [1, -2 * cos(w1), 1];
a_notch = [1, -2 * r * cos(w1), r^2];

% Respuesta en frecuencia del filtro notch
[H_notch, w] = freqz(b_notch, a_notch, 1024);

% Gráfica de la respuesta en frecuencia del filtro notch
figure;
plot(w / pi, abs(H_notch), 'b');
title('Respuesta en frecuencia del filtro notch');
xlabel('Frecuencia Normalizada (\times\pi rad/muestra)');
ylabel('Magnitud');
grid on;

