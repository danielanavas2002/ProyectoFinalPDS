% Supongamos que ya tienes los vectores aT y bT definidos
% Ejemplo:
% aT = [1.234, 56.78, -91011, 1234567]; % Puedes reemplazar estos valores con los que tengas
% bT = [-98765, 4321, 0.987, -0.12345]; % Puedes reemplazar estos valores con los que tengas

coeficientes = zeros(1,9);

% Asegurarse de que ambos vectores tengan al menos 5 elementos
if length(aT) < 5
    aT = [aT, zeros(1, 5 - length(aT))];
end
if length(bT) < 5
    bT = [bT, zeros(1, 5 - length(bT))];
end

% Combina los vectores asegurando un total de 10 elementos
coeficientesTemp = [bT(1:5), aT(2:5)];

% Inicializa un cell array para guardar los datos formateados
datos_formateados = cell(size(coeficientes));

% Recorre cada elemento de coeficientes y aplica las condiciones
for n = 1:length(coeficientesTemp)
    if (coeficientesTemp(n) >= 100000 || coeficientesTemp(n) <= -10000)
        dato = sprintf('%.1f', coeficientesTemp(n));
    elseif (coeficientesTemp(n) >= 10000 || coeficientesTemp(n) <= -1000)
        dato = sprintf('%.2f', coeficientesTemp(n));
    elseif (coeficientesTemp(n) >= 1000 || coeficientesTemp(n) <= -100)
        dato = sprintf('%.3f', coeficientesTemp(n));
    elseif (coeficientesTemp(n) >= 100 || coeficientesTemp(n) <= -10)
        dato = sprintf('%.4f', coeficientesTemp(n));
    elseif (coeficientesTemp(n) >= 10 || coeficientesTemp(n) < 0)
        dato = sprintf('%.5f', coeficientesTemp(n));
    else
        dato = sprintf('%.6f', coeficientesTemp(n));
    end
    dato
end
