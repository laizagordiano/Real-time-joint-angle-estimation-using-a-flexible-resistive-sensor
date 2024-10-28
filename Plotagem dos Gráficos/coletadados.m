% Nome do dispositivo serial
serialPort = "/dev/ttyACM0";  % Substitua com o nome correto da porta serial

% Configure a frequência de leitura em Hz
freq = 10; % 10 leituras por segundo
interval = 1 / freq; % Intervalo entre leituras em segundos

try
    % Abra o arquivo do dispositivo serial para leitura
    fid = fopen(serialPort, 'r');
    
    if fid == -1
        error("Não foi possível abrir a porta serial.");
    end
    
    % Nome do arquivo para salvar os dados recebidos
    nomeArquivo = 'dados_stm32.txt';
    arquivo = fopen(nomeArquivo, 'w');
    
    if arquivo == -1
        fclose(fid); % Fechar a porta serial se não conseguir abrir o arquivo
        error("Não foi possível abrir o arquivo para escrita.");
    end
    
    disp("Iniciando a leitura em tempo real...");
    
    % Leitura e escrita de dados em tempo real
    while true
        % Ler a linha de dados da porta serial
        data = fgetl(fid);
        
        % Verificar se a leitura foi bem-sucedida
        if ischar(data)
            % Mostrar os dados recebidos
            disp(data);
            % Escrever os dados no arquivo
            fprintf(arquivo, '%s\n', data);
        end
        
        % Pausa para manter a frequência de leitura
        pause(interval);
    end
    
catch err
    disp("Erro ao ler/escrever na porta serial: " + err.message);
    
    % Fechar os arquivos em caso de erro
    if exist('fid', 'var') && fid ~= -1
        fclose(fid);
    end
    
    if exist('arquivo', 'var') && arquivo ~= -1
        fclose(arquivo);
    end
end

% Fechar o arquivo da porta serial e o arquivo salvo
if exist('fid', 'var') && fid ~= -1
    fclose(fid);
end

if exist('arquivo', 'var') && arquivo ~= -1
    fclose(arquivo);
end

disp("Dados recebidos do STM32 foram salvos em '" + nomeArquivo + "'.");
