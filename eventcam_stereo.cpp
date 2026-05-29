// Autor: Moacir Wendhausen    
// Projeto: VORIS
// Data: 10/01/2026
// Controle aquisição: sistema estéreo baseado em duas câmeras de eventos SylkiEvCam da Century Arks.
// Existe também a opção de trabalhar no modo single câmera, onde apenas a câmera master é utilizada, e a slave é desativada.
// Este  controle stereo single é efetuado a través da variável "consta int numCams", onde numCams=2 ativa o modo estéreo e numCams=1 ativa o modo mono.
// Programa principal que controla a captura de dados das câmeras de eventos e convencionais, bem como o controle do trigger por hardware e do LED de iluminação.
// Apresenta um menu interativo no terminal para controle das opções baseado nas libs do OpenCV, e um dashboard visual, também OpenCV, para exibir os frames 
// capturados e o menu de controle em tempo real.
// São utilizados dosi canais de PWM da Jetson Orina Nano para controlar o blink do LED e a potência do LED.
// O trigger por hardware é gerado usando os GPIOs da Jetson Orin Nano, controlados via a interface Sysfs do Linux.
// Os principais parâmetros estão definidos no header "parametros.h", como os tempos de trigger, números de série das câmeras, configurações de PWM, entre outros.


#include <gpiod.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/viz.hpp>

#include <metavision/sdk/core/utils/cd_frame_generator.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/base/events/event_cd.h>

#include "Spinnaker.h"
#include "trata_convcam.h"
#include "trata_evcam.h"
#include "trata_IO.h"
#include "trata_parametros.h"


// Função que exibe o menu de opções:
void showMenu(int pinTrigger, int pinLed, int64_t duracao){     
    std::cout << ""<< std::endl;
    std::cout << "******************** Menu  ********************"<< std::endl;
    std::cout << " 1 - Ler biases da câmera" << std::endl;
    std::cout << " 2 - Gravar biases na câmera" << std::endl;   
    std::cout << " 3 - Trigger: iniciar gravacao de eventos (.raw)" << std::endl;       
    std::cout << " 4 - Start/Stop blink Led"<< std::endl;
    std::cout << " 5 - Captura imagem pela cam. convencinal"<< std::endl;    
    std::cout << " > - Incrementa pulso Led" << std::endl;
    std::cout << " < - Decrementa pulso Led" << std::endl;
    std::cout << " + - Incrementa potência Led" << std::endl;
    std::cout << " - - Decrementa potência Led" << std::endl;
    std::cout << " L - Limpa Tela" << std::endl;
    std::cout << " Q - Sair do programa "<< std::endl;
    std::cout << "**********************************************"<< std::endl;
    std::cout << " Digite a opção: ";
    std::cout << std::endl;
}


// Simples rotina para limpar a tela:
void limparTela() {
    // \033[2J: Limpa a tela inteira
    // \033[H: Move o cursor para a posição inicial (canto superior esquerdo)
    std::cout << "\033[2J\033[H" << std::flush;
}


// Função que gera trem de N pulsos, onde N é definido por numPulse:
void pulseTrigger(int numPulse, gpiod_line *line, int pin, int64_t duracaoPulso){
    std::cout << " Gerando pulso de: " << duracaoPulso << "ms no pino: "<< pin << std::endl;

    for (int ctPulse=0; ctPulse<numPulse; ctPulse++){
        gpiod_line_set_value(line, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(duracaoPulso));
        gpiod_line_set_value(line, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(duracaoPulso));
    }
    std::cout << " Pulso finalizado." << std::endl;
}



// 
void saveData_Stereo_TriggerHW(EventCamera &cam_01 , EventCamera &cam_02, gpiod_line *line, const PARAMETROS_GERAIS &params) {
    try {
        // Obter data para o nome da pasta e do arquivo:
        auto agora = std::chrono::system_clock::now();
        auto tempo_t = std::chrono::system_clock::to_time_t(agora);
        struct tm *info = std::localtime(&tempo_t);

        std::stringstream Path_01; 
        Path_01<< "../out/data_evecam_" << std::put_time(info, "%d_%m_%Y") << "/L";
        std::stringstream Path_02;
        Path_02<< "../out/data_evecam_" << std::put_time(info, "%d_%m_%Y") << "/R";

        // Cria as pastas se elas não existirem:
        if (!std::filesystem::exists(Path_01.str())) {
            std::filesystem::create_directories(Path_01.str());
        }
        if (!std::filesystem::exists(Path_02.str())) {
            std::filesystem::create_directories(Path_02.str());
        }

        // Gera os nomes dos arquivos de dados .raw:
        std::string serialNumber_01= cam_01.getSerial();
        std::string serialNumber_02= cam_02.getSerial();
        std::stringstream ss_time;
        ss_time << std::put_time(info, "%H%M%S");
        std::string time= ss_time.str();
        std::string filename_01= "evecam_sn_" + serialNumber_01 + "_" + time + ".raw";
        std::string filename_02= "evecam_sn_" + serialNumber_02 + "_" + time + ".raw";
        std::string full_path_01 = Path_01.str()  + "/" + filename_01;
        std::string full_path_02 = Path_02.str()  + "/" + filename_02;

        // Dispara a gravação dos eventos nas duas câmeras:
        bool ok_01 = cam_01.startRecording(full_path_01);   
        bool ok_02 = cam_02.startRecording(full_path_02);

        // Inicia Gravação doarquivo de dados:
        if (ok_01 && ok_02){
            std::cout << "Salvando dados ....." << std::endl;

            // Pré-trigger: aguarda um tempo em microsegundos definido em duracao_PosTrigger_microSeg para garantir que o arquivo foi aberto e o buffer inicializou:
            std::this_thread::sleep_for(std::chrono::microseconds(params.duracao_pre_trigger));
            
            // *******************Início do pulso de trigger**************** 
            // Transição do trigger para nível alto:
            gpiod_line_set_value(line, 1);  

            // Mantém o pulso em nivel alto pelo tempo especificado em "duracao_pulso_trigger_microSeg":
            std::this_thread::sleep_for(std::chrono::microseconds(params.duracao_pulso_trigger));

            // Transição do trigger para nível alto:
            gpiod_line_set_value(line, 0);
            // ********************Fim do pulso de trigger******************

            // Pós-trigger: aguarda um tempo em microsegundos definido em duracao_PosTrigger_microSeg antes de fecahr o arquivo de dados: 
            std::this_thread::sleep_for(std::chrono::microseconds(params.duracao_pos_trigger));

            // Finaliza a Gravação e fecha arquivo de dados:
            if (cam_01.stopRecording() && cam_02.stopRecording()){
                std::cout << "Dados salvos no arquivo: " << "\"" << full_path_01 << "\"" << std::endl;
                std::cout << "Dados salvos no arquivo: " << "\"" << full_path_02 << "\"" << std::endl;
            }
            else{
                std::cout << "!!!ERRO ao fechar arquivo:" << std::endl;
            }
        }

    } catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha na thread de captura: " << e.what() << std::endl;
    }
}


// 
void saveData_Mono_TriggerHW(EventCamera &cam, gpiod_line *line, const PARAMETROS_GERAIS &params) {
    try {
        // Obter data para o nome da pasta e do arquivo:
        auto agora = std::chrono::system_clock::now();
        auto tempo_t = std::chrono::system_clock::to_time_t(agora);
        struct tm *info = std::localtime(&tempo_t);

        // gera o nome do diretório:
        std::stringstream ss_pasta;
        ss_pasta << "out/data_evecam_" << std::put_time(info, "%d_%m_%Y");
        std::string nome_pasta = ss_pasta.str();

        // Criar a pasta se ela não existir
        if (!std::filesystem::exists(nome_pasta)) {
            std::filesystem::create_directory(nome_pasta);
        }

        // Gera o nome do arquivo de dados .raw:
        std::string serialNumber= cam.getSerial();
        std::stringstream ss_time;
        ss_time << std::put_time(info, "%H%M%S");
        std::string time= ss_time.str();
        std::string filename= "evecam_sn" + serialNumber + "_" + time + ".raw";
        std::string full_path= nome_pasta + filename;

        // Inicia Gravação doarquivo de dados:
        if (cam.startRecording(full_path)){
            std::cout << "Salvando dados ....." << std::endl;

            // Pré-trigger: aguarda um tempo em microsegundos definido em duracao_PosTrigger_microSeg para garantir que o arquivo foi aberto e o buffer inicializou:
            std::this_thread::sleep_for(std::chrono::microseconds(params.duracao_pre_trigger));
            

            // *******************Início do pulso de trigger**************** 
            // Transição do trigger para nível alto:
            gpiod_line_set_value(line, 1);  

            // Mantém o pulso em nivel alto pelo tempo especificado em "duracao_pulso_trigger_microSeg":
            std::this_thread::sleep_for(std::chrono::microseconds(params.duracao_pulso_trigger));

            // Transição do trigger para nível alto:
            gpiod_line_set_value(line, 0);
            // ********************Fim do pulso de trigger******************

            // Pós-trigger: aguarda um tempo em microsegundos definido em duracao_PosTrigger_microSeg antes de fecahr o arquivo de dados: 
            std::this_thread::sleep_for(std::chrono::microseconds(params.duracao_pos_trigger));

            // Finaliza a Gravação e fecha arquivo de dados:
            if (cam.stopRecording())
                std::cout << "Dados salvos no arquivo: " << "\"" << full_path << "\"" << std::endl;
            else
                std::cout << "!!!ERRO ao fechar arquivo:" << std::endl;
        }

    } catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha na thread de captura: " << e.what() << std::endl;
    }
}


bool ativaLedLight(LightController& led, PWM& pwm_A, PWM& pwm_B){
    if (!pwm_A.getStatus()){
        if (!pwm_A.enable())           {
            std::cout<<" [Error]: não foi possível ativar o PWM blink led"<< std::endl;
            led.setRunning(false);
            return false;
        }    
    }

    if (!pwm_B.getStatus()){
        if (!pwm_B.enable()){
            std::cout<<" [Error]: não foi possível ativar o PWM que controla a tensão do led." << std::endl;
            led.setRunning(false);
            return false;
        }    
    }

    led.setRunning(true);
    return true;
}



void desativaLedLight(LightController& led, PWM& pwm_A, PWM& pwm_B){
    if (pwm_A.getStatus()){
        pwm_A.disable();
       // std::cout<<" PWM blink: Desativado."<< std::endl; 
    }
    else
        std::cout<<" PWM blink já está desativado." << std::endl;

    if (pwm_B.getStatus()){
        pwm_B.disable();
       // std::cout<<" PWM voltage: Desativado."<< std::endl; 
    }
    else
        std::cout<<" PWM voltage já está desativado." << std::endl;        
    led.setRunning(false); 
}


// Incrementa o valor do PWM dado em percentual, o parâmetro "use_led_potencia" é usado para limitar o duty-cycle em 10% 
// caso esteja sendo usado o LED de potencia LT2PR, para evitar danos ao led:
void incrementaPWM(PWM& pwm, const std::string funcao_PWM, bool use_led_potencia){
    if (use_led_potencia){
        int passo= 1;
        long dutyCycle= pwm.getDutyCycle();
        // Limita o duty-cycle em 10% caso esteja sendo usado o LED de potencia LT2PR:
        if (dutyCycle<=(10-passo)){
            dutyCycle += passo;                    
            pwm.setDutyCycle(dutyCycle);
        }  
        else
            std::cout<< "[Led LT2PR] Duty-cycle atingiu o valor máximo de 10." <<std::endl;
    }    
    else{
        int passo= 1;
        long dutyCycle= pwm.getDutyCycle();

        if (dutyCycle <= (100-passo)){
            dutyCycle += passo;
            pwm.setDutyCycle(dutyCycle);
        }               
        else
            std::cout<< "Duty-Cycle= 100%)"; 
    }

}


// Decrementa o valor do PWM dado em percentual:
void decrementaPWM(PWM& pwm, const std::string funcao_PWM){ 
    int passo= 2;
    long dutyCycle= pwm.getDutyCycle();

    if (dutyCycle>= passo){
        dutyCycle -= passo;
        pwm.setDutyCycle(dutyCycle);
    } 
    else if (dutyCycle==1){
        dutyCycle= 0;
        pwm.setDutyCycle(dutyCycle);
    }
    else
        std::cout<< "Duty-Cycle= 0%)";                               

}


// Verificação se os pinos forma configurados ok, por segurança:
int confirma_gpios_actives(GPIO_Lines &gpios_actives, gpiod_chip *chip){
    if (gpios_actives.triggerEventCam == nullptr || gpios_actives.piscaLed == nullptr || gpios_actives.triggerNormalCam == nullptr || 
        gpios_actives.controlMotor01 == nullptr  || gpios_actives.controlMotor02 == nullptr ) {
            std::cerr << "[ERRO] Falha crítica na inicialização dos GPIOs. Abortando!!!!" << std::endl;
        if (chip)
            gpiod_chip_close(chip);
        return 0;
    }
    return 1;
}

// Esa fuanção é executada antes de tentar instanciar uma camera de eventos, para verificar se há câmeras 
// conectadas no barramento USB, evitando erros de conexão ou falhas de hardware.
// Caso não sejam detectadas cameras, o progrma é abortado.
bool detectaCamerasConectadas(){
    try{
        // Tenta capturar a lista de seriais de todas as câmeras conectadas:
        Metavision::DeviceDiscovery::SerialList dispositivos= Metavision::DeviceDiscovery::list();
        if (!dispositivos.empty()){
            int numDevices= dispositivos.size();
            std::cout <<  std::endl;            
            std::cout << "Camera(s) de eventos detectadas): "<< numDevices << std::endl;
            for (int ct=0; ct<numDevices; ct++){
                auto it = std::next(dispositivos.begin(), ct); 
                // Exibe apenas os 8 primeiros caracteres do número de série, que são os mais relevantes para identificação. O restante é sufixo e pode variar entre modelos ou versões.
                std::cout << " - " << it->substr(0, 11) << " | Nº Série: " << it->substr(32, 40) << std::endl;
            }
            std::cout << std::endl; 
            return true;
        }
        else{
            std::cerr << "Nenhuma câmera de eventos detectada no barramento USB." << std::endl; 
            return false;
        }
    }
    catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha ao escanear barramento USB." << e.what() << std::endl;
        return false;
    }    
}


// Este menu é exibido no terminal do OpenCV junamente com os frames capturados pela câmera convencional, 
// para facilitar a visualização e controle das opções de trigger e configuração do LED. 
// Ele é desenhado diretamente sobre o frame da câmera convencional, utilizando as funções de desenho do OpenCV, 
// como putText e circle. O menu inclui as opções de controle, bem como um indicador "LIVE" que pisca para mostrar que a captura está ativa. 
// A função MenuFrame é chamada a cada frame capturado pela câmera convencional para atualizar o menu em tempo real.
void MenuFrame(const cv::Mat& input, cv::Mat& output) {
    if (input.empty()) return;

    int largura_menu = 300;

    // 1. Garante que o input seja convertido para 3 canais (Colorido) 
    // para podermos desenhar em Verde/Vermelho.
    cv::Mat input_color;
    if (input.channels() == 1) {
        cv::cvtColor(input, input_color, cv::COLOR_GRAY2BGR);
    } else {
        input_color = input;
    }

    // 2. Cria a moldura com a borda preta à direita
    cv::copyMakeBorder(input_color, output, 0, 0, 0, largura_menu, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    // 3. Configurações de texto
    int x_start = input_color.cols + 20;
    int y_start = 40;
    int espacamento = 30;

    std::vector<std::string> itens = {
        "*** MENU DE CONTROLE ***",
        "1 - Ler biases",
        "2 - Gravar biases",
        "3 - Trigger (Gravacao)",
        "4 - Start/Stop Blink LED",
        "5 - Captura Convencional",
        "-----------------------",
        "+ / - : Potencia LED",
        "> / < : Duracao Pulso",
        "L : Limpa Tela",
        "Q : Sair"
    };

    // 4. Desenha o texto
    for (size_t i = 0; i < itens.size(); ++i) {
        cv::putText(output, 
                    itens[i], 
                    cv::Point(x_start, y_start + (int)(i * espacamento)), 
                    cv::FONT_HERSHEY_SIMPLEX, 
                    0.5, 
                    cv::Scalar(0, 255, 0), // Verde
                    1, 
                    cv::LINE_AA);
    }

    // 5. Indicador "LIVE" (Círculo vermelho piscando no canto superior direito)
    // Usamos o tempo do sistema para fazer piscar
    auto milis = std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::system_clock::now().time_since_epoch()).count();
    
    if ((milis / 500) % 2 == 0) { // Pisca a cada 500ms
        cv::circle(output, cv::Point(output.cols - 20, 20), 7, cv::Scalar(0, 0, 255), -1);
        cv::putText(output, "LIVE", cv::Point(output.cols - 60, 25), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
    }
}



// Esta função é responsável por preparar a infraestrutura visual do sistema, Menu de Usuário" e as 
// janelas de exibição das câmaras de evntos. Ela aloca dinamicamente os acumuladores de eventos, chamdos Generators,
// define os parâmetros temporais de integração dos dados assíncronos, configura o frame de exibição e regista 
// as rotinas, callbacks, que capturam os fluxos de dados de eventos brutos vindos do hardware via USB.
// Parâmetros de entrada:
//      - paramVis: Referência para a estrutura de contexto global da aplicação, armazena matrizes, mutexes e flags.
//      - event_cam: Ponteiro ou array contendo as instâncias de hardware das câmaras Metavision, Master e Slave.
//      - numCams: Inteiro que define o número de câmaras ativas no sistema, 1 para Monocular, 2 para Estéreo.
//      - cam_w: Largura nativa da resolução do sensor de eventos, em pixels.
//      - cam_h: Altura nativa da resolução do sensor de eventos, em pixels.
//
// Ela captura uma amostragem discreta sobre um fluxo contínuo. Os eventos são acumulados em buffers temporais 
// antes de serem convertidos em matrizes bidimensionais, cv::Mat, compatíveis com o OpenCV..
// Ela inicializa os Canvas para cada câmera e para o menu, configura os geradores de frames para cada câmera de eventos.
void configuraVisualizer(parametrosFrameGenerator &ctx, EventCamera event_cam[], int numCams, int cam_w, int cam_h) {
    int largura_canvas_stereo = (cam_w * numCams);

    //Aloca a memória para o canvas estéreo a qual o loop while usa para executar o cv::imshow():
    ctx.canvas_stereo = cv::Mat::zeros(cam_h, largura_canvas_stereo, CV_8UC3);
    ctx.canvas_cam_L = ctx.canvas_stereo(cv::Rect(0, 0, cam_w, cam_h));

    // Só ativa o canvas para visualizar a visão estérreo apensa se numCams=2:
    if (numCams==2) {
        ctx.canvas_cam_R = ctx.canvas_stereo(cv::Rect(cam_w, 0, cam_w, cam_h));
    } 

    // Sempre ativa o canvas do menu com opçoes de escolhas para o usuario:
    ctx.canvas_menu = cv::Mat::zeros(ctx.altura_canvas_menu, ctx.largura_canvas_menu, CV_8UC3);

    // Os "generators", especificamente a classe Metavision::CDFrameGenerator, são acumuladores e tradutores de dados bidimensionais.
    // A função  deles é atuar como uma ponte entre o hardware e o olho humano. 
    // A função dles é pegar o fluxo contínuo de dados de eventos puros, assíncronos, e acumular dentro de uma janela de tempo fixa, 
    // neste caso, 10000us, 10ms. Com esses os agrupados, eles geram uma matriz de imagem convencional a uma taxa fixa, por exemplo 30Hz,
    // transformando dados binários  em frames visíveis pelo OpenCV.
    // Limpar os geradores antigos para evitar lixo de execuções anteriores:
    ctx.generators.clear();

    // Configura Câmera 1, esquerda:
    ctx.generators.push_back(std::make_unique<Metavision::CDFrameGenerator>(cam_w, cam_h));
    ctx.generators[0]->set_display_accumulation_time_us(ctx.tam_buff_uSeg);
    ctx.generators[0]->set_color_palette(Metavision::ColorPalette::Dark);

    event_cam[0].setCDCallback([&ctx](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
                                    if (ctx.showViewer && !ctx.generators.empty()) {
                                    ctx.generators[0]->add_events(ev_begin, ev_end);
                                    }
    });

    // Configura Câmera 2, direita, apenas se numCams == 2:
    if (numCams == 2) {
        ctx.generators.push_back(std::make_unique<Metavision::CDFrameGenerator>(cam_w, cam_h));
        ctx.generators[1]->set_display_accumulation_time_us(ctx.tam_buff_uSeg);
        ctx.generators[1]->set_color_palette(Metavision::ColorPalette::Dark);

        event_cam[1].setCDCallback([&ctx](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
            if (ctx.showViewer && ctx.generators.size() > 1) {
                ctx.generators[1]->add_events(ev_begin, ev_end);
            }
        });
    }


    // Desenha o Menu Estático
    std::vector<std::string> itens = {"-----------------------", 
                                      "1 - Ler Biases", 
                                      "2 - Gravar Biases", 
                                      "3 - Trigger REC", 
                                      "4 - Blink LED", 
                                      "5 - Cam. Convencional", 
                                      "6 - Load eventos de arquivo .raw",
                                      "7 - Ligar/Desligar exibicao eventos",
                                      " ",
                                      "+ / - : Varia potencia LED", 
                                      "> / < : Varia Duracao Pulso",
                                      "-----------------------",                                       
                                      " ", 
                                      "Q - Sair"};
    
    // Configura a cor de fundo do canvas do menu:
    ctx.canvas_menu.setTo(cv::Scalar(220, 220, 220));

    // Configura o canvas do menu:
    for (size_t i = 0; i < itens.size(); ++i) {
        if (i<13)
            cv::putText(ctx.canvas_menu, itens[i], cv::Point(20, 70 + i*40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        else
            cv::putText(ctx.canvas_menu, itens[i], cv::Point(20, 70 + i*40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }

    // Cria a janela em modo Autosize:
    cv::namedWindow("Menu de Usuario", cv::WINDOW_AUTOSIZE);    
}



// Metodo para carregar os eventos a aprtir de arquivo de dados do tipo .raw:
bool loadEventos(std::vector<Metavision::EventCD> &buff, std::string filePath){

    try {
        std::cout << "Abrindo arquivo de dados via stream offline: " << filePath << "..." << std::endl;
        
        // É Instanciada uma câmera "virtual" apontando para o arquivo .raw.
        // Esta técncia é necessa´ria porque o openeb não disponibilzar leitura direta, mesmo com decodificador, 
        // do arquivo de dados para tranferi-lo á um buffer, este artif[icio é necessário.
        // OU seja, ele trata o arquivo como se fosse um stream de dados.
        Metavision::Camera cam = Metavision::Camera::from_file(filePath);

        // Registra uam callback para capturar os dados de evenos decodificados.
        // À medida que a thread interna do SDK lê o arquivo, os ponteiros ev_begin e ev_end
        // delimitam o bloco de structs EventCD prontas que enviamos para o final do seu vetor principal.
        cam.cd().add_callback([&buff](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
            buff.insert(buff.end(), ev_begin, ev_end);
        });

        // Inicia a câmera, o que equivale a inicar o decodiifcador interno de stream do OpenEB, necessário, pois os eventos estão gairados 
        // em formato binários codificado:
        cam.start();
        std::cout << "Lendo arquivo e transferindo dados para o buffer..." << std::endl;

        // Loop síncrono que executa da função enquanto o arquivo está sendo decodificado e lido:
        while (cam.is_running()) {
            // Aplica-se um curto intervalo para não exigir 100% da CPU da Jetson com um laço vazio:
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        cam.stop();

        // --- Pronto! Todos os dados estão na variável 'buff' ---
        std::cout << "Leitura de arquivo para o buffer concluida. " << std::endl;
        std::cout << "Total de eventos: " << buff.size() << std::endl;
        if (!buff.empty()) {
            std::cout << "Primeiro timestamp: " << buff.front().t << " us" << std::endl;
            std::cout << "Último timestamp: "  << buff.back().t  << " us" << std::endl;
        }        

        return true;

    } catch (const std::exception &e) {
        std::cerr << "Erro ao processar o arquivo: " << e.what() << std::endl;
        return false;
    }
}



// Funcção principal:
int main(int argc, char *argv[]) {
    // Rotina que limpa o terminal:
    limparTela();

    // Define o nome e path do arquivo que será lido para análise:
    std::string eventFileName= "../out/data_evecam_27_05_2026/L/evecam_sn_00000680_133955.raw";

    // Carrgea os parametros gerais definidos na struct "PARAMETROS_GERAIS" do header "parametros.h":
    PARAMETROS_GERAIS parametros_gerais;


    //********************************************************************************************/
    //****************************** Configuração GPIO da Jetson *******************************/
    //********************************************************************************************/

    // Declara o ponteiro do chip aqui para poder ser fechado dentro do main:
    struct gpiod_chip *chip = nullptr;

    // Isntancia um objeto para acessar a cofniguração do GPIO da Jetson. A classe está declarada no header "controlJetson.h":
    configJetson configuraGPIO_Jetson;

    // Chama o método get() para capturar as informações dos pinos e lines "ativos" da Jetson:    
    auto activePins= configuraGPIO_Jetson.getPinos();
    
    // Separa as informações dos pinos por função:
    int pin_PiscaLed= activePins.header_pin_IO_I;
    int pin_TriggerEventCam= activePins.header_pin_IO_G;
    int pin_TriggerCam= activePins.header_pin_IO_H;
    //int pin_ControlLaser= activePins.header_pinH;

    // Chama o método de configuração "configuraGPIO_Jetson.configura_GPIO_Jetson(&chip)" para inicialização do barramento GPIO da Jetson.
    // Ela retorna uma struct contendo ponteiros com os endereços de cada linha do GPIO da Jetson para controle pelo Kernell.
    // Através desses endereços que os pinos de IO são diretamente manipulados, por exemplo, mudanças de nivel lógico.
    GPIO_Lines gpios_actives= configuraGPIO_Jetson.configura_GPIO_Jetson(&chip);

    // Por segurança, verifica se as linhas "gpios_actives" foram ativadas, configuradas ok, caso contrario o programa é abortado para evitar falhas de hardware:
    if (!confirma_gpios_actives(gpios_actives, chip))
        return 1; 


    // ***********************************************************************************************/
    // ******************************** Configuraçaõ do PWM ******************************************/
    // ***********************************************************************************************/    
    // Configuração do PWM direta via Sysfs (Linux Kernel), que utiliza a interface de arquivos virtuais do 
    // sistema operacional para manipular registradores de hardware nativos. Este método elimina 
    // dependências externas, garantindo que o sinal de PWM seja gerado de forma estável por hardware 
    // independente, sem sobrecarga da CPU.

    // Define se está usando o led de potencia LT2PR da Opto Engineering
    bool useLed_LT2PR= true;

    // Instanciando os objetos PWM com a classe PWM:
    PWM pwm_BlinkLed(parametros_gerais.periodo_PWM_A, parametros_gerais.dutyCycle_PWM_A, parametros_gerais.channelToExport_A, "Canal A");
    PWM pwm_PowerLed(parametros_gerais.periodo_PWM_B, parametros_gerais.dutyCycle_PWM_B, parametros_gerais.channelToExport_B, "Canal B");



    //********************************************************************************************/
    //********** Configuração da camera convencional FLIR (Model BlackFly BFS-U3-04S2M) **********/
    //********************************************************************************************/
    // Instancia um objeto do tipo COnvCamera: 
    ConvCamera* convCam_01 = nullptr;

    // Instanciar o Singleton: é uma instância única do sistema que gerencia a comunicação com o hardware para acessar a camera.
    // Carrega a Camada de Transporte: Inicializa os drivers e protocolos (USB3, GigE) necessários para detectar e conversar com as câmeras.
    // Ponto de Entrada: É o objeto obrigatório para listar dispositivos e gerenciar o ciclo de vida da SDK.
    Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();

    // Varredura das portas: Escaneia as interfaces (USB/GigE) em busca de câmeras conectadas.
    // Cria uma lista contendo referências (objetos CameraPtr) para todos os dispositivos encontrados.
    // Permite localizar uma câmera específica, por exemplo pelo nº de Serie para começar a operá-la.
    Spinnaker::CameraList camList = system->GetCameras();

    // Instancia o  objeto da camera convencional apenas se estiver habilitado este procedimento.
    if (parametros_gerais.useCamera_Conv){

        // Busca Seletiva: Percorre a lista de câmeras detectadas procurando o identificador único, Serial Number, definido nos parametros_gerais.
        // Verifica se a câmera desejada está conectada e disponível antes de iniciar a operação.
        Spinnaker::CameraPtr pCamBase = camList.GetBySerial(parametros_gerais.serialNumber_conv_cam_01);

        // Testa se a camera com o referido nº de séri foi encontrada:
        if (!pCamBase.IsValid()) {
            std::cerr << "Câmera não detectada!" << std::endl;
            return -1;
        }

        // Conversão de Tipo: Transforma o ponteiro genérico da SDK, Spinnaker::Camera*, no tipo definido pela minha classe ConvCamera: 
        // Permite acesso aos métodos e atributos personalizados especificos da classe ConvCamera e também da classe original da FLIR, que foi herdada.
        // Desta forma, a classe ConmvCamera terá acesso a toda as funções e metodos da classe Spinnaker::Camera:
        convCam_01 = static_cast<ConvCamera*>(pCamBase.get());
        
        // Se ok, será exibida a configuraçã da camera convencional 
        if (convCam_01){
            convCam_01->Init();
            std::cout<< std::endl;
            std::cout<< "*** Câmera convencional: ***"<< std::endl;
            convCam_01->exibir_modelo_camera();
            std::cout<< std::endl;
            convCam_01->DeInit();
        }
    }    



    //********************************************************************************************/
    //*************************** Configuração da camera de eventos ******************************/
    //********************************************************************************************/

    // Se não detectar camera de eventos no barramente USB, o programa é abortado para evitar falhas de hardware:
    if (!detectaCamerasConectadas()){
        return 1;
    }

    // Buffer que irá armazenar os eventos proveinetes do arquivo de dados .raw capturado pela camera de eventos:
    std::vector<Metavision::EventCD> buffEventos;

    // Instancia o objeto event_cam_xx para operar com a camera de eventos:
    // Esta "Classe EventCamera" foi criada para tratar dos objeos, atributops e métodos do SDK Metavision: 
    // O construtor recebe um parametro booleano do tipo "argumento padrão", só é passado quando for necessário,
    // Ou seja, quando a câmera é master, para configurar o sincronismo de hardware. 
    // Caso contrário, o valor padrão é "false", ou seja, a câmera é slave.
    // Define se o sistema sera mono ou estéreo:
    const int numCams= 1;


    /*
    // Instanciação dos objetos que controlam o hardware das câmeras de eventos:
    EventCamera event_cam[numCams] = {
            EventCamera(parametros_gerais.serialNumber_event_cam3, "Left", true),
            EventCamera(parametros_gerais.serialNumber_event_cam2, "Right", false)
    };
    */

    EventCamera event_cam[numCams] = { EventCamera(parametros_gerais.serialNumber_event_cam3, "Left", true)};    

    // Configura alguns parametros iniciais para cada câmera de eventos, como o bias, trigger e sincronismo de hardware, 
    // usando as funções da classe EventCamera, que por sua vez utilizam a API do SDK Metavision:
    for (int i=0; i<numCams; i++){
        // COnfigura, carrega, os biases na camera de eventos definidos em setings.json:
        if (!event_cam[i].setBias(parametros_gerais)){
            std::cerr << "Câmera de eventos[ " << i << "] não detectada!" << std::endl;
            return -1;
        }

        // Imporante!!! Esta função habilita o Trigger via HAL da API:
        event_cam[i].enableHardwareTrigger();

        // Esta função configura o sincronismo de tempo entre as cameras de eventos via HAL da API: 
        // onde a câmera master é a "Left" e a slave é a "Right":
        event_cam[i].configSincronismo();

        // Se numCams=2 a segunda camera será inicializada:
        if (i==1)
            event_cam[i].callStart();

    }

    // Captura as dimensões da camera de eventos para configurar a visualização no dashboard:
    int cam_w = event_cam[0].getWidth();
    int cam_h = event_cam[0].getHeight();

    // Instanci a struct que contem os parâmetros para configurar o construtor da visualização, 
    // que é o objeto responsável por gerenciar a visualização dos frames das câmeras de eventos e do menu no dashboard:
    parametrosFrameGenerator paramVis;
 
    // Inicializa e configura o pipeline de visualização gráfica para o sistema de câmaras de eventos:
    configuraVisualizer(paramVis, event_cam, numCams, cam_w, cam_h);

    // Dispara o funcionamenoe das câmeras de eventos.
    // A partir deste ponto, as câmeras de eventos começam a capturar e gerar frames, 
    // que são processados pelos geradores de frames e exibidos no dashboard em tempo real.
    // Para o sicnronismo de tempo funcionar adequadametnte, a contagem do laço for deve ser invertida 
    // porque a câmera SLAVE [1] deve ser iniciada antes da MASTER [0] para garantir que ela esteja pronta para r
    // eceber os sinais de sincronismo temporal:
    for (int i = numCams - 1; i >= 0; i--) {
        event_cam[i].callStart(); 
    }

    
    // Instacia objeto ledLight para controlar o estado do Led ou lase de luz estruturada:
    LightController ledLight;
    bool running = true;
    bool exibeStereo = false; // Controla se a janela estéreo está aberta ou fechada
        
    // Loop principal:    
    while(running) {

        int key = cv::waitKey(1);

        if (paramVis.showViewer) {
            // Menu de opções sempre visível:
            cv::imshow("Menu de Usuario", paramVis.canvas_menu);

            // Só exibirá os frames stereo com dados de eventos apenas se a opção "7" ativar "exibeStereo":
            if (exibeStereo) {
                std::unique_lock<std::mutex> lock(paramVis.mutex);

                // Atualiza e formata o frame esquerdo no canvas_stereo, canvas ROI da Esquerda
                if (!paramVis.frame_L.empty()) {
                    if (paramVis.frame_L.channels() == 1)
                        cv::cvtColor(paramVis.frame_L, paramVis.canvas_cam_L, cv::COLOR_GRAY2BGR);
                    else
                        paramVis.frame_L.copyTo(paramVis.canvas_cam_L);

                    cv::putText(paramVis.canvas_cam_L, "Left (Sync Master)", cv::Point(15, 30), 
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
                }

                // Atualiza e formata o frame direito no canvas_stereo, canvas ROI da Direita:
                if (!paramVis.frame_R.empty()) {
                    if (paramVis.frame_R.channels() == 1)
                        cv::cvtColor(paramVis.frame_R, paramVis.canvas_cam_R, cv::COLOR_GRAY2BGR);
                    else
                        paramVis.frame_R.copyTo(paramVis.canvas_cam_R);

                    cv::putText(paramVis.canvas_cam_R, "Right (Sync Slave)", cv::Point(15, 30), 
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
                }

                // Exibe a janela estéreo combinada:
                cv::imshow("Visualizador Estereo", paramVis.canvas_stereo);
            }            
        }

       
        // Máquina de estados do menu principal:
        if (key!=-1){
            char my_char= static_cast<char>(key);

            switch (my_char)
            {
                case '1':{
                        // Efetua a leitura dos biases da camera de eventos:
                        for (int i=0; i<numCams; i++){
                            event_cam[i].readCameraBiases();
                        }
                        break;
                }
                
                case '2':
                        // Chama função para configurar, enviar, os biases à camera de eventos:                    
                        for (int i=0; i<numCams; i++){
                            event_cam[i].readCameraBiases();
                        }
                        break;

                case '3':
                        {
                            // Usa-se uma thread com função lambda, onde:
                            // [&] = Captura: ela captura todas as variáveis e métodos, por referencia, no escopo de main(), pois ela utilzia várias funções e variáveis;
                            // () = Parâmetros: Sem nenhum parâmetro como argumento;
                            // {} = Corpo: onde são chamadas as funções.:
                            std::thread t([&]() { 
                                for (int ctCiclo=0;  ctCiclo < parametros_gerais.numero_ciclos_trigger; ctCiclo++){                           
                                    // Primeira ativa a projeção da luz estruturada:
                                    ativaLedLight(ledLight, pwm_BlinkLed, pwm_PowerLed);
                                    // Chama a gravação do dados da câmera de eventos, que é feita em paralelo, ou seja, enquanto a câmera de eventos está gravando os 
                                    // dados no arquivo .raw, o programa continua executando as próximas linhas de código, sem esperar a finalização da gravação.  
                                    if (numCams > 1) {
                                        saveData_Stereo_TriggerHW(event_cam[0], event_cam[1], gpios_actives.triggerEventCam, parametros_gerais);
                                    } else {
                                        saveData_Mono_TriggerHW(event_cam[0], gpios_actives.triggerEventCam, parametros_gerais);
                                    }
                                    // Por ultimo, desativa o projeção de luz estruturada:
                                    desativaLedLight(ledLight, pwm_BlinkLed, pwm_PowerLed);
                                }
                            });

                            // Desacoplar a thread para que o viewer não trave
                            t.detach();
                            break;
                        } 

                case '4':
                        {
                            if (!ledLight.getRunning())
                                ativaLedLight(ledLight, pwm_BlinkLed, pwm_PowerLed);
                            else
                                desativaLedLight(ledLight, pwm_BlinkLed, pwm_PowerLed);                                             
                            break;                        
                        }  
                        
                case '5':
                        {
                            // Captura uma imagem pela câmera convencional:
                            if (convCam_01){
                                convCam_01->Init();
                                convCam_01->capturarImagem();
                                convCam_01->DeInit();
                            }
                            else
                                std::cout << "Câmera convencional não instanciada." << std::endl;
                            break;                        
                        }  
                        
                case '6':{
                    //std::string eventFileName= "../out/data_evecam_27_05_2026/L/evecam_sn_00000680_133955.raw";
                    if (loadEventos(buffEventos, eventFileName)){
                        std::cout<< "Eventos carregados no buffer." << std::endl;
                    }
                    break;
                } 
                

                case '7': {
                    // Togle, inverte o estado:
                    exibeStereo = !exibeStereo; 

                    if (exibeStereo) {
                        // Cria a janela do OpenCV dedicada para os dados de eventios 
                        cv::namedWindow("Visualizador Estereo", cv::WINDOW_AUTOSIZE);
                        
                        // Ativa as threads de renderização em 30Hz, este valor esta definido em "paramVis.taxa_geracao_frames":
                        if (!paramVis.generators.empty()) {
                            paramVis.generators[0]->start(paramVis.taxa_geracao_frames, [&paramVis](const Metavision::timestamp &ts, const cv::Mat &frame) {
                                if (paramVis.showViewer) {
                                    std::unique_lock<std::mutex> lock(paramVis.mutex);
                                    frame.copyTo(paramVis.frame_L);
                                }
                            });
                            
                            if (paramVis.generators.size() > 1) {
                                paramVis.generators[1]->start(paramVis.taxa_geracao_frames, [&paramVis](const Metavision::timestamp &ts, const cv::Mat &frame) {
                                    if (paramVis.showViewer) {
                                        std::unique_lock<std::mutex> lock(paramVis.mutex);
                                        frame.copyTo(paramVis.frame_R);
                                    }
                                });
                            }
                        }
                    } 
                    else {
                        // Desativa as threads:
                        for (auto& gen : paramVis.generators) {
                            if (gen) gen->stop();
                        }
                        
                        // Destrói a janela de vídeo e limpa a tela do Ubuntu
                        cv::destroyWindow("Visualizador Estereo");
                        
                        // Limpa os buffers antigos para não iniciar com imagem congaleda da próxima ativação:
                        paramVis.frame_L = cv::Mat();
                        paramVis.frame_R = cv::Mat();
                    }
                    break;
                }                
                
                case '.':
                case '>': // Incrementa o pwm que controla o tempo de atuação do Led, duração do blink do Led
                        incrementaPWM(pwm_BlinkLed, "Blink Led", parametros_gerais.useLed_LT2PR);
                        break;    

                case ',':    
                case '<': // Decrementa o pwm que controla o tempo de atuação do Led, duração do blink do Led
                        decrementaPWM(pwm_BlinkLed, "Blink Led");
                        break;                    

                case '+':
                case '=': // Incrementa o pwm que controla a tensão analogica do Led (0 a 10V) ou laser (o a 5V):
                        incrementaPWM(pwm_PowerLed, "Tensão do Led", false);                                 
                        break;

                case '-':
                case '_': // Decremento o pwm que controla a tensão analogica do Led (0 a 10V) ou laser (o a 5V):
                        decrementaPWM(pwm_PowerLed, "Tensão do Led");
                        break;                      

                case 'l':
                case 'L':
                        // Chama função para limpar o terminal: 
                        limparTela();
                        parametros_gerais.hab_exibe_menu= true;
                        break;

                case 27 : // Saida do programa com Esc, corresponde ao cod. 27, "q" ou "Q":
                case 'q':
                case 'Q':
                        running = false;
                        std::cout <<std::endl;
                        break;

                default:
                        std::cout << "Comando invalido!" << std::endl;
                        break;
            }
        }
    }

    // Fecha a câmera:  
    for (int i=0; i<numCams; i++){
        event_cam[i].stop();
    }     


    // Para o gerador de frame:
    for (auto& gen : paramVis.generators) {
        if (gen) {
            gen->stop();
        }
    }

    // Fecha todas as janelas:    
    cv::destroyAllWindows();

    // Desabilita o pwm
    pwm_BlinkLed.disable();

    return 0;
}