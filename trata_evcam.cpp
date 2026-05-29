// Autor: Moacir Wendhausen
// Projeto: VORIS
// Data: 21/01/2026
// Função: Este arquivo contém a implementação da classe EventCamera, que é responsável por controlar a câmera de eventos usando o SDK Metavision da Prophesee. A classe EventCamera herda todos
// os métodos e atributos da classe Metavision::Camera, e sobrescreve os métodos de inicialização, configuração de biases, configuração de trigger por hardware, entre outros, para adaptar às 
// necessidades do projeto.


#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <utility>

#include <metavision/sdk/stream/camera.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/facilities/i_ll_biases.h>
#include <metavision/hal/facilities/i_camera_synchronization.h>

#include "trata_evcam.h"
#include "trata_parametros.h"

using json = nlohmann::json;

// O construtor da classe camera_conv.
// Se a câmera for encontrada e aberta com sucesso, o objeto cam é inicializado com a câmera correspondente ao serial number fornecido.
// Isto singifica que após gerar o objeto cam, a câmera de eventos já está aberta e pronta para uso.
// Estabeleceu um canal de comunicação: agora o objeto cam é a interface para interagir com a câmera, ler eventos, configurar parâmetros, etc.
// Hardware incializado: agora é possível acessar as funcionalidades do device câmera através do objeto cam.
EventCamera::EventCamera(std::string serial, std::string nm, bool master) 
: Metavision::Camera(), // Inicia a base vazia/não inicializada
  serialNumber(serial),
  isMaster(master)
{
    // Tenta mover a câmera aberta para a base desta instância
    // Cast de 'this' para a classe base para usar o operator= (move assignment)
    static_cast<Metavision::Camera&>(*this) = Metavision::Camera::from_serial(serialNumber);
    this->width = this->geometry().get_width();
    this->height = this->geometry().get_height();
    this->inicializada = true;
    this->nome = nm;
    this->is_running = false;
}


// Da um start na camera, a apritr deste momento ele irá enviar dados de eventos:
void EventCamera::callStart(){
    // Antes de dar um start() verificar se ela já nçao está rodando, para evitar erros de start() em uma câmera já em execução:
    if (!this->is_running){
         if (this->start()){
            this->is_running = true;
            std::cout << "Câmera " << nome << ":........ Inicializada" << std::endl;
        }
        else{
            std::cout << "ERRO ao iniciar a câmera de eventos: " << nome << std::endl;
        }
    }
    else{
         std::cout << "Camera nome de ventos: " << nome << " - Nº serie: " << serialNumber << " já está em execução." << std::endl;
    }
}


/*
bool EventCamera::openEventCam() {
try {
        // Tentamos instanciar a câmera e mover para o nosso ponteiro
        cam = std::make_unique<Metavision::Camera>(Metavision::Camera::from_serial(serialNumber));
        
        if (cam) {
            inicializada = true;
            std::cout << "Hardware inicializado com sucesso via serial: " << serialNumber << std::endl;
            return true;
        }
    } 
    catch (const Metavision::CameraException &e) {
        std::cerr << "Erro ao abrir a camera: " << e.what() << std::endl;
        
        // Sua lógica de DeviceDiscovery permanece exatamente igual
        try {
            Metavision::DeviceDiscovery::SerialList dispositivos = Metavision::DeviceDiscovery::list();
            if (!dispositivos.empty()) {
                std::cout << "Cameras conectadas no barramento:" << std::endl;
                for (size_t ct = 0; ct < dispositivos.size(); ct++) {
                    auto it = std::next(dispositivos.begin(), ct);
                    std::cout << "Câmera [" << ct << "] - Serial: " << *it << std::endl;
                }
            } else {
                std::cerr << "Nenhuma câmera detectada no barramento USB." << std::endl;
            }
        } catch (...) {
            std::cerr << "[ERRO] Falha crítica ao escanear USB." << std::endl;
        }
        return false;
    }
    return false;
}
*/



void EventCamera::getParametrosGeraisEventCam(){
    // Captura dados da câmera instanciada:
    try {
        std::cout << "*** Camera de eventos ***"  << std::endl; 
        parametrosGerais.fabricante = this->get_camera_configuration().integrator;

        std::cout << "Plugin versão........: " << parametrosGerais.fabricante << std::endl; 

        parametrosGerais.plugin = this->get_camera_configuration().plugin_name;
        std::cout << "Plugin versão........: " << parametrosGerais.plugin << std::endl; 

        parametrosGerais.serial_cam = this->get_camera_configuration().serial_number;
        std::cout << "Nº Serial cam........: " << parametrosGerais.serial_cam << std::endl;

        parametrosGerais.versionFirm = this->get_camera_configuration().firmware_version;
        std::cout << "Versão do firmware...: " << parametrosGerais.versionFirm << std::endl;

        parametrosGerais.dataEncodeFormat = this->get_camera_configuration().data_encoding_format;
        std::cout << "Formato dos dados....: " << parametrosGerais.dataEncodeFormat << std::endl;
        std::cout << std::endl;
    } 
    catch (...) {
        std::cout << "Nao foi possivel obter o serial via CameraConfiguration." << std::endl;
    }

}

// Função chamada para a leitura dois Biases da câmera de eventos:
void EventCamera::readCameraBiases() {
    try {
        // Acessar via HAL explicitamente:
        auto *biases = this->get_device().get_facility<Metavision::I_LL_Biases>();

        if (biases) {
            std::cout << "\nBases Atual da câmera: "<< this->nome << std::endl;
            
            // Lista de biases comuns no IMX636 para verificar manualmente
            std::vector<std::string> bias_names = {
                 "bias_diff", "bias_diff_on", "bias_diff_off", "bias_fo", "bias_hpf", "bias_refr"
            };

            for (const auto& name : bias_names) {
                try {
                    int val = biases->get(name);
                    std::cout << " Bias: " << std::left << std::setw(15) << name 
                              << " | Valor: " << val << std::endl;
                } catch (...) {
                    // Se um nome específico não existir neste modelo, ignore
                }
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "Erro: " << e.what() << std::endl;
    }    
}


// Função que busca os valores dos biases no arquivo .json com as confuguraçloes da câmera:
int lerJsonFile(std::string path, std::string biasName){
    std::ifstream file(path);
    if (!file.is_open()) 
        return -1;

    json data;
    file >> data;

    // Navega na estrutura: ll_biases_state -> bias (que é um array)
    auto biases = data["ll_biases_state"]["bias"];

    for (auto& item : biases) {
        if (item["name"] == biasName) {
            return item["value"];
        }
    }
    std::cout << "Erro!!! Campo de leitura: " << biasName << " não encontrado no arquivo: "<< path << std::endl;
    return -1; // Não encontrado
}



bool EventCamera::setBias(PARAMETROS_GERAIS &params) {

    // Os biases max e min. são definidos em https://docs.prophesee.ai/stable/hw/manuals/biases.html 
    // Os calores para a ca~mera SilkyEvCam pertencem a geração Gen3.1 VGA, assim os valores máximos e mínimo
  
    // ANtes de gravar os valores de biases na camera verifica se os valores extrapolam os limites:
    int bias_diff_off= lerJsonFile(fullPath, "bias_diff_off");
    if (bias_diff_off<params.bias_diff_off_min || bias_diff_off>params.bias_diff_off_max){
        std::cout<< "ERRO!! O valor de bias_diff_off= "<< bias_diff_off << " está fora dos limites!!"<< " Ele deve ser entre=" << params.bias_diff_off_min << " e " << params.bias_diff_off_max << std::endl;  
        return false;
    }

    int bias_diff_on= lerJsonFile(fullPath, "bias_diff_on");
    if (bias_diff_on<params.bias_diff_on_min || bias_diff_on>params.bias_diff_on_max){
        std::cout<< "ERRO!! O valor de bias_diff_on= "<< bias_diff_on << " está fora dos limites!!"<< " Ele deve ser entre=" << params.bias_diff_on_min << " e " << params.bias_diff_on_max << std::endl;  
        return false;
    }

    int bias_fo= lerJsonFile(fullPath, "bias_fo");
    if (bias_fo<params.bias_fo_min || bias_fo>params.bias_fo_max){
        std::cout<< "ERRO!! O valor de bias_fo= "<< bias_fo << " está fora dos limites!!"<< " Ele deve ser entre=" << params.bias_fo_min << " e " << params.bias_fo_max << std::endl;  
        return false;
    }   
    
    int bias_hpf= lerJsonFile(fullPath, "bias_hpf");
    if (bias_hpf<params.bias_hpf_min || bias_hpf>params.bias_hpf_max){
        std::cout<< "ERRO!! O valor de bias_hpf= "<< bias_hpf << " está fora dos limites!!"<< " Ele deve ser entre=" << params.bias_hpf_min << " e " << params.bias_hpf_max << std::endl;
        return false;  
    }   

    int bias_refr= lerJsonFile(fullPath, "bias_refr");
    if (bias_refr<params.bias_refr_min || bias_refr>params.bias_refr_max){
        std::cout<< "ERRO!! O valor de bias_refr= "<< bias_refr << " está fora dos limites!!"<< " Ele deve ser entre=" << params.bias_refr_min << " e " << params.bias_refr_max << std::endl; 
        return false; 
    }  


    // Acessa a Facility de Biases de baixo nível:
    auto *i_ll_biases = this->get_device().get_facility<Metavision::I_LL_Biases>();
    // Testa se o acesso foi liberado:
    if (!i_ll_biases) {
        std::cerr << "[Erro] Nao foi possivel acessar a interface de Biases do hardware!" << std::endl;
        return false;
    }

    // Atualzia os biases na câmera:
    try {
        // Atuazia os valores um por um diretamente no registrador do sensor. Este processo é "on-the-fly", sem precisar de cam.stop()
        i_ll_biases->set("bias_diff_on", bias_diff_on);
        i_ll_biases->set("bias_diff_off", bias_diff_off);
        i_ll_biases->set("bias_fo", bias_fo);
        i_ll_biases->set("bias_hpf", bias_hpf);
        i_ll_biases->set("bias_refr", bias_refr);

        std::cout << "Câmera(s) iniciadas:" << std::endl;
        std::cout << "Câmera " << nome << ":....... NºSérie " << serialNumber << std::endl;
        std::cout << "Câmera " << nome << ":....... " << "Biases atualizados" << std::endl;
        return true;
    } catch (const std::exception &e) {
        std::cerr << "[Erro] Não foi possível gravar biases na câmera!!! " << e.what() << std::endl;
        return false;
    }   

    // Abaixo segue um método aleternativo para setar os biases da camera, ele é de mais alto nível, com apenas 1 linha:
    // cam.load(path.c_str());
    //return true;
}



// Este método configura a camera de eventos para gerar trigger por hardware:
void EventCamera::enableHardwareTrigger() {
    try {
        // O método get_device() retorna o dispositivo atual
        // O facility retorna um ponteiro para a interface de TriggerIn, que é a responsável por configurar o sincronismo de hardware e gerar trigger por hardware:
        //retorna um ponteiro para a interface de TriggerIn, que é a responsável por configurar o sincronismo de hardware e gerar trigger por hardware:
        auto *i_trigger_in = this->get_device().get_facility<Metavision::I_TriggerIn>();

        if (i_trigger_in) {
            // Habilita o canal principal (Main) para bordas de subida e descida
            i_trigger_in->enable(Metavision::I_TriggerIn::Channel::Main);           
            std::cout << "Câmera " << nome << ":....... " << "Trigger habilitado" << std::endl;
        } 
        else 
            std::cerr << "[Erro] Não foi possível acessar a interface de TriggerIn do hardware!" << std::endl;
    }
    catch (const std::exception &e) {
        std::cerr << "[EXCECAO Trigger]: " << e.what() << std::endl;
    }    
}


std::string EventCamera::verificaModoDeSincronismo(Metavision::I_CameraSynchronization *i_cam_sync){
    try {
        Metavision::I_CameraSynchronization::SyncMode modo_atual = i_cam_sync->get_mode();

        switch (modo_atual) {
            case Metavision::I_CameraSynchronization::SyncMode::MASTER:
                return "MASTER";
                break;
            case Metavision::I_CameraSynchronization::SyncMode::SLAVE:
                return "SLAVE";
                break;
            case Metavision::I_CameraSynchronization::SyncMode::STANDALONE:
                return "STANDALONE";
                break;
            default:
                return "DESCONHECIDO";
                break;
        }
     
    } catch (const std::exception &e) {
        std::cerr << "[EXCECAO Sincronismo]: " << e.what() << std::endl;
        return "ERRO_EXCECAO";
    }
    return "FALHA_LOGICA";
}

// Método que configura o sincronismo de tempo entre duas cameras de eventos. 
// Se ela for master, será confiurada para gerar o clock de sincronismo de tempo.
// Se for slave, será confiurada para receber o clock de sincronismo de tempo da camera master.
// Fonte: https://docs.prophesee.ai/4.6.2/hw/manuals/timing_interfaces.html#chapter-timing-interfaces-trigger-interfaces 
void EventCamera::configSincronismo() {
    try{
        std::string tipo_sincronismo;
        Metavision::I_CameraSynchronization *i_camera_synchronization;
        if (isMaster){
            i_camera_synchronization = this->get_device().get_facility<Metavision::I_CameraSynchronization>();
            if (i_camera_synchronization) {
                tipo_sincronismo= verificaModoDeSincronismo(i_camera_synchronization);
                i_camera_synchronization->set_mode_master();
                tipo_sincronismo= verificaModoDeSincronismo(i_camera_synchronization);
                std::cout << "Câmera " << nome << ":....... " << tipo_sincronismo << std::endl;
                std::cout << std::endl;
            } 
            else 
                std::cerr << "[Erro] Não foi possível acessar a interface de CameraSynchronization do hardware!" << std::endl;
        }
        else{
            i_camera_synchronization = this->get_device().get_facility<Metavision::I_CameraSynchronization>();
            if (i_camera_synchronization) {
                tipo_sincronismo= verificaModoDeSincronismo(i_camera_synchronization);
                i_camera_synchronization->set_mode_slave();
                tipo_sincronismo= verificaModoDeSincronismo(i_camera_synchronization);
                std::cout << "Câmera " << nome << ":....... " << tipo_sincronismo << std::endl;
                std::cout << std::endl;
                
            } 
            else 
                std::cerr << "[Erro] Não foi possível acessar a interface de CameraSynchronization do hardware!" << std::endl;
        }
    }
    catch (const std::exception &e) {
        std::cerr << "[EXCECAO Sincronismo]: " << e.what() << std::endl;
    }
}


// Este método inciai o resgistro dos eventos, solicitado pelo trigger de hardware: 
bool EventCamera::startRecording(const std::string& fullPath) {
    try {
        return this->start_recording(fullPath);
    } catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha ao iniciar gravação: " << e.what() << std::endl;
        return false;
    }
}


// Para a gravação de eventos:
bool EventCamera::stopRecording() {
    return this->stop_recording();
}


// Retorne o serial que já está na classe
std::string EventCamera::getSerial() {
    return serialNumber;
}
