// Autor: Moacir Wendhausen
// Projeto: VORIS
// Data: 21/01/2026
// Função: Este header contém a classe principal que controla a câmera de eventos, usando o SDK Metavision da Prophesee. 
// A classe EventCamera herda todos os métodos e atributos da classe Metavision::Camera, e sobrescreve os métodos de inicialização, 
// configuração de biases, configuração de trigger por hardware, entre outros, para adaptar às necessidades do projeto.


#ifndef EVENT_CAMERA_H
#define EVENT_CAMERA_H

#include <string>
#include <iostream>

#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/hal/device/device_discovery.h>
#include <metavision/hal/facilities/i_camera_synchronization.h>

#include "trata_parametros.h"

struct paramsEventCam
{
    std::string serial_cam= "unknow";
    std::string versionFirm= "unknow";
    std::string dataEncodeFormat= "unknow";
    std::string plugin= "unknow";
    std::string fabricante= "unknow";
};


// Esta classe EventCamera herda os métodos e atributos públicos da classe Metavision::Camera, que é a classe principal do SDK da Metavision para interagir com as câmeras de eventos.
// VAntagens de trabalhar com um classe proprietária é que além dela herdar os métodos da classe Metavision::Camera, ela pode ter seus próprios métodos e atributos personalizados 
// para facilitar a integração com o restante do código, como por exemplo, métodos para ler e configurar os biases, métodos para iniciar e parar a gravação, etc. 
class EventCamera : public Metavision::Camera { 

private:
    // Numero de serie da camera:
    std::string serialNumber;

    // Nome referencia da camera:
    std::string nome;

    // Flag que indica se a camera é master ou slave:
    // * Master: a câmera gera o clock de sincronismo de tempo.
    // * Slave: A câmera recebe o clock de sincronismo de tempo.
    bool isMaster;

    // Ponteiro para a interface de Trigger de hardware
    Metavision::I_TriggerIn* trigger_in = nullptr;

   //  std::unique_ptr<Metavision::Camera> cam;

    // Variavel tipo struct que guarda os paramerto gerais da camera
    paramsEventCam parametrosGerais;

    // Apenas guarda o estado aual da camera instanciada:
    bool inicializada;

    bool is_running;

    // Patha para leitura do json:
    std::string fileName= "settings.json";
    std::string path= "../";
    const std::string fullPath= path + '/' + fileName;

    // Variáveis que guardam a geometria da camera:
    int width = 0;
    int height = 0;

    std::string verificaModoDeSincronismo(Metavision::I_CameraSynchronization *i_cam_sync);


public:
    // Construtor que recebe o Serial Number:
    EventCamera(std::string serial, std::string nm, bool master=false);
    ~EventCamera(){};

    // Le da camera de eventos todos os valores dos biases:
    void readCameraBiases();

    // Método para leitura dos parêmtros gerias da camera de eventos:
    void getParametrosGeraisEventCam();

    void callStart();

    // APenas para pegar o nº de serie:
    std::string getSerial(); 

    // Método que efetua a leitura dos biases e armazena em vairável:
    bool setBias(PARAMETROS_GERAIS &params);

    int getWidth() const { 
        return width; 
    }

    int getHeight() const { 
        return height; 
    }


    // Método que inicia a gravação, captura, dos eventos por trigger:
    bool startRecording(const std::string& fullPath);

    // Metodo que para a gravação dos eventos:
    bool stopRecording();

    // 
    void setCDCallback(std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)> cb) {
        this->cd().add_callback(cb);
    }

    // Método para configurar o sincronismo de hardware, e gerar trigger por hardware:
    void enableHardwareTrigger();

    // Métod que configura o sincronismo de tempo entre duas cameras de eventos:
    void configSincronismo();

};

#endif