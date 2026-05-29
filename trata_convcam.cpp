// Autor: Moacir Wendhausen
// Projeto: VORIS
// Data: 21/01/2026
// Função: Este arquivo contém a implementação da classe ConvCamera, que é responsável por controlar a câmera convencional usando o SDK Spinnaker da FLIR. A classe ConvCamera herda todos
// os métodos e atributos da classe Spinnaker::Camera, e sobrescreve os métodos de inicialização, captura de imagem, entre outros, para adaptar às necessidades do projeto.


#include "trata_convcam.h"
#include <iostream>

// Construtor: apenas inicializa membros que não existem na classe pai
ConvCamera::ConvCamera() : Spinnaker::Camera() {
    // Se você removeu a variável 'inicializada', não precisa de nada aqui.
    // O estado da câmera agora é verificado via this->IsInitialized()
}


// Destruidor: a limpeza do objeto Camera é feita automaticamente 
// pelo gerenciamento de memória da SDK/SmartPointers.
ConvCamera::~ConvCamera() {
}


void ConvCamera::exibir_modelo_camera() {
    // Substituímos sua variável local pelo método herdado IsInitialized()
    if (!this->IsInitialized()) {
        std::cout << "Câmera não inicializada. Chame Init() primeiro." << std::endl;
        return;
    }

    // Acessamos o NodeMap diretamente através do método herdado
    auto& nodeMap = this->GetNodeMap();
    
    Spinnaker::GenApi::CStringPtr ptrModel = nodeMap.GetNode("DeviceModelName");
    if (Spinnaker::GenApi::IsAvailable(ptrModel)) {
        // DeviceSerialNumber() também é herdado da Spinnaker::Camera
        std::cout << "Câmera: " << ptrModel->GetValue() 
                  << " ..... Nº Serial: " << this->DeviceSerialNumber() << std::endl;
    }
}


// Getters simplificados usando os métodos da classe pai
std::string ConvCamera::get_serial() const {
    return this->DeviceSerialNumber().c_str();
}


bool ConvCamera::is_ok() {
    return this->IsInitialized();
}


Spinnaker::GenApi::INodeMap& ConvCamera::get_nodemap() {
    if (!this->IsInitialized()) {
        throw std::runtime_error("A câmera precisa estar inicializada (Init) para acessar o NodeMap!");
    }
    return this->GetNodeMap();
}



// Implementação do Init que o Linker não estava achando
void ConvCamera::Init() {
    // Chama a implementação original da SDK da FLIR
    Spinnaker::Camera::Init(); 
    
    // Agora atualiza o seu estado interno
    this->inicializada = true;
}



// Implementação do DeInit que o Linker não estava achando
void ConvCamera::DeInit() {
    // Chama a implementação original da SDK da FLIR
    Spinnaker::Camera::DeInit();
    
    // Atualiza o seu estado interno
    this->inicializada = false;
}



Spinnaker::ImagePtr ConvCamera::capturarImagem() {
    try {
        if (!this->is_ok()) return nullptr;

        // 1. Inicia o fluxo de dados do sensor para a memória
        this->BeginAcquisition();

        // 2. Captura o próximo frame disponível (timeout de 1000ms)
        Spinnaker::ImagePtr pImage = this->GetNextImage(1000);

        if (pImage->IsIncomplete()) {
            std::cerr << "Imagem incompleta!" << std::endl;
            pImage->Release(); // Importante liberar antes de retornar
            this->EndAcquisition();
            return nullptr;
        }
        
        // 1. Criar uma instância do processador de imagem
        Spinnaker::ImageProcessor processor;

               
        // 3. Converter a imagem usando o processador
        // Note: convert() retorna um novo ImagePtr
        Spinnaker::ImagePtr convertedImage = processor.Convert(pImage, Spinnaker::PixelFormat_Mono8);             


        // 4. PARA o fluxo (Crucial para não sobrecarregar o barramento USB)
        this->EndAcquisition();

        if (convertedImage != nullptr) {
            std::cout << "Imagem capturada com sucesso - Resolução: " << convertedImage->GetWidth() 
                    << "x" << convertedImage->GetHeight() 
                    << " | Formato: " << convertedImage->GetPixelFormatName() << std::endl;
        }       

        return convertedImage;
    }

    catch (const Spinnaker::Exception& e) {
        std::cerr << "Erro na captura: " << e.what() << std::endl;
        return nullptr;
    }
}