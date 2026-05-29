// Autor: Moacir Wendhausen
// Projeto: VORIS
// Data: 21/01/2026
// Função: Este header contém a classe principal que controla a câmera convencional, usando o SDK Spinnaker da FLIR. 
// A classe ConvCamera herda todos os métodos e atributos da classe Spinnaker::Camera, e sobrescreve os métodos de inicialização, 
// captura de imagem, entre outros, para adaptar às necessidades do projeto.

#ifndef CAMERA_CONV_H
#define CAMERA_CONV_H

#include "Spinnaker.h"
#include <string>
#include <iostream>

// Esta classe ConvCamera herda todos os métodos e atributos da classe Spinnaker::camera:
class ConvCamera : public Spinnaker::Camera {
private:
    // Apenas guarda o estado aual da camera instanciada:
    bool inicializada;

public:

    Spinnaker::GenApi::INodeMap& get_nodemap();

    ConvCamera();
    virtual ~ConvCamera();

    // Sobrescrevemos os métodos:
    void Init();
    void DeInit();

    void exibir_modelo_camera();
    std::string get_serial() const;
    bool is_ok();

    Spinnaker::ImagePtr capturarImagem();    
};

#endif