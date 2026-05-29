#include "trata_IO.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <gpiod.h>



// Método para efetuar a configuração do IO da Jetson:
GPIO_Lines configJetson::configura_GPIO_Jetson(struct gpiod_chip **chip_ptr) {
    GPIO_Lines lines_out = {nullptr, nullptr, nullptr, nullptr, nullptr};

    //********************* Inicia configuração do barramenteo IO da Jetson: 
    // 1º. Abre o chip definido na string chipIO (privada da classe) e armazena no ponteiro fornecido pelo main:
    *chip_ptr = gpiod_chip_open_by_name(chipIO.c_str());

    // Testa abertura do chip:
    if (!(*chip_ptr)) {
        perror("Erro ao abrir gpiochip0!!!");
        return lines_out;
    }

    // 2º. Captura os pinos, lines, no GPIO da Jetosn para todos os pinos que serão utilziados:
    // Para o TRIGGER da camera de eventos usando lineF (Pino 7):
    lines_out.triggerEventCam = gpiod_chip_get_line(*chip_ptr, lines.line_IO_G);
    if (!lines_out.triggerEventCam) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle do trigger da camera de eventos na Linha: " << lines.line_IO_G << std::endl;
        return lines_out;
    }
    gpiod_line_set_value(lines_out.triggerEventCam, 0);

    // Para piscar o LED usando lineH (Pino 16):
    lines_out.piscaLed = gpiod_chip_get_line(*chip_ptr, lines.line_IO_I);
    if (!lines_out.piscaLed) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle do Led na linha: " << lines.line_IO_I << std::endl;
        return lines_out;
    }
    gpiod_line_set_value(lines_out.piscaLed, 0);    


    // Para o Trigger da câmera convencional:
    lines_out.triggerNormalCam = gpiod_chip_get_line(*chip_ptr, lines.line_IO_H);
    if (!lines_out.triggerNormalCam) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle do trigger da camera convencional na linha: " << lines.line_IO_H << std::endl;
        return lines_out;
    }
    gpiod_line_set_value(lines_out.triggerNormalCam, 0);    
    

     // Para o controle da fase 1 do motor:
    lines_out.controlMotor01 = gpiod_chip_get_line(*chip_ptr, lines.line_IO_A);
    if (!lines_out.controlMotor01) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle da fase 1 do Motor na linha: " << lines.line_IO_A << std::endl;
        return lines_out;
    }
    gpiod_line_set_value(lines_out.controlMotor01, 0);  
    
     // Para o controle da fase 2 do motor:
    lines_out.controlMotor02 = gpiod_chip_get_line(*chip_ptr, lines.line_IO_B);
    if (!lines_out.controlMotor02) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle da fase 2 do Motor na linha: " << lines.line_IO_B << std::endl;
        return lines_out;
    }
    gpiod_line_set_value(lines_out.controlMotor02, 0);   
   

    // 3º. Faz um request da linha como SAÍDA para o trigger:
    // Request do trigger da camera:
    if (gpiod_line_request_output(lines_out.triggerEventCam, "triggerEventCam", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_out.triggerEventCam, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // Usa o mapeamento físico privado da classe para o log:
        std::cout << "Jetson: trigger EvCam ................. pino: " << pinos.header_pin_IO_G << "  (Nivel= 0V)"<< std::endl;
    } 
    else {
        perror("[ERRO de request] Não foi possível configurar como OUTPUT o pino do trigger da camera de eventos.");
        return lines_out;
    }

    // Request do controloe de piscagem do led:
    if (gpiod_line_request_output(lines_out.piscaLed, "piscaLed", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_out.piscaLed, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: Pisca Led ..................... pino: " << pinos.header_pin_IO_I << " (Nivel= 0V)" << std::endl;
    } 
    else {
        perror("[ERRO de Request] Não foi possível configurar como OUTPUT o pino do LED.");
        return lines_out;
    }   


    // Request do controloe de trigger da camera convencional:
    if (gpiod_line_request_output(lines_out.triggerNormalCam, "TriggerNormalCam", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_out.triggerNormalCam, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: Trigger camera convencional ... pino: " << pinos.header_pin_IO_H << " (Nivel= 0V)"<< std::endl; 
    } 
    else {
        perror("[ERRO de Request] Não foi possível configurar como OUTPUT o pino de Trigger da camera convencional.");
        return lines_out;
    }   


     // Request do controle da fase 1 do Motor:
    if (gpiod_line_request_output(lines_out.controlMotor01, "ControlMotor_Fase01", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_out.controlMotor01, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: Controle MOTOR Fase 1 ......... pino: " << pinos.header_pin_IO_A << " (Nivel= 0V)"<< std::endl; 
    } 
    else {
        perror("[ERRO de Request] Não foi possível configurar como OUTPUT o pino de controle da Fase 01 do MOTOR.");
        return lines_out;
    }    


     // Request do controle da fase 2 do Motor:
    if (gpiod_line_request_output(lines_out.controlMotor02, "ControlMotor_Fase02", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_out.controlMotor02, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: Controle MOTOR Fase 2 ......... pino: " << pinos.header_pin_IO_B << " (Nivel= 0V)"<< std::endl; 
        std::cout << std::endl;
    } 
    else {
        perror("[ERRO de Request] Não foi possível configurar como OUTPUT o pino de controle da Fase 02 do MOTOR.");
        return lines_out;
    }        


    // ANTES DO RETURN, salve o estado interno:
    this->chip_ptr_interno = *chip_ptr;
    this->gpios_internas = lines_out;
    
    return lines_out;
}




// Metodo que apeans fecha e libera o GPIO da Jetson:
void configJetson::liberaGPIO_Jetson(struct gpiod_chip *chip, GPIO_Lines gpios){

    // Libera as linhas individuais (se existirem)
    if (gpios.triggerEventCam) {
        gpiod_line_release(gpios.triggerEventCam);
        std::cout << "Jetson: Linha do trigger da camera de eventos liberada." << std::endl;
    }

    if (gpios.piscaLed) {
        gpiod_line_release(gpios.piscaLed);
        std::cout << "Jetson: Linha do LED liberada." << std::endl;
    }

     if (gpios.triggerNormalCam) {
        gpiod_line_release(gpios.triggerNormalCam);
        std::cout << "Jetson: Linha do trigger da camera convencional liberada." << std::endl;
    }

    if (gpios.controlLaser) {
        gpiod_line_release(gpios.controlLaser);
        std::cout << "Jetson: Linha do Laser liberada." << std::endl;
    }

     if (gpios.controlMotor01) {
        gpiod_line_release(gpios.controlMotor01);
        std::cout << "Jetson: Linha 1 do motor liberada." << std::endl;
    }   
 
      if (gpios.controlMotor02) {
        gpiod_line_release(gpios.controlMotor02);
        std::cout << "Jetson: Linha 2 do motor liberada." << std::endl;
    }     


    // Fecha o controlador (chip)
    if (chip) {
        gpiod_chip_close(chip);
        std::cout << "Jetson: Controlador " << chipIO << " fechado com sucesso." << std::endl;
    }
}

// Construtor da classe PWM, com as devidas inicializações das variáveis menbros privadas:
PWM::PWM(int64_t periodo_ns, int64_t dutyCycle_perc, std::string path, std::string n_canal) 
        : period(periodo_ns), 
          dutyCycle(dutyCycle_perc),
          fullPath_chip(path),           
          active(false), 
          canal("pwm0/"), 
          nome(n_canal) 
          {
                this->inicializa_canal();
                usleep(100000);
                this->setPeriodo(period);
                usleep(100000);
                this->setDutyCycle(dutyCycle);
                usleep(100000);
          }

// Estrutor PWM:
PWM::~PWM() {
        disable();
        std::cout << "PWM " << canal.c_str() << " desabilitado e classe encerrada." << std::endl; 
    } 


// Método que inicializa o canal pwm:
void PWM::inicializa_canal() {
    // Exporta o canal:
    // Primeiro verifica se a pasta pwm0 existe se sim o canal já foi exportado.
    // A pasta pwm0 é criada com o export, se ela já existe não tem porque recriá-la, basta recofnigurar o pwm 
    if (!std::filesystem::exists(fullPath_chip + canal)){
        // Se não existe exporta:
        std::ofstream export_file(fullPath_chip + "export");
        export_file << "0"; 
        export_file.close(); 
    }
} 
    

// Ajusta o period do PWM:
bool PWM::setPeriodo(int64_t periodo_ns){
    // Atualiza  variável que guarda o periodo:
    bool set_ok= false;
    period= periodo_ns;

    // Configura o periodo:
    if (writeToFile("period", std::to_string(period))){
        std::cout << "Periodo PWM.............."<< this->nome << "= "<< period/1000000 << "ms ("<<1000000000/period << " Hz)" << std::endl;
        set_ok= true;
    }
    else
        std::cout << "[Erro] Não foi possível ajustar periodo canal:" << this->nome << std::endl;
    return set_ok;
}


// AJusta o Duty-Cycle do PWM:
bool PWM::setDutyCycle(int64_t dutyCycle_percentual) {
    dutyCycle= dutyCycle_percentual;
    bool set_ok= false;
    
    // Configura o duty_cycle
    int64_t dutyCycle_ns= (dutyCycle*period)/100;
    if (writeToFile("duty_cycle", std::to_string(dutyCycle_ns))){
        std::cout << "Duty-Cycle PWM..........."<< this->nome << "= "<< dutyCycle << "% ("<< dutyCycle_ns << "ns)." << std::endl;
        set_ok= true; 
    }
    else  
        std::cout << "[Erro] Não foi possível ajustar o duty-cycle do pwm canal:" << this->nome << std::endl;       
    return set_ok;            
}


// Habilita o PWM:
bool PWM::enable() { 
    if (writeToFile("enable", "1"))
        active= true;
    return active;   
}


// Desabilita o PWM:
bool PWM::disable() { 
    if (writeToFile("enable", "0"))
        active= false;
    return active;
}


