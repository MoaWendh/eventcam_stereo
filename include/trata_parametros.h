// Autor: Moacir Wendhausen
// Projeto: VORIS
// Data: 21/01/2026
// Função: Este header contém as declarações da struct "PARAMETROS_GERAIS", que é usada para armazenar os parâmetros gerais do sistema, 
// como os tempos de trigger, números de série das câmeras, configurações de PWM, entre outros.
// COntpem parâmetros associados tanto a ao IO da Jetson Orin Nano, IOs e PWMs, quanto a configuração da câmera de eventos, como os biases, 
// que são usados para configurar a câmera de eventos via SDK Metavision.

#ifndef CONFIG_PARAMETROS_H
#define CONFIG_PARAMETROS_H

#include <cstdint>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <memory> 
#include <metavision/sdk/core/utils/cd_frame_generator.h>

struct PARAMETROS_GERAIS {
    // Parametros que definem as caracterísicas do triiger por HW da camera: 
    int64_t duracao_pulso_trigger   = 1000000; // em micro segundos
    int64_t duracao_pre_trigger     = 50000;   // em micro segundos
    int64_t duracao_pos_trigger     = 50000;   // em micro segundos
    int64_t duracao_led             = 200000;  // em micro segundos
    int numero_ciclos_trigger       = 1;

    // Define se a gravação é feita em stereo ou mono, ou seja, se as duas câmeras de eventos são usadas ou apenas uma:
    const bool stereo= false;

    // 
    //std::string eventFileName= "old/data_evecam_27_05_2026/L/evecam_sn_00000680_132234.raw"

    // Núemros de séries das cameras convencionais
    const std::string serialNumber_conv_cam_01= "25083333";
    const std::string serialNumber_conv_cam_02= "00000414"; 

    // Números de série das câmeras de eventos:
    const std::string serialNumber_event_cam0= "00000414"; // HD
    const std::string serialNumber_event_cam2= "00000679"; // VGA
    const std::string serialNumber_event_cam3= "00000680"; // VGA     
    
    // Definição dos paths referentes aos chips de IO da Jeson que geram o PWM: 
    const std::string channelToExport_A= "/sys/class/pwm/pwmchip3/";  
    const std::string channelToExport_B= "/sys/class/pwm/pwmchip2/"; 

    // Definição do Duty-cicle dos PWMs, valor em percentual.
    long dutyCycle_PWM_A    = 5;   //  PWM referente ao controle do duty cycle para blink led.
    long dutyCycle_PWM_B    = 10;  // PWM referente ao controle da tensão.

    // Definição do periodo dos PWMs, valor em nano segundos.
    long periodo_PWM_A      = 100000000;  // A: referente ao blink do led (pino 32 da Jetson). 
    long periodo_PWM_B      = 1000000;    // B: referente a tensão do led (pino 33 da Jetson).     
     
    // Define se está usando o led de potencia LT2PR da Opto Engineering
    bool useLed_LT2PR       = true;

    // Declaração variaveis booleans do tipo atomic:
    bool useCamera_Conv     = false;
    bool useCamera_Event    = true;  
    
    // Flag que habilita menu:
    bool hab_exibe_menu     = true;
    
    // Definição dos parâmetros que guardam os valores máximos e mínimos dos biases da 
    // câmera de eventos, eles são usados para validar os dados lidos do arquivo JSON antes de serem gravados na câmera. 
    // Os biases max e min. são definidos em https://docs.prophesee.ai/stable/hw/manuals/biases.html 
    // Os calores para a ca~mera SilkyEvCam pertencem a geração Gen3.1 VGA, assim os valores máximos e mínimo
 
    const int bias_diff_default = 299; // Não alterar o valor do bias_diff, o default é 299.
    
    int bias_diff_on_min        = bias_diff_default + 75; // O valor mínimo do bias_diff_on é bias_dif_default + 75.
    int bias_diff_on_max        = bias_diff_default + 200; // O valor máximo do bias_diff_on é bias_dif_default + 200.
    
    int bias_diff_off_min       = 100; // O valor mínimo do bias_diff_off é 100.
    int bias_diff_off_max       = bias_diff_default - 65; // O valor máximo do bias_diff_off é bias_dif_default -65
    
    int bias_fo_min             = 1250;
    int bias_fo_max             = 1800;
    
    int bias_hpf_min            = 900;
    int bias_hpf_max            = 1800;
   
    int bias_refr_min           = 1300;
    int bias_refr_max           = 1800;
    
};

//Valores de parâmetros para confiuração do frame genetator, eles são usados para configurar o gerador de frames da câmera de eventos, 
// que é responsável por criar os frames a partir dos eventos capturados pela câmera, e exibi-los em tempo real no viewer.
struct parametrosFrameGenerator{ 
    std::mutex mutex;
    cv::Mat frame_L, frame_R;
    cv::Mat dashboard;
    cv::Mat roi_L, roi_R, roi_menu;
    std::atomic<bool> showViewer{true};
    std::string window_name_single = "Visualizacao - Single Câmera";
    std::string window_name_stereo = "Visualizacao - Sist. Estéreo";
    int largura_menu = 300;
    std::vector<std::unique_ptr<Metavision::CDFrameGenerator>> generators;
};


#endif