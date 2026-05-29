## Controle do sistema de visão estéreo baseado em duas câmeras de eventos SilkyEvCam 
- Autor: Moacir Wendhausen
- Projeto: Voris  
- Data: 13/02/2026  
 

### Setup:
- CPU ARM Jetson Orin Nano
- Plataforma Ubuntu 22.04
- Câmera de eventos SilkyEvCam da Century Arks
- Metavison SDK openeb Versão 5.1.1
- Plugin Century Arks SilkyEvCam 5.1.1
- OpenCV 4.8.0

---
### Características da SilkyEvCam:  

    Available Data Encoding Formats                   EVT3
    Connection                                        USB
    Current Data Encoding Format                      EVT3
    FW Build Date                                     Sun Oct 30 23:37:15 2022
    FW Release Version                                3.9.0-C
    FW Speed                                          5000
    Integrator                                        CenturyArks
    Sensor Name                                       Gen3.1
    Serial                                            00000680
    System Version                                    4.2.0

------

**Principais características do programa:**  
  
1- Visualização gráfica dos eventos em tempo real a partir do sisema stereo ou single câmera de eventos;

2- Salva dados da camera de eventos através de **thread** sincronizada com **trigger de hardware** com duração variável em milisegundos.

3- Efetua sincronismo de tempo entre as câmeras de eventos por hardware. Este sincronimso é feito através do conector série IX da câmea de eventos, 
   usando os sinais SYNC_IN e SYNC_OUT. Câmera esquerda configurada com MASTER e a câmera direita como SLAVE;

4- Controle da luminosidade, potência, do Led (laser) através de dois canais de PWM da Jetson Orin Nano. Um canal controla a potência atravcés do Duty-Cycle,
   o outro através do controle da tensão 0-10, usando-se um HW converso PWM-Tensão;
   
5- Controle de aquisição por trigger de hardware através dos pinos de IO do conector header da Jetson;

