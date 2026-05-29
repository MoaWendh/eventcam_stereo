
cmake_minimum_required(VERSION 3.5)

project(metavision_trigger_viewer)

# Define o padrão C++17 (versão 2027):
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Busca de Dependências
# Metavision SDK (Componentes core e stream para câmera e gerador de frames)
find_package(MetavisionSDK COMPONENTS core stream REQUIRED)

# Boost (Exigido internamente pelo Metavision)
find_package(Boost COMPONENTS program_options REQUIRED)
add_compile_definitions(BOOST_BIND_GLOBAL_PLACEHOLDERS)

# OpenCV (Para a janela gráfica e processamento de frames)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc viz)

# Libgpiod (Para o controle dos pinos da Jetson)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

# Configuração Manual do Spinnaker (FLIR)
# Definimos as variáveis manualmente pois as libs estão em /opt
set(SPINNAKER_INCLUDE_DIR /opt/spinnaker/include)
set(SPINNAKER_LIB_DIR /opt/spinnaker/lib)

# 2. Configuração de Inclusões
# Adiciona os diretórios de inclusão encontrados e o caminho específico do OpenCV 4 na Jetson
include_directories(
    ${MetavisionSDK_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GPIOD_INCLUDE_DIRS}
    ${SPINNAKER_INCLUDE_DIR}  # Inclusão dos headers do Spinnaker
    /usr/include/opencv4
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 3. Definição do Executável
set(SOURCE_FILE eventcam_stereo.cpp trata_IO.cpp trata_convcam.cpp trata_evcam.cpp) # 
add_executable(eventcam_stereo ${SOURCE_FILE})

# 4. Linkagem de Bibliotecas
# Adiciona o caminho de busca das bibliotecas do Spinnaker
target_link_directories(eventcam_stereo PRIVATE ${SPINNAKER_LIB_DIR})

target_link_libraries(eventcam_stereo
    PRIVATE
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
    ${GPIOD_LIBRARIES}
    Boost::program_options
    Spinnaker
)

# Mensagens de status para conferência durante o cmake ..
message(STATUS "Spinnaker path: ${SPINNAKER_INCLUDE_DIR}")
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")