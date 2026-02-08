🐟 AquaOS - Sistema de Automação para Aquários (IoT)
> Versão: 1.0
> Plataforma: ESP32 & ESP RainMaker

O AquaOS é um sistema de controle e monitoramento para aquários de água doce, desenvolvido para rodar no microcontrolador ESP32. O projeto utiliza a plataforma ESP RainMaker para controle remoto via smartphone (iOS/Android), permitindo gerenciamento de temperatura, iluminação, pH e alimentação automática.

🚀 Funcionalidades
📱 Controle Remoto Total: Via app ESP RainMaker (sem necessidade de broker MQTT externo).
🌡️ Termostato Inteligente:
    * Modo Automático: Mantém a temperatura estável (com histerese configurável) acionando aquecedor ou ventoinhas (coolers).
    * Modo Manual: Controle direto dos relés.
💡 Iluminação Automatizada: Timer sincronizado via NTP (Network Time Protocol) para simular ciclo dia/noite.
🍽️ Alimentador Automático: Controle preciso de motor de passo para dosagem de ração, com registro de "nível de ração".
🛡️ Modo TPA (Manutenção): Bloqueia filtros e aquecedores temporariamente para Troca Parcial de Água.
📊 Monitoramento: Logs em tempo real de temperatura e pH.

🛠️ Hardware Necessário
Microcontrolador: ESP32 (DevKit V1 recomendado)
Atuadores:
    * Módulo Relé 4 Canais (Para Filtro, Aquecedor e Luzes).
    * Transistor/MOSFET ou ULN2003 (Para Coolers PWM).
    * 2x Motores de Passo 28BYJ-48 + Drivers ULN2003 (Mecanismo do Alimentador).
Sensores:
    * DS18B20 (Temperatura - Prova d'água).
    * Sensor de pH (Analógico - Ex: PH-4502C).

🔌 Pinagem (Pinout)
A configuração padrão dos pinos está definida no arquivo principal, mas pode ser ajustada conforme necessidade:
| Componente | GPIO (ESP32) | Tipo |
| :--- | :--- | :--- |
| Relé Filtro | GPIO 5 | Saída Digital |
| Relé Aquecedor | GPIO 17 | Saída Digital |
| Relé Luzes | GPIO 23 | Saída Digital |
| Coolers | GPIO 19 | PWM / Digital |
| Sensor Temp (Data) | GPIO 4 | OneWire |
| Sensor pH | GPIO 34 | Entrada Analógica |
| Motor Passo 1 | 14, 26, 27, 13 | 4 Fios (IN1-IN4) |
| Motor Passo 2 | 32, 25, 33, 18 | 4 Fios (IN1-IN4) |

📦 Dependências e Bibliotecas
Este projeto requer as seguintes bibliotecas instaladas na Arduino IDE ou PlatformIO:
1.  ESP RainMaker (Nativa do pacote ESP32 board manager > 2.0.0)
2.  AccelStepper (Controle avançado de motores de passo)
3.  OneWire & DallasTemperature (Sensor DS18B20)

⚙️ Instalação e Configuração
1.  Clone o repositório:
    ```bash
    git clone [https://github.com/SEU_USUARIO/AquaOS-ESP32.git](https://github.com/SEU_USUARIO/AquaOS-ESP32.git)
    ```
2.  Configure as Credenciais:
    Renomeie o arquivo `secrets.example.h` para `secrets.h`.
    Defina sua senha de pareamento (`PROV_POP`) e nome do node.
    Nota: O arquivo `secrets.h` é ignorado pelo Git para segurança.
3.  Ajuste o Fuso Horário:
    No código principal, ajuste a variável `gmtOffset_sec` para sua região (Padrão: UTC-4 Cuiabá).
4.  Upload:
    Selecione a placa ESP32 Dev Module.
    Habilite o **Partition Scheme para `RainMaker` (ou Huge App) para garantir espaço.
    Compile e envie.

📱 Pareamento (Primeiro Uso)
1.  Instale o app ESP RainMaker (Android/iOS).
2.  Ative o Bluetooth do celular.
3.  Abra o app e clique em "Adicionar Dispositivo".
4.  Escaneie o QR Code (ou pareamento manual via BLE).
5.  Digite a senha de prova de posse (definida no `secrets.h`).

---
Desenvolvido por: Ana Lívia da Silva Gomes - Gaewnt
Estudante de Engenharia de Controle e Automação