# Sistema de monitoramento de GLP para espaços confinados
## Projeto de Microprocessadores - UPE/Poli

## **Orientações para compilação**:
Para que o projeto possa ser compilado corretamente, é necessário definir o *SSID* e *senha* da rede Wi-Fi que o ESP32 irá se conectar.

Por questões de segurança durante o compartilhamento do projeto, sugerimos que seja criado um arquivo chamado **env.h** ao lado do código principal [station_firmware.ino](station_firmware/station_firmware.ino) com a seguinte estrutura:

```cpp
// WiFi Config
#define ENV_SSID        "SSID da rede WiFi aqui"
#define ENV_PASSWORD    "Senha da rede WiFi aqui"
```