# Slave170 - Arduino MEGA Protocol 170 Slave

**Kurzbeschreibung**  
Slave170 ist eine Firmware für den Arduino MEGA, die als Slave im Protokoll 170 agiert. Sie reagiert auf Anfragen eines Masters und signalisiert Status über eine LED.

## Voraussetzungen

- Arduino MEGA (ATmega2560)  
- Arduino-CLI oder Arduino IDE  
- Anschluss über UART (Serial1)  

## Configuration
- `SLAVE_ID`: 0-255 Possible IDs
- `BAUDRATE`: Default 9600

## Bahavior & LED Status
| Zustand     | Typ     | Bedeutung                                |
| ----------- | ------- | ---------------------------------------- |
| **IDLE**    | OK      | Warten auf Anfrage (Betriebsbereit)      |
| **TIMEOUT** | WARNING | Keine Pings mehr eingetroffen            |
| **SUCCESS** | OK      | Anfrage erhalten und korrekt beantwortet |
| **ERROR**   | ERROR   | Fehlerhaftes Frame oder CRC-Fehler       |
  
## Troubleshooting
- **Keine LED-Aktivität**: Prüfe BAUDRATE und Anschlüsse an Serial1 (vielleicht sogar delays im Programm prüfen).
- **Permanent ERROR**: Testframe prüfen, CRC korrekt berechnen.
- **TIMEOUT zu früh**: Timeout-Intervall im Code anpassen.
