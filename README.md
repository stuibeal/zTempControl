Temperaturregelung für den Zapfapparat

2 PID Regler:
- je nachdem ob gezapft wird oder nicht übernimmt jeweils einer
- Regelung nach innerem PT100 Fühler: bei Nichtzapfung, schnellere Reaktion
- Regelung nach dem äußeren PT100 Fühler: bei Zapfung, näher am Zulauf, kühlt ganzen Block

Steuert Pumpe:
- ab 99W Leistung zweite Pumpe für Kühlkreislauf zuschalten (LED grün)
- normale Pumpe für Kühlkreislauf (LED rot)

Steuert Lüfter (LED gelb):
- wenn kein Kühlwasser durchläuft, springen die Lüfter an

Ansteuerung vom Haupt uC:
- i2c (fast mode)
- sendet/empfängt immer alle relevanten Daten
- siehe Inc/common.h




 
