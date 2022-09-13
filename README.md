<h1>Dokumentation WRO </h1>
Bei dem Vorentscheid haben wir einen Fischertechnik-TXT4.0-Roboter verwendetet. Da dieser fast keine Python Dokumentation besitzt, die Sensoren und Motoren extrem teuer und Fehleranfällig sind, haben wir uns für dieses Finale entschieden, auf einen Raspberry-Pi-Microcontroller umzusteigen. Unser momentanes Auto besteht trotzdem überwiegend aus Fischertechnik Steck-Teilen, da diese stabil und leicht veränderbar sind. Aber wir benutzen ganz andere technische Komponenten: Momentan haben wir 2 Ultraschallsensoren (links und rechts), ein Servomotor hinten platziert, da wir so besser um die Kurven fahren können, einen sogenannten „Fischertechnik-Hat“, mit welchem wir den Encomoter des Fischertechniksets steuern können, eine 170 Grad-Kamera, um die Hindernisse zu erkennen und um am Ende des Eröffnungsrennen stehen zu bleiben; und natürlich den Raspberry 3B mit einem „Aufsatz“ und einem Display. Diese ganzen Elemente sind mit Jumper-Kabel verbunden. Für die Ultraschallsensor haben wir noch eine Platine gelötet, da die Ultraschall Sensoren Wiederstände brauchen. Diese Verbesserungen ermöglichten uns viele Vorteile. Einerseits sind wir viel freier bei den Komponenten des Roboters. Zum Beispiel kostet ein Sensor von Raspberry oder Arduino maximal 10€, wo hingegen ein Sensor von Fischertechnik bei 50€ anfängt. Wir haben auch viel mehr Vorteile dadurch, weil wir uns die Gehäuse und Halterungen dieser Sensoren einfach in einem 3D-Drucker drucken konnten. Dafür mussten wir die ganzen Gehäuse und Befestigungen modellieren. Darüber hinaus konnten wir den Roboter in seiner software so anpassen wie wir es wollten. Wir haben unser Programm in der Programmiersprache python geschrieben. Diese Skripte haben wir dann über ein lokales Netzwerk zwischen dem Rasberry pi und dem Computer hochgeladen. In dem Programm nimmt sich der Roboter bei dem Eröffnungsrennen die Werte von den Ultraschallsensoren, welche links und rechts am Roboter leicht schräg positioniert sind. Aus diesen Werten rechnet der Roboter den Mittelpunkt aus und versucht sich die ganze Zeit sich diesem zu nähern. Dabei lenkt er umso mehr er vom Mittelpunkt entfernt ist. Zum Stoppen am Ende des Rennens benutzen wir eine Kamera, welche die blauen Linien zählt und nach 12 erfassten Linien nach ca. 3-5 Sekunden stehen bleibt. Diese sogenannte Kamera ist eine 170 Grad Kamera und wir benutzen sie auch bei dem Hindernisrennen. Hier benutzen wir zum Fahren das Programm von dem Eröffnungsrennen und fügen zum Beispiel hinzu, dass wenn die Kamera grün erkennt sich der Mittelpunkt nach links verschiebt und wir so keine Strecken „hard coden“ müssen und das Auto weiterhin autonom fährt. Das gleiche passiert bei Rot nur der Mittelpunkt wird nach rechts verschoben wird. 
