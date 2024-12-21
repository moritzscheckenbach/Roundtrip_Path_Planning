
1: Roundtrip Path Planner
Erzeuge oder Integriere Environment 
    2D map mit Hindernissen
    Parameter: (environment_size, object_amount)
    Return: environment
Erzeuge Startpunkte, Zwischenpunkte
    Parameter: (x_position, y_position, id)
        check: ist Start- & Zwischenpunkt in einem Objekt?
        (check: gibt es Lücken in Id-Folge?)
Path Planner anwenden (ggf. anpassen)
    Parameter: (route_optimization)
        check: modularitäten auf externe path planner
        check: collision check
        check: alle Punkte abgefahren?

2: Evaluation:
Customized visibility PRM
    Smoothing
    Route Plannning optimization
Graphische Representation
    Environment mit Objects, Path (Nodes; gesmoothed und nicht gesmoothed), Start- & Zwischenpunkte


### Hinweis (Moritz)
So wie ich es jetzt verstanden habe sollen wir eine Implementierung als Python-Paket erstellen.
Das ist eigentlich recht simpel, wir arbeiten in *.py-Dateien und schreiben dann noch eine __init__.py dazu. Erst die Präsentation bzw. die Beispiele sollten dann in *.ipynb-Dateien stattfinden (Also eigentlich so, wie es auch schon in den Required Project Materials vorgemacht wurde).


### Vorgehensweise (ChatGPT)
Um den Pfadplaner zu implementieren, würde ich die Aufgabe in mehrere Schritte unterteilen:

1. Projektstruktur erstellen:
Erstellen Sie ein Verzeichnis für den Pfadplaner.
Erstellen Sie Unterverzeichnisse für verschiedene Algorithmen und Hilfsfunktionen.

2. Grundlegende Klassen und Interfaces definieren:
Definieren Sie eine abstrakte Basisklasse oder ein Interface für die Pfadplanungsalgorithmen.
Implementieren Sie konkrete Klassen für jeden Algorithmus (z. B. LazyPRM, A*, Visibility PRM).

3. Eingabeverarbeitung:
Implementieren Sie eine Funktion, die die Eingabeargumente (Startposition, Zielpositionen, Umgebung, Algorithmus) verarbeitet.

4. Algorithmusauswahl und -ausführung:
Implementieren Sie eine Funktion, die den ausgewählten Algorithmus ausführt und die kollisionsfreie Route berechnet.

5. Modularität und Erweiterbarkeit sicherstellen:
Strukturieren Sie den Code so, dass neue Algorithmen oder Features einfach hinzugefügt werden können.

6. Ausgabe generieren:
Implementieren Sie eine Funktion, die die berechnete Route ausgibt und zusätzliche Informationen bereitstellt.

7. Hindernisprüfung und Optimierung:
Implementieren Sie Funktionen zur Überprüfung der Kollisionsfreiheit und zur Optimierung der Route.

8. API-Design:
Definieren Sie eine standardisierte API (z. B. plan_path), die mit bestehenden Pfadplanungssystemen oder Robotik-Frameworks kompatibel ist.
Hier ist ein Beispiel für die Projektstruktur und die grundlegenden Klassen:


path_planner/
├── __init__.py
├── algorithms/
│   ├── __init__.py
│   ├── lazy_prm.py
│   ├── a_star.py
│   └── visibility_prm.py
├── utils/
│   ├── __init__.py
│   └── obstacle_check.py
└── planner.py