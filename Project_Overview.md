


- evtl. falls Zeit: Optimierung Roundtrip Path Planner mit genormter Aufrufmethode der PRM Erstellung




im visibilityPRM_custom
- 1x _learnRoadmap (mit early stopping & connect all nodes [für multiQuery])
- 1x _learnRoadmap (ohne early stopping & connect all nodes [für singleQuery])
- 1x PlanPath (mit early stopping & connect all nodes [für singleQuery])

oder (Idee)
 - Anpassen von PlanPath, sodass auch für single query die gleiche _learnRoadmap methode wie für das multiquery verwendet werden kann



Es müssen noch die Importe und Aufrufe der Methoden und Dateien (umbenannt) korrekt angepasst werden






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



### Achtung

Frage: Warum nutzen wir unsere visibility Roadmap nicht einfach mehrfach? Ist das nicht der Sinn hinter dieser Art an Roadmap?




### ToDo

1. Gedanken machen, wie die Pfade optimiert werden könnten
    - Start and All Goals as Guard or direct connections between each other
    - Alle Connections verbinden
    - Nur Connections vor dem nächsten Guard oder hinter dem vorherigen Guard auf Sichtbarkeit überprüfen
    - evtl. Parallelisierung der Sichtbarkeitsüberprüfungen, um Rechenzeit zu verringern
    - dynamische Veränderung der nTrys (Early Stopping)
2. evtl. Visualisierung anpassen, indem mehrere Planer objekte gespeichert werden (zusätzlicher Vorteil,
    dass alle PRMs gespeichert werden)
3. Eine Roadmap für alle multi Query Algorithmen


### Hinweise Optimierung visibilityPRM

Der Fokus der visibility PRM liegt auf der minimalen Roadmap, nicht auf der Optimierung aller möglichen Kanten.
Aus diesem Grund kann es vorkommen, dass redundante Connections q zwischen zwei Guards A und B entstehen. Dadurch
kann ein suboptimaler Pfad, bei minimaler Roadmap entstehen.

Mit einer Sichtbarkeitsprüfung kann festgestellt werden, ob eine Connection q redundant ist oder nicht. Können zwei
Guards A und B direkt miteinander verbunden werden, ist q redundant und könnte entfernt werden. Dadurch wird der Pfad kürzer und die Roadmap enthält insgesamt weniger Punkte.

Eine erneute Kollisionsprüfung ist ratsam und wird zulasten der Effizienz gehen.


Ziele einer Optimierung könnten sein:
- Redundanzprüfung und anschließende Minimalisierung der Roadmap




### Optimierungsvorschläge

TBD
- unnötige punkte nicht generieren lassen, sondern direkt bei maperstellung nach möglichen Pfaden suchen
    - Pfad ist evtl. nicht optimal, aber der erste geschlossene pfad wird sofort gewählt (evtl. schneller)
    - Idee early stopping (Rechenzeit sinkt)

    - evtl. ein Problem: Viele Connection (unnötig) werden erzeugt, bis alle Guards erzeugt sind, die benötigt werden
    - In der While Schleife (nTrys) alle paar abläufe eine Abfrage ob bereits ein vollständiger Pfad exisitert, erster gefundener Pfad wird nicht optimal sein, aber die nachfolgende Rechenzeit abbrechen

Abgeschlossen
- Start und Goal als erstes als Guard setzten (prüfen ob direkte Verbindung möglich, dann abbruch mit Lösungspfad), ansonsten weiter wie Algorithmus es vorsieht. (Weitere verkürzung des Pfades) - Effizienzsteigerung, da gezielt
Punkte in den Graphen hinzugefügt werden, die eine sinnvolle Position haben. Nicht angewiesen auf random Positionen
    - Hinweis: Sollte dieser Ansatz gewählt werden, muss ggf. die Visualisierung geändert werden,

TBD
- Gewichteten Graphen einführen, um shortest_path die Möglichkeit zu geben tatsächlich den kürzesten Pfad zu ermitteln
    - evtl. Problem: shortest Path Berechnungsaufwand steigt stark an. Rechenzeit steigt.

TBD
- Verbindung aller Punkte (Guards und Connections) miteinander (wenn sie sich sehen) nach der PRM, um Verbindungen zwischen Connections usw. zu erzeugen -> evtl. bessere Pfade, da mehr wege verfügbar

TBD
- Mehrere Connections zwischen den selben Guards zulassen, um immer den kürzestmöglichen Pfad zu ermöglichen