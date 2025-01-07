
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


### ToDo

1. Graphen in einer Zeichnung
2. Benchmarks laufen lassen (5)
    - Time
    - Length
    - Smoothness
    - Obstacle avoidance
3. visibilityPRM optimieren
4. Gedanken machen, wie die Pfade optimiert werden könnten