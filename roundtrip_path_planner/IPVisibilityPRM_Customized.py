# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from IPPRMBase import PRMBase
import networkx as nx
from scipy.spatial import cKDTree
from IPPerfMonitor import IPPerfMonitor

class VisibilityStatsHandler():
    
    def __init__(self):
        self.graph = nx.Graph()
        
    def addNodeAtPos(self,nodeNumber,pos):
        self.graph.add_node(nodeNumber, pos=pos, color='yellow')
        return
    
    def addVisTest(self,fr,to):
        self.graph.add_edge(fr, to)
        return
        
class VisPRM_Custom(PRMBase):
    """Class implements an simplified version of a visibility PRM"""

    def __init__(self, _collChecker, _statsHandler = None):
        super(VisPRM_Custom, self).__init__(_collChecker)
        self.graph = nx.Graph()
        self.statsHandler = VisibilityStatsHandler() # not yet fully customizable (s. parameters of constructors)
                
    def _isVisible(self, pos, guardPos):
        return not self._collisionChecker.lineInCollision(pos, guardPos)
    
    def _isPathCollisionFree(self, path):
        for i in range(len(path) - 1):
            if self._collisionChecker.lineInCollision(self.graph.nodes[path[i]]['pos'], self.graph.nodes[path[i + 1]]['pos']):
                return False
        return True

    @IPPerfMonitor
    def _learnRoadmap(self, ntry):

        #nodeNumber = 2
        current_node_number = len(self.statsHandler.graph.nodes())
        print(f"Current node number: {current_node_number}")
        nodeNumber = current_node_number + 1
        print(f"New node number: {nodeNumber}")
        currTry = 0
        while currTry < ntry:
            #print currTry
            # select a random  free position
            q_pos = self._getRandomFreePosition()
            if self.statsHandler:
                self.statsHandler.addNodeAtPos(nodeNumber, q_pos)

            g_vis = None
        
            # every connected component represents one guard
            merged = False
            for comp in nx.connected_components(self.graph): # Impliciteley represents G_vis
                found = False
                merged = False
                for g in comp: # connected components consists of guards and connection: only test nodes of type 'Guards'
                    if 'nodeType' in self.graph.nodes()[g] and self.graph.nodes()[g]['nodeType'] == 'Guard':
                        if self.statsHandler:
                            self.statsHandler.addVisTest(nodeNumber, g)
                        if self._isVisible(q_pos,self.graph.nodes()[g]['pos']):
                            found = True
                            if g_vis == None:
                                g_vis = g
                            else:
                                self.graph.add_node(nodeNumber, pos = q_pos, color='lightblue', nodeType = 'Connection')
                                self.graph.add_edge(nodeNumber, g)
                                self.graph.add_edge(nodeNumber, g_vis)
                                #print "ADDED Connection node", nodeNumber
                                merged = True
                        # break, if node was visible,because visibility from one node of the guard is sufficient...
                        if found == True:
                            break;
                # break, if connection was found. Reason: computed connected components (comp) are not correct any more, 
                # they've changed because of merging
                if merged == True: # how  does it change the behaviour? What has to be done to keep the original behaviour?
                    break;                    

            if (merged==False) and (g_vis == None):
                self.graph.add_node(nodeNumber, pos = q_pos, color='red', nodeType = 'Guard') # Adding guards
                #print "ADDED Guard ", nodeNumber
                currTry = 0
            else:
                currTry += 1

            nodeNumber += 1

    @IPPerfMonitor
    def planPath(self, startList, goalList, config):
        """
        
        Args:
            start (array): start position in planning space
            goal (array) : goal position in planning space
            config (dict): dictionary with the needed information about the configuration options
            
        Example:
        
            config["ntry"] = 40 
        
        """
        # 0. reset
        self.graph.clear()
        self.statsHandler.graph.clear()
        
        # 1. check start and goal whether collision free (s. BaseClass)
        checkedStartList, checkedGoalList = self._checkStartGoal(startList,goalList)
        
        # 2. Check if start and goal can see each other
        self.statsHandler.addNodeAtPos(0, checkedStartList[0])
        self.statsHandler.addNodeAtPos(1, checkedGoalList[0])
        # self.statsHandler.addVisTest(0, 1)
        if self._isVisible(checkedStartList[0], checkedGoalList[0]):
            self.graph.add_node("start", pos=checkedStartList[0], color='lightgreen')
            self.graph.add_node("goal", pos=checkedGoalList[0], color='lightgreen')
            self.graph.add_edge("start", "goal")
            path = nx.shortest_path(self.graph,"start","goal")
            #formatted_path = ["-{}-{}-".format(self.graph.nodes[node]['pos'][0], self.graph.nodes[node]['pos'][1]) for node in ["start", "goal"]]
            print(f"Path found instantly due to visibility of start and goal: {path}")
            return path
        else:
            self.graph.add_node("start", pos=checkedStartList[0], color='red', nodeType = 'Guard')
            self.graph.add_node("goal", pos=checkedGoalList[0], color='red', nodeType = 'Guard')
            print(f"Start and goal are not visible to each other")
            
            # Aufteilen der ntry in kleinere Schritte
            step_size = 10  # Anzahl der Versuche pro Schritt
            for i in range(0, config["ntry"]//step_size):
                self._learnRoadmap(step_size)

                try:
                    print(f"X Koordinate Start: {self.graph.nodes['start']['pos'][0]}\nY Koordinate Start: {self.graph.nodes['start']['pos'][1]}")
                    test_path = nx.shortest_path(self.graph, "start", "goal")                    
                    print(f"Testpath found in iteration {i}: {test_path}")
                    # connect all nodes that can see each other - for shorter paths with the same nodes
                    for node in self.graph.nodes():
                        if self.graph.nodes[node].get('NodeType') != 'Guard':
                            for other_node in self.graph.nodes():
                                if node != other_node and self.graph.nodes[other_node].get('NodeType') != 'Guard':
                                    if self._isVisible(self.graph.nodes[node]['pos'], self.graph.nodes[other_node]['pos']):
                                        self.graph.add_edge(node, other_node)

                    path = nx.shortest_path(self.graph, "start", "goal")
                    print(f"Path found in iteration {i}: {path}")

                    return path

                except:
                    print(f"No path found in {i} iteration")
                    continue

            print("No path found after all iterations")
            return []
    

    @IPPerfMonitor
    def createGraph(self, startList, goalList, config):
        print("ist in der methode")
        # 0. reset
        self.graph.clear()
        self.statsHandler.graph.clear()
        print("Graph und StatsHandler wurden zurückgesetzt")
        
        # 1. check start and goal whether collision free (s. BaseClass)
        checkedStartList, checkedGoalList = self._checkStartGoal(startList,goalList)
        print(f"Start und Ziel überprüft: {checkedStartList}, {checkedGoalList}")

        # 2. Check if start and goal can see each other
        self.statsHandler.addNodeAtPos(0, checkedStartList[0])
        for i in range(0, len(checkedGoalList)):
            self.statsHandler.addNodeAtPos(i+1, checkedGoalList[i])
        print("Start- und Zielknoten zur StatsHandler hinzugefügt")
        
        all_nodes_are_visible = True
        for i in range(len(self.statsHandler.graph.nodes())-1):
            if self._isVisible(self.statsHandler.graph.nodes[i]['pos'], self.statsHandler.graph.nodes[i+1]['pos']):
                continue
            else:
                all_nodes_are_visible = False
                break
        print(f"Alle Knoten sind sichtbar: {all_nodes_are_visible}")
    
        self.graph.add_node("start", pos=checkedStartList[0], color='lightgreen')
        print("Startknoten zum Graph hinzugefügt")

        for i in range(0, len(checkedGoalList)):
            self.graph.add_node(f"goal_{i+1}", pos=checkedGoalList[i], color='lightgreen')
            if i > 0 and all_nodes_are_visible == True:
                self.graph.add_edge(f"goal_{i}", f"goal_{i+1}")
        print("Zielknoten zum Graph hinzugefügt")
        
        if all_nodes_are_visible == True:
            print("Alle Knoten sind sichtbar, Graph wird zurückgegeben")
            return self.graph
        
        # if start and goal are not visible to each other build roadmap
        else:
            print("Start und Ziel sind nicht sichtbar, Roadmap wird erstellt")
            # Aufteilen der ntry in kleinere Schritte
            step_size = 10  # Anzahl der Versuche pro Schritt
            for i in range(0, config["ntry"]//step_size):
                print(f"Roadmap Lernschritt {i}")
                self._learnRoadmap(step_size)

                print("Knoten im Graphen:")
                print(self.graph.nodes())  # Gibt alle Knoten und Attribute aus

                print("\nKanten im Graphen:")
                print(self.graph.edges())  # Gibt alle Verbindungen aus


                try:
                    # 3. find connection of start and goal to roadmap
                    # find nearest, collision-free connection between node on graph and start
                    
                    # Liste der Knoten erstellen
                    NotRoadmap = ["start"]
                    for i in range(len(checkedGoalList)):
                        NotRoadmap.append(f"goal_{i+1}")
                    print("Liste der Knoten im Graph (NotRoadmap):", NotRoadmap)
                    
                    posList = nx.get_node_attributes(self.graph,'pos')
                    print("PosList:", posList)
                    kdTree = cKDTree(list(posList.values()))
                    
                    print(checkedStartList[0])

                    result = kdTree.query(checkedStartList[0],k=5)
                    print("result:", result)
                    for node in result[1]:
                        target_node = list(posList.keys())[node]
                        if target_node not in NotRoadmap:   
                            if not self._collisionChecker.lineInCollision(checkedStartList[0],self.graph.nodes()[list(posList.keys())[node]]['pos']):
                                self.graph.add_node("start", pos=checkedStartList[0], color='lightgreen')
                                self.graph.add_edge("start", list(posList.keys())[node])
                                print("Startknoten mit Roadmap verbunden")
                                break

                    for i in range(0, len(checkedGoalList)):
                        result = kdTree.query(checkedGoalList[i],k=5)
                        for node in result[1]:
                            target_node = list(posList.keys())[node]
                            if target_node not in NotRoadmap:
                                if not self._collisionChecker.lineInCollision(checkedGoalList[i],self.graph.nodes()[list(posList.keys())[node]]['pos']):
                                    self.graph.add_node(f"goal_{i+1}", pos=checkedGoalList[i], color='lightgreen')
                                    self.graph.add_edge(f"goal_{i+1}", list(posList.keys())[node])
                                    print(f"Zielknoten {i} mit Roadmap verbunden")
                                    break

                    #print("\nKanten im Graphen:")
                    #print(self.graph.edges())  # Gibt alle Verbindungen aus
                    #print("Start-Knoten in Graph:", "start" in self.graph)
                    #print("Ziel-Knoten in Graph:", "goal_1" in self.graph)
                    #print("Start-Knoten Nachbarn:", list(self.graph.neighbors("start")))
                    #print("Goal_1 Nachbarn:", list(self.graph.neighbors("goal_1")))


            
                    try:
                        test_path = nx.shortest_path(self.graph,"start","goal_1")
                        print ('erster testpfad wurde gefunden')
                        for i in range(0, len(checkedGoalList)-1):
                            test_path = nx.shortest_path(self.graph,f"goal_{i+1}",f"goal_{i+2}")
                            print("Kürzester Pfad gefunden")
                    except:
                        continue

                    try:                            
                        # connect all nodes that can see each other - for shorter paths with the same nodes
                        for node in self.graph.nodes():
                            if self.graph.nodes[node].get('NodeType') != 'Guard':
                                for other_node in self.graph.nodes():
                                    if node != other_node and self.graph.nodes[other_node].get('NodeType') != 'Guard':
                                        if self._isVisible(self.graph.nodes[node]['pos'], self.graph.nodes[other_node]['pos']):
                                            self.graph.add_edge(node, other_node)
                        print("Alle sichtbaren Knoten verbunden")

                        return self.graph

                    except:
                        print("Kein kürzester Pfad gefunden")
                        continue

                except:
                    print(f"Kein Pfad in Iteration {i} gefunden")
                    continue

            print("Kein Pfad nach allen Iterationen gefunden")
            return []