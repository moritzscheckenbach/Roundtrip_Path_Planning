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
        
class VisPRM_Custom_4(PRMBase):
    """Class implements an simplified version of a visibility PRM"""

    def __init__(self, _collChecker, _statsHandler = None):
        super(VisPRM_Custom_4, self).__init__(_collChecker)
        self.graph = nx.Graph()
        self.statsHandler = VisibilityStatsHandler() # not yet fully customizable (s. parameters of constructors)
                
    def _isVisible(self, pos, guardPos):
        return not self._collisionChecker.lineInCollision(pos, guardPos)

    @IPPerfMonitor
    def _learnRoadmap(self, ntry):

        nodeNumber = 2
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
                    if self.graph.nodes()[g]['nodeType'] == 'Guard':
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
                        if found == True: break;
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
        print(f"Checked Start List: {checkedStartList}")
        print(f"Checked Goal List: {checkedGoalList}")

        self.graph.add_node("start", pos=checkedStartList[0], color='red', nodeType = 'Guard')

        
        # 2. Check if start and goal can see each other
        directly_visible = True

        self.statsHandler.addNodeAtPos(0, checkedStartList[0])
        for i in range(len(checkedGoalList)):
            self.statsHandler.addNodeAtPos(i+1, checkedGoalList[i])
            self.statsHandler.addVisTest(i, i+1)
            self.graph.add_node("goal"+str(i+1), pos=checkedGoalList[i], color='red', nodeType = 'Guard')

            Nodes_in_Graph = nx.get_node_attributes(self.statsHandler.graph,'pos')
            Name_in_Graph = list(self.graph.nodes)
            Nodepositions = nx.get_node_attributes(self.graph,'pos')
            print(f"Nodepositions: {Nodepositions}")
            print(f"Nodes in Graph: {Nodes_in_Graph}")
            print(f"Name in Graph: {Name_in_Graph}")
            if not self._isVisible(Nodes_in_Graph[i], Nodes_in_Graph[i+1]):
                directly_visible = False
            
            else:
                # self.graph.add_node("goal"+str(i+1), pos=checkedGoalList[i], color='lightgreen')
                print("Test 2")
                self.graph.add_edge(Name_in_Graph[i], Name_in_Graph[i+1]) # Aktuell nicht benötigt, weil an anderer Stelle bereits implementiert

        if directly_visible:
            print("Directly visible")
            # Wenn sich alle Punkte direkt sehen können, kann man sich das planen sparen
            self.graph.add_node("start", pos=checkedStartList[0], color='lightgreen')
            for i in range(len(checkedGoalList)):
                self.graph.add_node("goal"+str(i+1), pos=checkedGoalList[i], color='lightgreen')
            
            self.graph.add_edge("start", "goal"+str(1))
            for i in range(len(checkedGoalList)-1):
                self.graph.add_edge("goal"+str(i+1), "goal"+str(i+2))

        else:
            print("Not directly visible")
            # 3. If not, learn roadmap
            self._learnRoadmap(config["ntry"])

            # 4. Find path
            try:
                path = nx.shortest_path(self.graph,"start","goal"+str(len(checkedGoalList)))
            except:
                print("No path found")
                return []


        # if self._isVisible(checkedStartList[0], checkedGoalList[0]):
        #     self.graph.add_node("start", pos=checkedStartList[0], color='lightgreen')
        #     self.graph.add_node("goal", pos=checkedGoalList[0], color='lightgreen')
        #     self.graph.add_edge("start", "goal")
        #     path = nx.shortest_path(self.graph,"start","goal")
        #     #formatted_path = ["-{}-{}-".format(self.graph.nodes[node]['pos'][0], self.graph.nodes[node]['pos'][1]) for node in ["start", "goal"]]
        #     return path
        # else:
        #     self.graph.add_node("start", pos=checkedStartList[0], color='red', nodeType = 'Guard')
        #     self.graph.add_node("goal", pos=checkedGoalList[0], color='red', nodeType = 'Guard')

        #     self._learnRoadmap(config["ntry"])

        #     try:
        #         path = nx.shortest_path(self.graph,"start","goal")
        #         #formatted_path = ["-{}-{}-".format(self.graph.nodes[node]['pos'][0], self.graph.nodes[node]['pos'][1]) for node in path]
        #     except:
        #         print("No path found")
        #         return []
        #     #print(f"Self.graph ist {self.graph}")
        #     #print(f"Solution_Visibility ist {path}")
        #     return path