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
        
class VisPRM_Custom_can(PRMBase):
    """Class implements an simplified version of a visibility PRM"""

    def __init__(self, _collChecker, _statsHandler = None):
        super(VisPRM_Custom_can, self).__init__(_collChecker)
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

        # connect all nodes that can see each other - for shorter paths with the same nodes
        for node in self.graph.nodes():
            if self.graph.nodes[node].get('NodeType') != 'Guard':
                for other_node in self.graph.nodes():
                    if node != other_node and self.graph.nodes[other_node].get('NodeType') != 'Guard':
                        if self._isVisible(self.graph.nodes[node]['pos'], self.graph.nodes[other_node]['pos']):
                            self.graph.add_edge(node, other_node)

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
            return path
        else:
            self.graph.add_node("start", pos=checkedStartList[0], color='red', nodeType = 'Guard')
            self.graph.add_node("goal", pos=checkedGoalList[0], color='red', nodeType = 'Guard')

            self._learnRoadmap(config["ntry"])

            try:
                path = nx.shortest_path(self.graph,"start","goal")
                #formatted_path = ["-{}-{}-".format(self.graph.nodes[node]['pos'][0], self.graph.nodes[node]['pos'][1]) for node in path]
            except:
                print("No path found")
                return []
            #print(f"Self.graph ist {self.graph}")
            #print(f"Solution_Visibility ist {path}")
            return path