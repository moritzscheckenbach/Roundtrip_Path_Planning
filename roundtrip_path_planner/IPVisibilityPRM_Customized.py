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
                self.graph.add_node(nodeNumber, pos = q_pos, color='red', nodeType = 'Guard') # Add the first guard -> Obsolete when Start and Goal are added as guards in the beginning
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
        print("Test_1")
        # 0. reset
        self.graph.clear()
        
        # 1. check start and goal whether collision free (s. BaseClass)
        checkedStartList, checkedGoalList = self._checkStartGoal(startList,goalList)
        
        print("Test_2")
        # 2. Check if start and goal can see each other
        self.statsHandler.addNodeAtPos(0, checkedStartList[0])
        self.statsHandler.addNodeAtPos(1, checkedGoalList[0])
        self.statsHandler.addVisTest(0, 1)
        print("Test_3")
        if self._isVisible(checkedStartList[0], checkedGoalList[0]):
            print("Test_4")
            self.graph.add_node("start", pos=checkedStartList[0], color='lightgreen')
            print("Test_5")
            self.graph.add_node("goal", pos=checkedGoalList[0], color='lightgreen')
            print("Test_6")
            self.graph.add_edge("start", "goal")
            print("Test_7")
            path = nx.shortest_path(self.graph,"start","goal")
            print("Test_8")
            #formatted_path = ["-{}-{}-".format(self.graph.nodes[node]['pos'][0], self.graph.nodes[node]['pos'][1]) for node in ["start", "goal"]]
            return path
        else:
            self.graph.add_node("start", pos=checkedStartList[0], color='red', nodeType = 'Guard')
            print("Test_9")
            self.graph.add_node("goal", pos=checkedGoalList[0], color='red', nodeType = 'Guard')
            print("Test_10")


            import matplotlib.pyplot as plt

            pos = nx.get_node_attributes(self.graph, 'pos')
            colors = nx.get_node_attributes(self.graph, 'color').values()
            nx.draw(self.graph, pos, node_color=colors, with_labels=True, node_size=500, font_size=10)
            plt.show()

            self._learnRoadmap(config["ntry"])

            import matplotlib.pyplot as plt

            pos = nx.get_node_attributes(self.graph, 'pos')
            colors = nx.get_node_attributes(self.graph, 'color').values()
            nx.draw(self.graph, pos, node_color=colors, with_labels=True, node_size=500, font_size=10)

            # Draw obstacles
            if hasattr(self._collisionChecker, 'drawObstacles'):
                self._collisionChecker.drawObstacles(plt)

            plt.show()


            print("Test_11")
            try:
                path = nx.shortest_path(self.graph,"start","goal")
                #formatted_path = ["-{}-{}-".format(self.graph.nodes[node]['pos'][0], self.graph.nodes[node]['pos'][1]) for node in path]
            except:
                return []
            print(f"Solution_Visibility ist {path}")
            return path