# coding: utf-8

"""
This is the main file for the roundtrip path planner.


"""

################################
# Import the necessary libraries
################################

import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import heapq
import math


#######################
# Planner Configuration
#######################

"""
An dieser Stelle müssen für alle Algorithmen die Konfigurationen vordefiniert werden,
sodass sie später in der GUI ausgewählt werden können und keine weiteren Parameter
mehr nötig sind.

Kopiert aus IP-X-0-Automated_PlanerTest.ipynb
"""

supportedPlanners = dict()

basicConfig = dict()
basicConfig["radius"] = 3
basicConfig["numNodes"] = 200
supportedPlanners["basePRM"] = [IPBasicPRM.BasicPRM, basicConfig, IPVISBasicPRM.basicPRMVisualize]

basicConfig2 = dict()
basicConfig2["radius"] = 6
basicConfig2["numNodes"] = 600
supportedPlanners["basePRM2"] = [IPBasicPRM.BasicPRM, basicConfig2, IPVISBasicPRM.basicPRMVisualize]

visbilityConfig = dict()
visbilityConfig["ntry"] = 300
supportedPlanners["visibilityPRM"] = [IPVisibilityPRM.VisPRM, visbilityConfig, IPVISVisibilityPRM.visibilityPRMVisualize ]

# kClosestConfig = dict()
# kClosestConfig["k"] = 7
# kClosestConfig["numNodes"] = 300
# supportedPlanners["kClosestPRM"] = [IPKClosestPRM.KClosestPRM, kClosestConfig, IPVISBasicPRM.basicPRMVisualize]

lazyConfig = dict()
lazyConfig["initialRoadmapSize"] = 10
lazyConfig["updateRoadmapSize"]  = 5 
lazyConfig["kNearest"] = 8
supportedPlanners["lazyPRM"] = [IPLazyPRM.LazyPRM, lazyConfig, IPVISLazyPRM.lazyPRMVisualize]

astarConfig = dict()
astarConfig["heuristic"] = 'euclidean' 
astarConfig["w"]  = 0.5
supportedPlanners["astar"] = [IPAStar.AStar, astarConfig, IPVISAStar.aStarVisualize]

# astarConfig2 = dict()
# astarConfig2["heuristic"] = 'euclidean' 
# astarConfig2["w"]  = 0.9
# supportedPlanners["astar2"] = [IPAStar.AStar, astarConfig2, IPVISAStar.aStarVisualize]

# rrtSimpleConfig = dict()
# rrtSimpleConfig["numberOfGeneratedNodes"] = 100 
# rrtSimpleConfig["testGoalAfterNumberOfNodes"]  = 10
# supportedPlanners["simpleRRT"] = [IPRRT.RRTSimple, rrtSimpleConfig, IPVISRRT.rrtPRMVisualize]


########################
# Roundtrip Path Planner
########################

class Roundtrip_Path_Planner:
    def __init__(self, startpos, targetlist, environment, planner, config, collChecker):
        
        # Initialize all necessary members
        self.startpos = startpos # start position in configuration space
        self.targetlist = targetlist # list of goal positions in configuration space
        self.environment = environment # reference to Environment
        self.planner = planner # reference to algorithm
        self.config = config # dictionary with the needed information about the algorithms configuration options
        self.collChecker = collChecker # reference to CollisionChecker

    def checkPlanner(planner):
        if not planner in supportedPlanners:
            raise ValueError("Planner currently not supported")
