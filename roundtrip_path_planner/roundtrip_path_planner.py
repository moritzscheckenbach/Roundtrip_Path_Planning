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
mehr nötig sind. Beispiel steht unten (zu finden im Automated_PlanerTest)
"""

# basicConfig = dict()
# basicConfig["radius"] = 3
# basicConfig["numNodes"] = 200
# plannerFactory["basePRM"] = [IPBasicPRM.BasicPRM, basicConfig, IPVISBasicPRM.basicPRMVisualize]


##############
# Roundtrip Path Planner
##############

class Roundtrip_Path_Planner:
    def __init__(self, startpos, targetlist, environment, algorithm, config, collChecker):
        
        # Initialize all necessary members
        self.startpos = startpos # start position in configuration space
        self.targetlist = targetlist # list of goal positions in configuration space
        self.environment = environment # reference to Environment
        self.algorithm = algorithm # reference to algorithm
        self.config = config # dictionary with the needed information about the algorithms configuration options
        self.collChecker = collChecker # reference to CollisionChecker

        
