# coding: utf-8

"""
This is the main file for the roundtrip path planner.

Args:
startpos (array): start position in configuration space
targetlist (list): list of goal positions (arrays) in configuration space
environment: the environment in which the path should be planned

"""

################################
# Import the necessary libraries
################################

import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import heapq
import math

import algorithms.IPAStar as IPAStar
import algorithms.IPBasicPRM as IPBasicPRM
import algorithms.IPLazyPRM as IPLazyPRM
import algorithms.IPPRMBase as IPPRMBase
import algorithms.IPRRT as IPRRT
import algorithms.IPVisibilityPRM as IPVisibilityPRM

import visualizations.IPVISBasicPRM as IPVISBasicPRM
import visualizations.IPVISLazyPRM as IPVISLazyPRM
import visualizations.IPVISVisibilityPRM as IPVISVisibilityPRM
import visualizations.IPVISAStar as IPVISAStar
import visualizations.IPVISRRT as IPVISRRT

import utils.IPPerfMonitor as IPPerfMonitor
import utils.IPBenchmark as IPBenchmark
import utils.IPEnvironment as Environment
import utils.IPTestSuiteSS2023 as IPTestSuiteSS2023
import utils.IPPlanerBase as IPPlanerBase 


###########################
# structure for the results
###########################

class ResultCollection (object):
    
    def __init__(self, plannerFactoryName, planner, benchmark, solution, perfDataFrame):
        self.plannerFactoryName = plannerFactoryName
        self.planner = planner
        self.benchmark = benchmark
        self.solution = solution
        self.perfDataFrame = perfDataFrame


########################
# Roundtrip Path Planner
########################

class Roundtrip_Path_Planner:
    def __init__(self, startpos, targetlist, environment, planner):
        
        # Initialize all necessary members
        self.startpos = startpos # start position in configuration space
        self.targetlist = targetlist # list of goal positions in configuration space
        self.environment = environment # reference to Environment
        self.planner = planner # reference to algorithm
        self.config = self.getplannerconfig # dictionary with the needed information about the algorithms configuration options

    def getplannerconfig(self):

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

        return supportedPlanners
        """
        Evtl. nutzbare Abfrage, ob der gewählte Algorithmus unterstützt wird
        und welche Konfigurationen zur Verfügung stehen
        """
        # if not self.planner in supportedPlanners:
        #     raise ValueError("Planner currently not supported")
        # else:
        #     config = supportedPlanners[self.planner]
        #     print(f"Using planner: {self.planner}")
        #     print(f"found default config: {config} \n Do you want to use this config? (y/n)")
        #     answer = input()

        #     if answer == "n":
        #         print("Please enter the new config \n ")
        #         new_config = {}
        #         for key in config[1]:
        #             new_value = input(f"Enter value for {key} (default: {config[1][key]}): ")
        #             if new_value == "":
        #                 new_config[key] = config[1][key]
        #             else:
        #                 new_config[key] = type(config[1][key])(new_value)
        #         print(f"New configuration: {new_config}")
        #         print(f"Is this configuration correct? (y/n)")
        #         answer = input()

        #         if answer == "n":
        #             print("Using default config")
        #             return config
        #         else:
        #             print("Using new config")                
        #             return new_config
                
        #     else:
        #         print("Using default config")
        #         return config

    def plan(self):
        resultList = list()
        key = self.planner # key enthält den Nemen des Planers
        producer = self.config[key] # producer enthält alle items der config des Planers
        print(key, producer) # Key ist der Planer, producer ist der Konstruktor (Vorher definierte Configs [siehe oben])

        print ("Planning: " + key + " - " + self.environment.name)
        planner = producer[0](self.environment.collisionChecker) # Aufruf erstes Element aus producer (Klasse des Planers) und Aufruf der Methode Benchmark.collisionChecker
        IPPerfMonitor.clearData() # Löscht die Daten aus dem Dataframe des Performance Monitors
        
        """TBD: Evtl. Anpassung des Auswahlverfahrens für das nächste Ziel"""
        usedstart = self.startpos
        goals = self.targetlist
        usedgoal = None

        min_distance = float('inf')
        for goal in goals:
            distance = math.sqrt((usedstart[0] - goal[0]) ** 2 + (usedstart[1] - goal[1]) ** 2)
            if distance < min_distance:
                min_distance = distance
                usedgoal = goal
        

            try:
                
                resultList.append(ResultCollection(key,
                                                planner, 
                                                self.environment, 
                                                planner.planPath(usedstart,usedgoal,producer[1]), # Aufruf der Methode planPath des Planers
                                                IPPerfMonitor.dataFrame()
                                                ),
                            )
            except Exception as e:
            #    throw e
                print ("PLANNING ERROR ! PLANNING ERROR ! PLANNING ERROR ")
                pass


######
# Test
######

fullBenchList = IPTestSuiteSS2023.benchList
testList = fullBenchList[0]

Roundtrip_Path_Planner(testList.startList, testList.goalList, testList.environment ,"basePRM").plan()