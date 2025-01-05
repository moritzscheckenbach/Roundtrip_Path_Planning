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

import IPAStar
import IPBasicPRM
import IPLazyPRM
import IPPRMBase
import IPRRT
import IPVisibilityPRM

import IPVISBasicPRM
import IPVISLazyPRM
import IPVISVisibilityPRM
import IPVISAStar
import IPVISRRT

from IPPerfMonitor import IPPerfMonitor


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
        
        # Initialize all necessary members
        self.startpos = startpos # start position in configuration space
        self.targetlist = targetlist # list of goal positions in configuration space
        self.environment = environment # reference to Environment
        self.planner = planner # reference to algorithm
        self.config = supportedPlanners # dictionary with the needed information about the algorithms configuration options

    def plan(self):
        resultList = list()
        key = self.planner # key enthält den Nemen des Planers
        producer = self.config[key] # producer enthält alle items der config des Planers
        print(key, producer) # Key ist der Planer, producer ist der Konstruktor (Vorher definierte Configs [siehe oben])

        print ("Planning: " + key + " - " + self.environment.name)
        planner = producer[0](self.environment.collisionChecker) # Aufruf erstes Element aus producer (Klasse des Planers) und Aufruf der Methode Benchmark.collisionChecker
        IPPerfMonitor.clearData() # Löscht die Daten aus dem Dataframe des Performance Monitors
        
        """TBD: Evtl. Anpassung des Auswahlverfahrens für das nächste Ziel"""
        usedstart = self.startpos[0]
        goals = self.targetlist
        pastgoals = []

        for i in range(len(goals)):
            min_distance = None
            min_distance = float('inf')

            for goal in goals:
                if goal not in pastgoals:
                    distance = math.sqrt((usedstart[0] - goal[0]) ** 2 + (usedstart[1] - goal[1]) ** 2)
                    if distance < min_distance:
                        min_distance = distance
                        goal_gespeichert = goal
            
            # Aktualisieren des Startpunktes und der Liste der bereits besuchten Punkte
            usedstart = goal_gespeichert # Alter Zielpunkt wird zum neuen Startpunkt
            pastgoals.append(goal_gespeichert) # Zielpunkt wird in die Liste der bereits besuchten Punkte eingetragen

        # Pfadplanung für die besuchten Ziele
        usedstart = self.startpos[0]
        for i in range(len(pastgoals)):
            print(f"Pastgoals: {pastgoals}")
            print(f"Usedstart: {usedstart}")
            tmp_1 = []
            tmp_1.append(pastgoals[i])
            tmp_2 = []
            tmp_2.append(usedstart)
            
            print(f"tmp_1: {tmp_1}")
            print(f"tmp_2: {tmp_2}")


            try:
                resultList.append(ResultCollection(key,
                                                planner, 
                                                self.environment, 
                                                planner.planPath(tmp_2,tmp_1,producer[1]), # Aufruf der Methode planPath des Planers
                                                IPPerfMonitor.dataFrame()
                                                ),
                            )
            except Exception as e:
                print ("PLANNING ERROR ! PLANNING ERROR ! PLANNING ERROR ")
                pass
            usedstart = pastgoals[i]
            print(f"New Usedstart: {usedstart}")


