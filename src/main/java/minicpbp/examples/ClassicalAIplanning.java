/*
 * mini-cp is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License  v3
 * as published by the Free Software Foundation.
 *
 * mini-cp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY.
 * See the GNU Lesser General Public License  for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with mini-cp. If not, see http://www.gnu.org/licenses/lgpl-3.0.en.html
 *
 * Copyright (c)  2018. by Laurent Michel, Pierre Schaus, Pascal Van Hentenryck
 *
 * mini-cpbp, replacing classic propagation by belief propagation
 * Copyright (c)  2019. by Gilles Pesant
 */

package minicpbp.examples;

import minicpbp.engine.core.IntVar;
import minicpbp.engine.core.Solver;
import minicpbp.examples.ClassicalAIPlanning.*;
import minicpbp.search.DFSearch;
import minicpbp.search.Objective;
import minicpbp.search.Search;
import minicpbp.search.SearchStatistics;
import minicpbp.util.Procedure;
import minicpbp.util.exception.InconsistencyException;

import java.util.HashMap;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static minicpbp.cp.BranchingScheme.*;
import static minicpbp.cp.Factory.*;

/**
 * A generic CP-BP model&solve approach to classical AI planning
 */
public class ClassicalAIplanning {

    public static int currentBestPlanCost;
    public static int timeout = 1000; // to look for a plan of given length, in milliseconds
    public static int failout = 100; // to look for a plan of given length

    public static void main(String[] args) {

        HashMap<String,String> opt = ArgumentsReader.readArguments(args);

        String step;
        printInfoMessage(step = "Starting");

        ExperimentResources exeStats = new ExperimentResources();
        exeStats.printMaxHeapMemory();
        exeStats.printStats("Base Memory Usage");

        // ============================
        printInfoMessage(step = "Reading instance");
        PlanningInstance instance = new PlanningInstance(opt.get("instance"));
        exeStats.printStats(step);

        // ============================
        printInfoMessage(step = "Start Search loops");
        long totalTime = 0;
        final SolutionWriter solutionWriter = new SolutionWriter(true, opt.get("output"));
        currentBestPlanCost = instance.getUpperBoundCost();
        try {
            // try plans of increasing length
            for (int length = instance.minPlanLength; length <= instance.maxPlanLength; length++) {
                if (currentBestPlanCost <= instance.getLowerBoundCost(length)) {
                    break; // current best is at least as good as lower bound on plan cost from that length on
                }
                System.out.println("—————————————————————————————————");
                System.out.println("Search for plans of length: " + length);
                try {

                    // ============================
                    step = "Defining the CP model";
                    PlanningModel model = new PlanningModel(instance, length, currentBestPlanCost);
                    Solver cp = model.getSolver();
//                    cp.setTraceSearchFlag(true);
                    IntVar[] action = model.getAction();
                    IntVar planCost = model.getPlanCost();
                    exeStats.printStats("CP model init");

                    // ============================
                    step = "Defining the search";
                    Search search;
                    Supplier<Procedure[]> branching = null;
                    switch (opt.get("branching")){
                        case "maxMarginal":
                            branching = maxMarginal(action);
                            break;
                        case "lexicoMaxMarginalValue":
                            branching = lexicoMaxMarginalValue(action);
                            break;
                        case "firstFailMaxMarginalValue":
                            branching = firstFailMaxMarginalValue(action);
                            break;
                        case "domWdegMaxMarginalValue":
                            branching = domWdegMaxMarginalValue(action);
                            break;
                        case "maxMarginalStrength":
                            branching = maxMarginalStrength(action);
                            break;
                        case "minEntropy":
                            branching = minEntropy(action);
                            break;
                        case "domWdeg":
                            if (Objects.equals(opt.get("search"), "lds")){
                                throw new RuntimeException("Cant use lds with domWdeg");
                            }
                            branching = domWdeg(action);
                            cp.setMode(Solver.PropaMode.SP);
                            break;
                        default:
                            throw new IllegalArgumentException(
                                    String.format("Branching heuristic '%s' not recognized", opt.get("branching")));
                    }

                    if (Objects.equals(opt.get("search"), "lds"))
                        search = makeLds(cp, branching);
                    else if (Objects.equals(opt.get("search"), "dfs"))
                        search = makeDfs(cp, branching);
                    else throw new IllegalArgumentException("Missing search option");

                    System.out.format("Solving with '%s' search and '%s' branching heuristic\n",
                            opt.get("search"), opt.get("branching"));

                    AtomicReference<Double> lastMeasurementTime = new AtomicReference<>(exeStats.elapsedCPUTime());
                    cp.onFixPoint(() -> {
                        if (lastMeasurementTime.get() + 30 < exeStats.elapsedCPUTime()){
                            exeStats.printStats("Searching");
                            lastMeasurementTime.set(exeStats.elapsedCPUTime());
                        }
                    });

                    search.onSolution(() -> {
                        solutionWriter.newSolution(action, planCost.min(), exeStats.elapsedCPUTimeStr());
                        exeStats.printStats("Solution found");
                        currentBestPlanCost = planCost.min();
                    });

                    // ============================
                    step = "Searching plans";
                    Objective obj = cp.minimize(planCost);
                    SearchStatistics stats;
                    if(Objects.equals(opt.get("branching"), "domWdeg")){
                        assert search instanceof DFSearch;
                        stats = ((DFSearch) search).optimizeRestarts(obj, statistics -> false);
                    }
                    else {
                        stats = search.optimize(obj,statistics -> false);
                    }
                    System.out.println("Search over: " + stats.longString());
                    totalTime += stats.timeElapsed();

                } catch (InconsistencyException e) {
                    System.out.println("No solution (Search inconsistent)");
                } finally {
                    System.out.println();
                }
            }
            solutionWriter.close();

            if (solutionWriter.nSolutions() == 0){
                printOutcome("unsolvable");
            } else {
                printOutcome("solved-opt");
            }
        } catch (OutOfMemoryError error){
            System.out.println("MiniCPBP ran in an OutOfMemoryError while : " + step);
            printOutcome("out-of-memory");
        }
        printInfoMessage("CPU time: " + exeStats.elapsedCPUTimeStr());
        printInfoMessage("Wall time: " + exeStats.elapsedWallTime() );
        printInfoMessage("accumulated search time : "+totalTime*1e-3+" s");
    }

    private static void nlPrintInfoMessage(String message) {
        System.out.println("\n[MiniCPBP] " + message);
    }

    private static void printInfoMessage(String message) {
        System.out.println("[MiniCPBP] " + message);
    }

    private static void printOutcome(String message){
        printInfoMessage("CP solver outcome: " + message);
    }

}
