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

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static minicpbp.cp.BranchingScheme.*;
import static minicpbp.cp.Factory.*;

/**
 * A generic CP-BP model&solve approach to classical AI planning
 */
public class ClassicalAIplanningNoOp {

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

        String lp = null;
        if(opt.containsKey("lp")){
            try {
                lp = new String(Files.readAllBytes(Paths.get(opt.get("lp"))));
            } catch (IOException e){
                System.out.println("Error while reading the lp problem.");
                e.printStackTrace();
                System.exit(1);
            }
        }

        // ============================
        printInfoMessage(step = "Start Search loops");
        long totalTime = 0;
        final SolutionWriter solutionWriter = new SolutionWriter(true, opt.get("output"));
        currentBestPlanCost = instance.getUpperBoundCost();
        try {
            // try plans of increasing length
            System.out.println("—————————————————————————————————");
            System.out.println("Search for plans between length: " + instance.minPlanLength + " and " + instance.maxPlanLength);
            try {
                // ============================
                step = "Defining the CP model";
                PlanningModelNoOp model = new PlanningModelNoOp(instance, instance.maxPlanLength, instance.minPlanLength, lp);
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
                    case "lexico":
                        branching = lexico(action);
                        cp.setMode(Solver.PropaMode.SP);
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
//                if(Objects.equals(opt.get("branching"), "domWdeg")){
//                    assert search instanceof DFSearch;
//                    stats = ((DFSearch) search).optimizeRestarts(obj, statistics -> false);
//                }
//                else {
//                    stats = search.optimize(obj,statistics -> false);
//                }
                stats = search.optimize(obj,statistics -> false);
                System.out.println("Search over: " + stats.longString());
                totalTime += stats.timeElapsed();

            } catch (InconsistencyException e) {
                System.out.println("No solution (Search inconsistent)");
            } finally {
                System.out.println();
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
