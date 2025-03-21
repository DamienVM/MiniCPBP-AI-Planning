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
import minicpbp.search.Objective;
import minicpbp.search.Search;
import minicpbp.search.SearchStatistics;
import minicpbp.util.exception.InconsistencyException;

import java.lang.management.ManagementFactory;
import java.util.HashMap;
import java.util.Objects;

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

        final long startCpuTime = ManagementFactory.getThreadMXBean().getThreadCpuTime(Thread.currentThread().getId());
        final long startWallTime = System.currentTimeMillis();

        PlanningMemoryUsage memoryUsage = new PlanningMemoryUsage();
        memoryUsage.printMaxHeapMemory();
        memoryUsage.printUsage("Base Memory Usage");

        // ============================
        nlPrintInfoMessage(step = "Reading instance");
        PlanningInstance instance = new PlanningInstance(opt.get("instance"));
        memoryUsage.printUsage(step);

        // ============================
        nlPrintInfoMessage(step = "Start Search loops");
        long totalTime = 0;
        final SolutionWriter solutionWriter = new SolutionWriter(true, opt.get("output"));
        currentBestPlanCost = instance.getUpperBoundCost();
        try {
            // try plans of increasing length
            for (int length = instance.minPlanLength; length <= instance.maxPlanLength; length++) {
                if (currentBestPlanCost <= instance.getLowerBoundCost(length)) {
                    break; // current best is at least as good as lower bound on plan cost from that length on
                }
                System.out.println("_________________________________");
                System.out.println("Search for plans of length : " + length);
                try {

                    // ============================
                    step = "Defining the CP model";
                    PlanningModel model = new PlanningModel(instance, length, currentBestPlanCost);
                    Solver cp = model.getSolver();
                    IntVar[] action = model.getAction();
                    IntVar planCost = model.getPlanCost();
                    memoryUsage.printUsage("CP model");

                    // ============================
                    step = "Defining the search";
                    Search search;
                    if (Objects.equals(opt.get("search"), "lds"))
                        search = makeLds(cp, maxMarginal(action));
                    else if (Objects.equals(opt.get("search"), "dfs"))
                        search = makeDfs(cp, maxMarginal(action));
                    else throw new IllegalArgumentException("Missing search option");

                    search.onSolution(() -> {
                        solutionWriter.newSolution(action, planCost.min());
                        currentBestPlanCost = planCost.min();
                        memoryUsage.printUsage("Solution");
                    });

                    // ============================
                    step = "Searching plans";
                    Objective obj = cp.minimize(planCost);
                    //            SearchStatistics stats = search.optimize(obj, statistics -> (statistics.timeElapsed() >= timeout && statistics.numberOfFailures() >= failout));
                    SearchStatistics stats = search.optimize(obj,statistics -> false);
                    //            SearchStatistics stats = search.solve();
                    System.out.println("== End plan search of length : " + length);
                    System.out.format("== Search Statistics: %s", stats);
                    totalTime += stats.timeElapsed();

                } catch (InconsistencyException e) {
                    System.out.println("== No solution (Search inconsistent)");
                } finally {
                    System.out.println();
                }
            }
            solutionWriter.close();
            printInfoMessage("Search ended nicely");
        } catch (OutOfMemoryError error){
            System.out.println("MiniCPBP ran in an OutOfMemoryError while : " + step);
        }
        printInfoMessage("full CPU time : " + (ManagementFactory.getThreadMXBean().getThreadCpuTime(Thread.currentThread().getId()) - startCpuTime )*1e-9+" s");
        printInfoMessage("full wall time : " + (System.currentTimeMillis()-startWallTime)*1e-3+" s");
        printInfoMessage("accumulated search time : "+totalTime*1e-3+" s");
    }

    private static void nlPrintInfoMessage(String message) {
        System.out.println("\n[MiniCPBP] " + message);
    }

    private static void printInfoMessage(String message) {
        System.out.println("[MiniCPBP] " + message);
    }


}
