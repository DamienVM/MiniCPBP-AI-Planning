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

package minicpbp.search;

import minicpbp.state.StateManager;
import minicpbp.util.exception.InconsistencyException;
import minicpbp.util.exception.NotImplementedException;
import minicpbp.util.Procedure;
import minicpbp.engine.core.IntVar;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Predicate;
import java.util.function.Supplier;

/**
 * Limited Discrepancy Search Branch and Bound implementation
 */
public class LDSearch extends Search{

    private Supplier<Procedure[]> branching;
    private Supplier<Procedure[]> LDSbranching;
    private boolean geometric; // true if the sequence of max discrepancies follows a geometric progression (with ratio=2); false if it follows an arithmetic progression (with difference=1)
    private int discrepancyUB;
    private StateManager sm;

    private List<Procedure> solutionListeners = new LinkedList<Procedure>();
    private List<Procedure> failureListeners = new LinkedList<Procedure>();

    /**
     * Creates a Limited Discrepancy Search object with a given branching
     * that defines the search tree dynamically.
     *
     * @param sm the state manager that will be saved and restored
     *           at each node of the search tree
     * @param branching a generator of closures in charge of defining the ordered
     *                  children nodes at each node of the depth-first-search tree.
     *                  When it returns an empty array, a solution is found.
     *                  A backtrack occurs when a {@link InconsistencyException}
     *                  is thrown.
     * @param geometric to indicate whether the progression of maxDiscrepancy is geometric
     * @param discrepancyUB an upper bound on the number of discrepancies in the rightmost branch of a complete search tree
     */
    public LDSearch(StateManager sm, Supplier<Procedure[]> branching, boolean geometric, int discrepancyUB) {
        this.sm = sm;
        this.branching = branching;
	this.geometric = geometric;
	this.discrepancyUB = discrepancyUB;
    }
	
    /**
     * Adds a listener that is called on each solution.
     *
     * @param listener the closure to be called whenever a solution is found
     */
    public void onSolution(Procedure listener) {
        solutionListeners.add(listener);
    }

    /**
     * Adds a listener that is called whenever a failure occurs
     * and the search backtracks.
     * This happensthat when a {@link InconsistencyException} is thrown
     * when executing the closure generated by the branching.
     *
     * @param listener the closure to be called whenever a failure occurs and
     *                 the search need to backtrack
     */
    public void onFailure(Procedure listener) {
        failureListeners.add(listener);
    }

    private void notifySolution() {
        solutionListeners.forEach(s -> s.call());
    }

    private void notifyFailure() {
        failureListeners.forEach(s -> s.call());
    }

    private SearchStatistics solve(SearchStatistics statistics, Predicate<SearchStatistics> limit) {
        sm.withNewState(() -> {
	    int maxDiscrepancy = 1;
            try {
		if (discrepancyUB==0) { // special case of all vars already being fixed
		    LDSbranching = new LimitedDiscrepancyBranching(branching, 0);
		    lds(statistics, limit);
		}
                else 
		    while(maxDiscrepancy <= discrepancyUB) { // nb discrepancies of rightmost branch <= nb vars * (domain size - 1)
			LDSbranching = new LimitedDiscrepancyBranching(branching, maxDiscrepancy);
			// System.out.println("LDS: on search tree with max discrepancy = "+maxDiscrepancy);
			lds(statistics, limit);
			// System.out.println(statistics);
			if (geometric)
			    maxDiscrepancy *= 2;
			else
			    maxDiscrepancy++;
		    }
                statistics.setCompleted();
            } catch (StopSearchException ignored) {
 		//System.out.println("c LDS: currently on search tree with max discrepancy = "+maxDiscrepancy);
            } catch (StackOverflowError e) {
                throw new NotImplementedException("c lds with explicit stack needed to pass this test");
            }
        });
        return statistics;
    }


    /**
     * Effectively start a depth first search
     * looking for every solution.
     *
     * @return an object with the statistics on the search
     */
    public SearchStatistics solve() {
        SearchStatistics statistics = new SearchStatistics();
        return solve(statistics, stats -> false);
    }

    /**
     * Effectively start a depth first search
     * with a given predicate called at each node
     * to stop the search when it becomes true.
     *
     * @param limit a predicate called at each node
     *             that stops the search when it becomes true
     * @return an object with the statistics on the search
     */
    public SearchStatistics solve(Predicate<SearchStatistics> limit) {
        SearchStatistics statistics = new SearchStatistics();
        return solve(statistics, limit);
    }

    /**
     * Executes a closure prior to effectively
     * starting a depth first search
     * with a given predicate called at each node
     * to stop the search when it becomes true.
     * The state manager saves the state
     * before executing the closure
     * and restores it after the search.
     * Any {@link InconsistencyException} that may
     * be throw when executing the closure is also catched.
     *
     * @param limit a predicate called at each node
     *             that stops the search when it becomes true
     * @param subjectTo the closure to execute prior to the search starts
     * @return an object with the statistics on the search
     */
    public SearchStatistics solveSubjectTo(Predicate<SearchStatistics> limit, Procedure subjectTo) {
        SearchStatistics statistics = new SearchStatistics();
        sm.withNewState(() -> {
            try {
                subjectTo.call();
                solve(statistics, limit);
            } catch (InconsistencyException e) {
            }
        });
        return statistics;
    }

    public SearchStatistics solveRestarts(Predicate<SearchStatistics> limit, int nbFailCutof, double restartFactor) {
        throw new NotImplementedException();
    }

    public void initializeImpact(IntVar... x) {
        throw new NotImplementedException();
    }

    public void initializeImpactDomains(IntVar... x) {
        throw new NotImplementedException();
    }

    /**
     * Effectively start a branch and bound
     * depth first search with a given objective.
     *
     * @param obj the objective to optimize that is tightened each
     *            time a new solution is found
     * @return an object with the statistics on the search
     */
    public SearchStatistics optimize(Objective obj) {
        return optimize(obj, stats -> false);
    }

    /**
     * Effectively start a branch and bound
     * depth first search with a given objective
     * and with a given predicate called at each node
     * to stop the search when it becomes true.
     *
     * @param obj the objective to optimize that is tightened each
     *            time a new solution is found
     * @param limit a predicate called at each node
     *             that stops the search when it becomes true
     * @return an object with the statistics on the search
     */
    public SearchStatistics optimize(Objective obj, Predicate<SearchStatistics> limit) {
        SearchStatistics statistics = new SearchStatistics();
        if (!obj.problemIsBound()) { // avoid in special case of problem solved by propagation alone
            onSolution(() -> {
                if (obj.tracingOptimization()) {
                    System.out.println(" (solution found in " + statistics.numberOfFailures() + " fails, " + statistics.numberOfNodes() + " choices and " + statistics.timeElapsed() + " msecs)");
                }
                obj.tighten();
            });
        }
        else {
            onSolution(() -> {
                if (obj.tracingOptimization()) {
                    System.out.println(" (solution found in " + statistics.numberOfFailures() + " fails, " + statistics.numberOfNodes() + " choices and " + statistics.timeElapsed() + " msecs)");
                }
            });
        }
        return solve(statistics, limit);
    }

    /**
     * Executes a closure prior to effectively
     * starting a branch and bound depth first search
     * with a given objective to optimize
     * and a given predicate called at each node
     * to stop the search when it becomes true.
     * The state manager saves the state
     * before executing the closure
     * and restores it after the search.
     * Any {@link InconsistencyException} that may
     * be throw when executing the closure is also catched.
     *
     * @param obj the objective to optimize that is tightened each
     *            time a new solution is found
     * @param limit a predicate called at each node
     *             that stops the search when it becomes true
     * @param subjectTo the closure to execute prior to the search starts
     * @return an object with the statistics on the search
     */
    public SearchStatistics optimizeSubjectTo(Objective obj, Predicate<SearchStatistics> limit, Procedure subjectTo) {
        SearchStatistics statistics = new SearchStatistics();
        sm.withNewState(() -> {
            try {
                subjectTo.call();
                optimize(obj, limit);
            } catch (InconsistencyException e) {
            }
        });
        return statistics;
    }


    private void lds(SearchStatistics statistics, Predicate<SearchStatistics> limit) {
        if (limit.test(statistics))
            throw new StopSearchException();
        Procedure[] branches = LDSbranching.get();
        if (branches.length == 0) {
            statistics.incrSolutions();
            notifySolution();
        } else {
            for (Procedure b : branches) {
                sm.withNewState(() -> {
                    try {
                        statistics.incrNodes();
                        b.call();
                        lds(statistics, limit);
                    } catch (InconsistencyException e) {
                        statistics.incrFailures();
                        notifyFailure();
                    }
                });
            }
        }
    }

    

}
