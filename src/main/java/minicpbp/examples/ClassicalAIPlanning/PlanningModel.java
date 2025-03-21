package minicpbp.examples.ClassicalAIPlanning;

import minicpbp.engine.core.IntVar;
import minicpbp.engine.core.Solver;
import minicpbp.util.Automaton;

import static minicpbp.cp.Factory.*;

public class PlanningModel {

    // decision variables defining the sequential plan: action[0],action[1],...,action[length-1]
    private final Solver cp;
    private final IntVar[] action;
    private final IntVar planCost;

    public PlanningModel(PlanningInstance instance, int length, int currentBestPlanCost){

        cp = makeSolver();
        action = new IntVar[length];

        for (int i = 0; i < length; i++) {
            action[i] = makeIntVar(cp, 0, instance.nbActions - 1);
            action[i].setName("action" + i);
        }
        // objective to minimize
        planCost = makeIntVar(cp, instance.getLowerBoundCost(length), currentBestPlanCost - 1);
        planCost.setName("plan_cost");
        IntVar[] automataCosts = new IntVar[instance.nbOptimizationConstraints];
        int k = 0;
        // for each component of factored transition system...
        for (int i = 0; i < instance.automaton.length; i++) {
            Automaton automaton = instance.automaton[i];
            IntVar[] localAction;
            if (automaton.doesNeedMapping()) {
                localAction = new IntVar[length];
                // map the original actions to these local actions
                for (int j = 0; j < length; j++) {
                    localAction[j] = makeIntVar(cp, 0, automaton.nbLocalActions() - 1);
                    localAction[j].setName("automaton" + i + "_action" + j);
                    cp.post(table(new IntVar[]{action[j], localAction[j]}, automaton.actionMap()));
                }
            } else {
                // no need to map local actions if there is no mapping
                localAction = action;
            }
            if (automaton.optimizationConstraint()) {
                if (instance.objectiveCombinator == 2 || instance.objectiveCombinator == 3) {
                    IntVar automatonCost = makeIntVar(cp, 0, currentBestPlanCost);
                    automatonCost.setName("automaton"+i+"_cost");
                    automataCosts[k++] = automatonCost;
                    cp.post(costRegular(localAction, automaton.transitionFct(), automaton.initialState(),
                            automaton.goalStates(), automaton.actionCost(), automatonCost));
                } else { // objectiveCombinator == 1 i.e. same
                    cp.post(costRegular(localAction, automaton.transitionFct(), automaton.initialState(),
                            automaton.goalStates(), automaton.actionCost(), planCost));
                }
            } else {
                cp.post(regular(localAction, automaton.transitionFct(), automaton.initialState(),
                        automaton.goalStates()));
            }
        }
        // express planCost as combination of automataCost
        switch (instance.objectiveCombinator) {
            case 0: // no objective
                planCost.assign(length);
                break;
            case 1: // same; already taken care of
                break;
            case 2: // sum
                cp.post(sum(automataCosts, planCost));
                break;
            case 3: // max
                cp.post(maximum(automataCosts, planCost));
                break;
            case 4: // action cost
                IntVar[] planActionCost = makeIntVarArray(cp, length, instance.maxActionCost + 1);
                for (int i = 0; i < length; i++) {
                    cp.post(element(instance.actionCost, action[i], planActionCost[i]));
                }
                cp.post(sum(planActionCost, planCost));
        }
    }

    public Solver getSolver(){
        return cp;
    }

    public IntVar[] getAction(){
        return action;
    }

    public IntVar getPlanCost(){
        return planCost;
    }
}
