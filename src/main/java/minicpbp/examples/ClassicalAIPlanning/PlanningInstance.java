package minicpbp.examples.ClassicalAIPlanning;

import minicpbp.util.Automaton;
import minicpbp.util.io.InputReader;

public class PlanningInstance {

    public int minPlanLength;
    public int maxPlanLength;
    public int nbActions;
    public int objectiveCombinator; // 0 if no objective; 1/2/3 for same/sum/max
    public int lowerBoundB;
    public int lowerBoundC;
    public int nbAutomata;
    public Automaton[] automaton;
    public int[] actionCost;
    public int maxActionCost;
    public int nbOptimizationConstraints;

    public PlanningInstance(String path){
        InputReader reader = new InputReader(path);

        minPlanLength = reader.getInt();
        maxPlanLength = reader.getInt();
        nbActions = reader.getInt();
        objectiveCombinator = reader.getInt();
        if (objectiveCombinator==0) {// no action costs
            lowerBoundB = 0;
            lowerBoundC = 1; // lower (and upper) bound is length
            maxActionCost = 1;
        }
        else {
            lowerBoundB = reader.getInt();
            lowerBoundC = reader.getInt();
            maxActionCost = 0;
        }
        nbAutomata = reader.getInt();
        if(objectiveCombinator==4){
            actionCost = new int[nbActions];
            for (int i = 0; i < nbActions; i++) {
                actionCost[i] = reader.getInt();
            }
        }
        automaton = new Automaton[nbAutomata];
        nbOptimizationConstraints = 0;
        for(int i=0; i<nbAutomata; i++) {
            automaton[i] = new Automaton(reader,nbActions);
            if (automaton[i].optimizationConstraint()) {
                nbOptimizationConstraints++;
                int[] localActionCost = automaton[i].actionCost();
                for (int j = 0; j < automaton[i].nbLocalActions(); j++) {
                    if (localActionCost[j] > maxActionCost){
                        maxActionCost = localActionCost[j];
                    }
                }
            }
        }
    }

    public int getUpperBoundCost(){
        return maxPlanLength * maxActionCost + 1;  // trivial strict upper bound
    }

    public int getLowerBoundCost(int planLength){
        return lowerBoundB + lowerBoundC * planLength;
    }

}
