package minicpbp.examples.ClassicalAIPlanning;

import minicpbp.engine.constraints.Sum;
import minicpbp.engine.core.Constraint;
import minicpbp.engine.core.IntVar;
import minicpbp.engine.core.Solver;
import minicpbp.util.Automaton;

import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.IntStream;

import static minicpbp.cp.Factory.*;

public class PlanningModelNoOp {

    // decision variables defining the sequential plan: action[0],action[1],...,action[length-1]
    private final Solver cp;
    private final IntVar[] action;
    private final IntVar planCost;

    public PlanningModelNoOp(PlanningInstance instance, int upper_bound, int lower_bound, String lpProblemString){

        cp = makeSolver();
        action = new IntVar[upper_bound];

        for (int i = 0; i < upper_bound; i++) {
            action[i] = makeIntVar(cp, 0, instance.nbActions - 1);
            action[i].setName("action" + i);
        }
        // objective to minimize
        planCost = makeIntVar(cp, lower_bound, upper_bound);
        planCost.setName("plan_cost");
        IntVar[] automataCosts = new IntVar[instance.nbOptimizationConstraints];
        int k = 0;
        // for each component of factored transition system...
        for (int i = 0; i < instance.automaton.length; i++) {
            Automaton automaton = instance.automaton[i];
            IntVar[] localAction;
            if (automaton.doesNeedMapping()) {
                localAction = new IntVar[upper_bound];
                // map the original actions to these local actions
                for (int j = 0; j < upper_bound; j++) {
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
                    IntVar automatonCost = makeIntVar(cp, 0, lower_bound);
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
                planCost.assign(upper_bound);
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
                IntVar[] planActionCost = makeIntVarArray(cp, upper_bound, instance.maxActionCost + 1);
                for (int i = 0; i < upper_bound; i++) {
                    cp.post(element(instance.actionCost, action[i], planActionCost[i]));
                }
                cp.post(sum(planActionCost, planCost));
        }

        // To store the counts of each operator (action)
        IntVar[] operatorCounts = makeIntVarArray(cp, instance.nbActions, 0, upper_bound);
        for (int i = 0; i < instance.nbActions; i++) {
            operatorCounts[i].setName("op_count_action_" + i);
            cp.post(among(action, i, operatorCounts[i]));
        }

        cp.post(new Sum(operatorCounts, upper_bound));

        parseAndAddOperatorCountingConstraints(lpProblemString, operatorCounts);
    }

    /**
     * Parses the LP problem string and adds operator counting constraints to the model.
     *
     * @param lpProblemString The full LP problem string.
     * @param operatorCounts  An array of IntVar representing the counts of each operator (action).
     */
    private void parseAndAddOperatorCountingConstraints(String lpProblemString, IntVar[] operatorCounts) {
        Pattern subjectToPattern = Pattern.compile("Subject To\\s*(.*?)\\s*End", Pattern.DOTALL);
        Matcher subjectToMatcher = subjectToPattern.matcher(lpProblemString);

        if (subjectToMatcher.find()) {
            String subjectToBlock = subjectToMatcher.group(1);
            String[] constraintLines = subjectToBlock.split("\\n");

            // Pattern for a constraint line: cX: <expression> <operator> <rhs>
            Pattern constraintPattern = Pattern.compile("c\\d+:\\s*(.*?)\\s*(<|>|>=|<=|=)\\s*(-?\\d+)");
            // Pattern for individual terms: [+|-]?\s*(\d*\s*x\d+)
            // This captures the optional sign, optional coefficient, and the 'x' variable
            Pattern termPattern = Pattern.compile("([+-]?\\s*\\d*\\s*x\\d+)");

            String previousLine = "";
            for (String line : constraintLines) {
                line = line.trim();
                if (line.isEmpty()) continue;

                line = previousLine + line;
                Matcher constraintMatcher = constraintPattern.matcher(line);
                if (! constraintMatcher.matches()) {
                    previousLine = line;
                }
                else {
                    previousLine = "";
                    String expression = constraintMatcher.group(1);
                    String operator = constraintMatcher.group(2);
                    int rhs = Integer.parseInt(constraintMatcher.group(3));

                    List<IntVar> terms = new ArrayList<>();

                    Matcher termMatcher = termPattern.matcher(expression);
                    while (termMatcher.find()) {
                        String term = termMatcher.group(1).trim();

                        Pattern coeffPattern = Pattern.compile("([+-])?\\s*(\\d+)?\\s*x(\\d+)");
                        Matcher coeffMatcher = coeffPattern.matcher(term);

                        if(coeffMatcher.matches()){
                            int coeff = coeffMatcher.group(2) != null ? Integer.parseInt(coeffMatcher.group(2)) : 1;
                            coeff *= coeffMatcher.group(1) != null && coeffMatcher.group(1).equals("-") ? -1 : 1;;
                            int varID = Integer.parseInt(coeffMatcher.group(3)) - 1;

                            terms.add(mul(operatorCounts[varID], coeff));
                        }
                    }

                    IntVar[] expr = terms.toArray(new IntVar[0]);

                    IntVar sum = expr.length == 1 ? expr[0] : sum(terms.toArray(new IntVar[0]));

                    switch (operator) {
                        case ">":
                            sum.remove(rhs);
                            sum.removeBelow(rhs);
                            break;
                        case ">=":
                            sum.removeBelow(rhs);
                            break;
                        case "<":
                            sum.remove(rhs);
                            sum.removeAbove(rhs);
                            break;
                        case "<=":
                            sum.removeAbove(rhs);
                            break;
                        case "=":
                            sum.assign(rhs);
                            break;
                        default:
                            System.err.println("Unsupported operator: " + operator + " in constraint: " + line);
                            break;
                    }
                }
            }
        } else {
            System.err.println("Warning: 'Subject To' block not found in the LP problem string.");
        }
    }

    private IntVar[] concatenate(IntVar[] a, IntVar[] b) {
        IntVar[] result = Arrays.copyOf(a, a.length + b.length);
        System.arraycopy(b, 0, result, a.length, b.length);
        return result;
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
